# PCES on wadamesh (proof of concept)

This fork adds client support for **PCES**, the Path Computation Element Service a
Commsol station offers on air. You ask "what is the best path from node A to node
B?". The station answers from the link-quality database it builds from TRACE
results, and you can install the answer as a contact's route. The protocol of
record is Commsol's `docs/pces.md`. Its reference encoder and decoder are in
`src/commsol/pces.py`.

PCES is **native firmware, not a Lua app**. wadamesh deliberately keeps
transmissions out of Lua: apps cannot log in to nodes or send them requests. The
Lua SDK is unchanged.

## Where it is

| Where | What |
|---|---|
| App drawer: **PCES** tile | The PCES page. Choose a target, then tap **Find Best Route**. |
| Contact action sheet: **PCES route** | The same page, with that contact already the target. |
| Contacts: **PCES** badge | Repeaters and rooms advertising PCES (feature 1 `0xC5xx`, bit 1). |
| Contacts: **PCES 72** badge (filled; the accepted route's score) | Contacts whose stored route is the one PCES installed. It goes when MeshCore learns a different path, or on Restore / Reset path. |
| Contacts filter: **PCES servers** | Only those. |
| In the background | Asks a PCES server for the best path to itself when its advert arrives (see below). |

## The best-path test

This mirrors the uConsole Console's PCES page (meshcore-uconsole, `meshcore/pces.py`).
Nothing is sent before **Find Best Route**. Each step is logged on the page.

1. **First hop R.** Candidates, up to 3:
   - the target's current first hop;
   - repeaters this radio hears directly: Discover hits, best SNR first;
   - 0-hop repeater contacts, most recently heard first.

   PCES must know R, so several are kept to try.
2. **Server.** A guest login (blank password; a station admits it as a PCES-only guest), reused for 15 minutes. Then **INFO**: link count, data window, newest measurement age, SF and decode limit.
3. **Route.** PATH from R to the target. For a repeater or room the target is itself, and its own hash ends the path rather than being a hop. For a companion, which never relays, the target is its last relay, so a companion needs a known route first. When PCES doesn't know R, or has no path from it, the next R is tried.
4. **Accept.** PCES's own data must back the route:
   - the weakest leg is at least **3 dB above the decode limit** INFO reported;
   - INFO's newest measurement is at most **60 minutes** old.

   If accepted, the route is installed as the target's out path and the previous one is kept. If not, the reason is shown, with **Use anyway**.

The uConsole also sends a test over the new route. Here it stays "unverified" until
use confirms it. MeshCore floods and relearns a route that stops delivering.

**Accepted PCES routes** lists what was installed, each with **Restore**. The list is
kept until reboot. **Server info** runs INFO on its own.

**Trace route** (on the page, for the target) sends a TRACE out along the target's
stored route and back, and logs the SNR at each hop and the round trip. A repeater or
room target turns the trace itself. A companion doesn't relay traces, so its trace
turns at its last relay. There is one trace every 15 s, and no reply within 20 s is
reported.

### Default route vs PCES route

**Compare routes** (on the page, once PCES has an answer or an installed route for the
target) measures both on air, the same way:
- **Default** is the route MeshCore had before PCES replaced it, or the stored route
  when the answer isn't installed.
- **PCES** is the PCES route.

It sends traces out along each and back, alternating, 2 each, 15 s apart. Each route
shows how many traces came back, the average round trip, and the weakest hop's SNR
(every hop's reading, out and back). A verdict follows: deliveries first, then the
weakest hop (a 2 dB or larger difference), else "about even". Nothing is installed or
changed. A default that floods has no fixed route to trace, so only the PCES route is
traced.

### PCES routes are pinned

A route PCES installs is **pinned** until you **Restore** it, or until **Reset path**
on the contact. A pin holds a fingerprint of the route it was given, so it lapses (and
the score badge goes) once MeshCore stores a different path:

- **Ping** sends along it. If no reply comes, it reports "No reply" instead of the
  usual fallback, which erases the stored route and floods a login to learn a new
  one.
- **Logins** (the admin app, room joins, PCES's own guest login) go along it instead
  of erasing it first. The PCES guest login never erases a route, pinned or not.
- MeshCore's own path learning is unchanged: if the route stops delivering, a flood
  still teaches a new one.

Pins live in RAM, like the Restore list.

### Server

A server you pick is saved in NVS (`pces_srv`) and kept across reboots, even while it
is out of contacts. **Clear saved server** forgets it. With nothing saved, the page
uses the first repeater or room that advertises PCES. The marker is optional, so with
none advertised it lists your repeaters and rooms to pick from.

### Automatic: best path to a PCES server when its advert arrives

When an advert with the PCES marker arrives through relays, the client asks that
server for the best path to itself. A is the relay that delivered the advert (the
last hash on its flood path), and B is the station itself (length 0). This is the
spec's worked example.

- An advert heard direct is ignored: there is nothing to improve.
- Each server is asked at most once per 6 hours. There is a single attempt and no retry.
- The ask never runs while the PCES page is open or another request is in flight.
- If the server has no route yet, the answer is installed, with a toast. Otherwise the route is left alone.

### On-air etiquette (Commsol spec)

- Requests are sent only on Find Best Route, Server info, or a PCES server's advert. Never on a timer or in a loop over contacts.
- No request is retried. A run tries at most 3 first hops, each once.
- At most one PCES request every **12 s**, so 5 a minute, matching the server's limit. The login doesn't count towards it.
- PCES requests share the UI's single-flight request slot with status, telemetry and admin requests and never clobber them.

## Code

| File | What |
|---|---|
| `src/helpers/PcesProto.h` | Pure codec: INFO/PATH/LINK, status texts, the feature-1 marker, answer-to-route conversion. |
| `src/ui-touch/PcesApp.h` | The page, the four-step run, the automatic ask, the service tick (started from `UITask::loop`). Included into `UITask.cpp` after `RepeaterAdminApp.h`. |
| `src/MyMesh.h` | `onAdvertRecv` override that keeps advert feature 1 in a RAM cache (`advertFeat1`, never read from storage) and records PCES server adverts heard through relays (`takePcesAdvert`). Also `uiSetContactPath()`. |
| `src/helpers/esp32/TouchPrefsStore.*` | `touchPrefsGet/SetPcesServer`: the saved server. |
| `src/ui-touch/UITask.cpp` | The drawer tile, the contact-sheet button, the badge and filter, the reply hooks, Back/Esc. |
| `web/mesh/*` | Stand-ins for the browser and host builds. `WADA_DEMO_PCES=1`: a canned PCES station on Hilltop Repeater, which is marked as a PCES server, and a 2-hop route to Diego. `WADA_DEMO_PCES_ADVERT=1`: a fake PCES server advert 8 s after boot. |

## Testing

- **Codec:**

  ```
  c++ -std=c++17 -Wall -Wextra -I src test/test_pces_proto.cpp -o /tmp/t
  /tmp/t
  ```

- **Simulator:** build with `web/build.sh host`. Each script needs a fresh `WADA_DATA_DIR`. Each script's header gives its exact command and what to expect.
  - `web/scripts/pces-route.wada`: the PCES tile, choose target, the full four-step run.
  - `web/scripts/pces-filter.wada`: the badge and the PCES servers filter.
  - `web/scripts/pces-auto.wada`: the automatic ask on a server's advert.
- **Device:** `pio run -e LilyGo_TDeck_companion_radio_touch`. Not yet tested against a real station.

## Not done in this proof of concept

- No automatic test-send over the new route (it stays "unverified"); **Trace route** is the manual test.
- LINK is encoded and decoded but has no screen.
- The accepted-routes list, for Restore, lives in RAM only.
- The target picker lists the 64 most recently heard contacts. For any other, use **PCES route** on the contact.
