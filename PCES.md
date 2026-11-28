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
| Contacts filter: **PCES servers** | Only those, plus your saved server even if its station sends no marker. |
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
3. **Route.** PATH BOTH WAYS from R to the target: a path measured in both directions, each leg scored by its weaker one, so its reverse is safe to seed as the far end's way back (below). For a repeater or room the target is itself, and its own hash ends the path rather than being a hop. For a companion, which never relays, the target is its last relay, so a companion needs a known route first. When PCES doesn't know R, or has no path from it, the next R is tried.
   - No R has a path both ways: one plain **PATH** from the first R that PCES knows. That path is scored in our direction only, so it is installed but no return path is seeded.
   - A server from before PATH BOTH WAYS (status 9, older Commsol): plain PATH from each R, as before, and no return path is seeded.

   With 3 first hops that is at most 5 requests with INFO, the server's limit a minute (not counting retries of lost ones).
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

### The far end's way back (return-path seed)

Installing a route only fixes your direction. The far end still answers along its own
stored route to you, or floods; MeshCore ignores the path a direct request came in on.
So whenever PCES installs a route from a PATH BOTH WAYS answer (Find Best Route, Use
anyway, or an automatic ask), it then **seeds the far end's return path**. A route from a
plain PATH answer is scored in your direction only: its reverse was never checked, so it
isn't seeded, and the far end learns its own way back as usual.

1. A repeater or room keeps a route only for a client on its access list. So unless a
   guest session with it is fresh, a blank guest login goes first, along the new route,
   retried up to 3 times.
2. 2.5 s after LOGIN_OK (straight away for a companion), one standard MeshCore **PATH**
   packet goes **direct along the new route**. It is the packet
   `BaseChatMesh::handleReturnPathRetry` sends, and its content is the route reversed.
3. The far end stores that as its route back to you: a repeater or room in
   `onPeerPathRecv`, a companion for a known contact. This is stock firmware, nothing
   custom on the far end.

The page logs it, e.g. "Return path seeded: Diego will reply via 7F33 > F733 > BF78". The
far end keeps it until it learns another route; a route that stops working repairs
itself by flooding. A companion must have you as a contact to accept it. Restore puts
your route back but doesn't re-seed the far end.

### Kept across reboots

These survive a reboot, stored in NVS:
- the saved server (`pces_srv`);
- the accepted PCES routes (`pces_acc`), for Restore, the score badge and the benchmark;
- the PCES server markers heard (`pces_mk`), for the badge and filter before the next advert.

- the saved server's name and type (`pces_si`). If the saved server is missing from
  contacts after a reboot (auto-purge, or anything else), it's added back from these at
  boot, and its next advert refreshes it.

After a reboot, a route PCES installed is pinned again only if it is still the contact's
stored route. Badges redraw as soon as a marker or an accepted route changes; they don't
wait for the contact list's periodic refresh.

### Route benchmark (dev tool): default route vs PCES route

The **Route benchmark (dev)** card is always shown for a target. Until PCES has an
answer or an installed route for it, the card says to tap **Find Best Route** first. It measures both routes on air, the same way, over many
samples:

- **Default** is MeshCore's own fixed path to the target: the route it had before PCES
  replaced it, or the stored route when the answer isn't installed. **PCES** is the PCES
  route. A flood is never benchmarked:
  - When no fixed default is known, **Run** first finds one with a single flooded path
    discovery. That's what MeshCore's own path discovery sends: a flooded guest login for
    a repeater or room, otherwise a flooded base-telemetry request. The path the reply
    carries back becomes the default. The target's stored route, including an installed
    PCES route and its pin, is put back as it was.
  - No reply within 30 s means no default path to compare with. It is not retried.
  - If the default path is the PCES route, there is nothing to compare, and the card says so.
  - A companion whose default path is direct (no relays) can't be traced, so there's
    nothing to sample.
- **Samples per route** is 10, 25, 50 or 100. **Gap between traces** is 15, 30 or 60 s.
  Traces alternate, default first. The button shows the expected duration (2 × samples
  × gap).
- Each sample is a TRACE out along the route and back. It records delivered or not,
  the round trip, and the weakest hop's SNR (every hop's reading, out and back).
- Per route the card shows:
  - delivery rate with a **Wilson 95% interval**;
  - RTT mean, SD, median and p90;
  - weakest-hop SNR mean, SD and worst.
- Differences are reported as PCES minus default, each with a two-sided p:
  - delivery: two-proportion z-test;
  - weakest-hop SNR and RTT: **Welch's t-test**, Student t with Welch-Satterthwaite df.

  A plain-language line names only what is significant at p < 0.05.
- Every sample is appended to `/pces/bench-<epoch>.csv` on the UI data filesystem
  (internal flash on the T-Deck: **Files > Internal**, or USB Files). The CSV starts
  with a commented header (target, both routes, settings), then the columns
  `seq,ms,route,delivered,rtt_ms,weakest_snr_db,hop_snrs_db`.
- It runs with the page closed, and **Stop benchmark** ends it. Nothing is installed or
  changed.

The statistics are in `src/helpers/PcesStats.h`, with the host test
`test/test_pces_stats.cpp`, checked against R's outputs.

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

Pins are rebuilt from the saved accepted routes after a reboot.

### Server

**Server check** (on the page, under the server) shows what this device knows about it:
- its contact key and type;
- its advertised name (a contact with no name isn't kept at reboot);
- the PCES marker last heard (or none since boot);
- when it was last updated, and its advert clock against ours;
- whether it's a favourite, with **Keep it (favourite)** (favourites are never
  auto-purged or evicted);
- the auto-purge setting.

The Commsol station only sends the marker when `repeater.advert_feature1` is set to
0xC502 (openhop-advert-features.patch). With it unset, there is no marker, so no badge
and no filter match. The saved server still works.

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
- Each server is asked at most once per 6 hours from its adverts. Each ask retries as above.
- The ask never runs while the PCES page is open or another request is in flight.
- The answer is installed, with a toast, if the server has no route yet or its route is
  one PCES installed earlier. A route MeshCore learned itself is left alone.

**When you move.** Every 30 s the T-Deck compares its GPS position with where it was
when the server's route was last asked. After **0.5 km** it asks again, at most once
every 2 minutes, and installs the answer by the same rule. The first hop comes from the
Find Best Route candidates (current first hop, then repeaters heard directly). PCES
servers are fixed stations, so only this device's movement counts. It needs a GPS fix.

### On-air etiquette (Commsol spec)

- Requests are sent only on Find Best Route, Server info, or a PCES server's advert. Never on a timer or in a loop over contacts.
- A request with no answer is retried, up to **3 tries per step** (login, INFO, PATH), in
  Find Best Route and the automatic asks alike. On LoRa that is almost always a lost
  packet, ours or the reply, because a found PCES server is all but always up. Each try
  waits 15 s for a login and 20 s for a request. A late reply to an earlier try is
  ignored. A run tries at most 3 first hops.
- At most one PCES request every **12 s**, so 5 a minute, matching the server's limit. The login doesn't count towards it.
- PCES requests share the UI's single-flight request slot with status, telemetry and admin requests and never clobber them.

## Code

| File | What |
|---|---|
| `src/helpers/PcesProto.h` | Pure codec: INFO/PATH/PATH BOTH WAYS/LINK, status texts, the feature-1 marker, answer-to-route conversion. |
| `src/ui-touch/PcesApp.h` | The page, the four-step run, the automatic ask, the service tick (started from `UITask::loop`). Included into `UITask.cpp` after `RepeaterAdminApp.h`. |
| `src/MyMesh.h` | `onAdvertRecv` override that keeps advert feature 1 in a RAM cache (`advertFeat1`, never read from storage) and records PCES server adverts heard through relays (`takePcesAdvert`). Also `uiSetContactPath()`. |
| `src/helpers/esp32/TouchPrefsStore.*` | `touchPrefsGet/SetPcesServer`: the saved server. |
| `src/ui-touch/UITask.cpp` | The drawer tile, the contact-sheet button, the badge and filter, the reply hooks, Back/Esc. |
| `web/mesh/*` | Stand-ins for the browser and host builds. `WADA_DEMO_PCES=1`: a canned PCES station on Hilltop Repeater, which is marked as a PCES server, and a 2-hop route to Diego. `WADA_DEMO_PCES_ADVERT=1`: a fake PCES server advert 8 s after boot. `WADA_DEMO_PCES_BOTH=none` / `old`: the canned station has no path both ways / predates PATH BOTH WAYS. |

## Testing

- **Codec and statistics:**

  ```
  c++ -std=c++17 -Wall -Wextra -I src test/test_pces_proto.cpp -o /tmp/t
  /tmp/t
  c++ -std=c++17 -Wall -Wextra -I src test/test_pces_stats.cpp -o /tmp/s
  /tmp/s
  ```

- **Simulator:** build with `web/build.sh host`. Each script needs a fresh `WADA_DATA_DIR`. Each script's header gives its exact command and what to expect.
  - `web/scripts/pces-route.wada`: the PCES tile, choose target, the full four-step run.
  - `web/scripts/pces-filter.wada`: the badge and the PCES servers filter.
  - `web/scripts/pces-auto.wada`: the automatic ask on a server's advert.
- **Device:** `pio run -e LilyGo_TDeck_companion_radio_touch`. Not yet tested against a real station.

## Not done in this proof of concept

- No automatic test-send over the new route (it stays "unverified"); **Trace route** is the manual test.
- LINK is encoded and decoded but has no screen.
- The target picker lists the 64 most recently heard contacts. For any other, use **PCES route** on the contact.
