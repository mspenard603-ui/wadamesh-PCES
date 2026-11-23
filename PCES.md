# PCES on wadamesh (proof of concept)

This fork adds client support for **PCES**, the Path Computation Element Service a
Commsol station offers on air. You ask "what is the best path from node A to node
B?". The station answers from the link-quality database it builds from TRACE
results, and you can install the answer as a contact's route. The protocol of
record is Commsol's `docs/pces.md`. Its reference encoder and decoder are in
`src/commsol/pces.py`.

PCES is **native firmware, not a Lua app**. wadamesh deliberately keeps
transmissions out of Lua: apps cannot log in to nodes or send them requests. PCES
needs both, so it lives in the firmware next to the repeater admin app and reuses
its plumbing. The Lua SDK is unchanged.

## Using it

1. Open a contact's action sheet (tap it in Contacts) and choose **PCES route**.
2. **Server.** A server you pick is saved and kept across reboots; **Clear saved server** forgets it. With nothing saved, the screen uses the first repeater or room that advertises the PCES marker. Its advert feature 1 is `0xC5xx` with bit 1 set, and the station's repeater identity advertises `0xC502`. The marker is optional, so when no contact advertises it, the screen lists your repeaters and rooms to pick from. **Server info** asks the station for INFO: link count, data window, newest measurement age and SF.
3. **Ask PCES for a route.** Following the spec, A is your first hop, the first relay of the contact's current route. B is the contact itself when it is a repeater or room, otherwise the last relay of its route. A contact with no route yet can't be planned: message it once so MeshCore learns a path.
4. The first ask logs in to the server with a blank password, which the station accepts as a PCES-only guest. It waits 2.5 s, so the request doesn't collide with the re-flooded login reply, then sends `C5 01 01 [width] A B`. The guest session is reused for 15 minutes.
5. **Answer.** The screen shows the metric (0 to 100), the weakest-leg SNR, the PCES path and the route that would be installed. Nothing changes until you tap **Use this route**. The replaced route is kept, and **Restore route** puts it back.

### Automatic: best path to a PCES server when its advert arrives

When an advert carrying the PCES marker reaches the device through relays, the
firmware asks that server, unprompted, for the best path to itself. A is the relay
that handed us the advert, the last hash on its flood path. B is the station itself
(length 0). This is the spec's worked example. The ask uses the same login and
request as the screen.

- An advert heard direct (0 hops) is ignored: there is no path to improve.
- Each server is asked at most once per boot and then not again for 6 hours, whether or not it answered. There is a single attempt and no retry.
- A queued ask waits up to 60 s for a free moment. It never runs while the PCES screen is open, while anything else holds the request slot, or within 12 s of the last PCES request. Otherwise it is dropped.
- Answer, when the server contact has **no route yet**: the route is installed and a toast says so ("PCES: route to X installed (metric N)"). MeshCore still floods and relearns if it stops working.
- Answer, when the server contact **already has a route**: the route is left alone. The toast points to **PCES route** on that contact, where the answer is waiting with **Use this route**.
- The server must already be a contact, because a login needs its key. Adverts from nodes that aren't contacts are ignored.

The hook is `MyMesh::onAdvertRecv` → `takePcesAdvert()`. The ask runs from the service
tick in `PcesApp.h` (`pcesAutoTick`), which `UITask::loop` starts.

On-air etiquette from the Commsol spec, enforced by the screen and the automatic ask:
- It sends only on a tap or a PCES server's advert (rate-limited as above), never on a timer or in a loop over contacts.
- Each tap makes one attempt. No answer after 40 s is reported, never retried.
- At most one request every 12 s, so 5 a minute, matching the server's limit.
- It shares the UI's single-flight request slot, so it waits behind status, telemetry and admin requests and never clobbers them.

Contacts that advertise PCES (repeaters and rooms whose feature 1 is `0xC5xx` with
bit 1 set) carry a **PCES** badge after their name in the contact list.

## Code

| File | What |
|---|---|
| `src/helpers/PcesProto.h` | Pure codec: INFO/PATH/LINK encode and decode, status texts, the feature-1 marker, A/B planning and answer-to-route conversion. No Arduino or LVGL. |
| `src/ui-touch/PcesApp.h` | The screen. It is included into `UITask.cpp` after `RepeaterAdminApp.h` and shares its helpers. Login goes through `uiSendAdminLogin`, the request through `uiSendBinaryRequest`, and replies are routed from `UITask::onAdminLoginResult` / `onBinaryReply`. |
| `src/MyMesh.h` | `onAdvertRecv` override that keeps advert feature 1, which the core parses and drops, and records PCES server adverts heard through relays (`takePcesAdvert`). `advertFeat1()` reads it from a 64-entry RAM cache, falling back to the stored advert blob. `uiSetContactPath()` installs a route. |
| `src/helpers/esp32/TouchPrefsStore.*` | `touchPrefsGet/SetPcesServer`: the saved server, as a 32-byte key in NVS (`pces_srv`). |
| `src/ui-touch/UITask.cpp` | The PCES badge on contact rows, the "PCES route" button on the contact action sheet, the reply hooks, and Back/Esc via the popup registry. |
| `web/mesh/WebMesh.*` | Stand-ins for the browser and host builds. Routes are only mirrored there. |
| `web/mesh/DemoBackend.cpp` | With `WADA_DEMO_PCES=1`, Hilltop Repeater answers as a canned PCES station and Diego has a 2-hop route to improve. `WADA_DEMO_PCES_ADVERT=1` (in `WebMesh.cpp`) fakes a PCES server advert. |

## Testing

- **Codec, host:**

  ```
  c++ -std=c++17 -Wall -Wextra -I src test/test_pces_proto.cpp -o /tmp/t
  /tmp/t
  ```

  This checks the spec's worked example, LINK, INFO, error statuses, malformed replies, A/B planning, and answers that must be refused (wrong width, not starting at your first hop).
- **Whole screen, host simulator:** run `web/build.sh host`, then:

  ```
  WADA_DEMO_PCES=1 web/build-host/wadamesh-host --headless --script web/scripts/pces-route.wada --shots /tmp/pces
  ```

  This drives open → choose server → ask → answer → install → restore → the 12 s floor, and screenshots each step.
- **Automatic ask, host simulator:**

  ```
  WADA_DEMO_PCES=1 WADA_DEMO_PCES_ADVERT=1 web/build-host/wadamesh-host --headless --script web/scripts/pces-auto.wada --shots /tmp/pces
  ```

  A fake advert 8 s after boot marks Hilltop Repeater as a PCES server heard via Harbor Room. Expect the "route installed" toast, then Hilltop's PCES screen showing that route.
- **Device:** `pio run -e LilyGo_TDeck_companion_radio_touch` builds. The other boards and hardware are untested. You need a Commsol station in range, running and able to transmit.

## Not done in this proof of concept

- LINK is encoded and decoded but has no screen.
- The saved "Restore" route lives in RAM only. It is lost on reboot, and only the last replaced route is kept.
- Not yet tested on hardware or against a real station.
