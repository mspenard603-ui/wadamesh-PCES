# PCES on wadamesh (proof of concept)

This fork adds client support for **PCES**, the Path Computation Element Service a
Commsol station offers on air. You ask "what is the best path from node A to node
B?". The station answers from the link-quality database it builds from TRACE
results, and you can install the answer as a contact's route. The protocol of
record is Commsol's `docs/pces.md`. Its reference encoder and decoder are in
`src/commsol/pces.py`.

PCES itself is a Lua app, `deploy/apps/pces`. The firmware gains a small,
consent-gated SDK extension that any client of a node-hosted service can use.
PCES is the first such client.

## SDK additions (`wada.mesh`, boards with `sdk_ext`)

| Call | What it does | Grant |
|---|---|---|
| `contacts()[i].feat1` | The advert feature-1 word the node last announced (0 = none). Commsol nodes advertise `0xC5xx`, and bit 1 means "answers PCES". | none (it is broadcast) |
| `login(who [, password])` | Login to a repeater or room. The result arrives in `app.on_login{pubkey, ok, perms, err}` within 20 s. Blank = guest; the Commsol station admits a blank login as a PCES-only guest. | requests |
| `request(who, data [, timeout_ms])` | Sends a binary REQ, `data[1]` = request type. Returns `"direct"`/`"flood"`. The answer (minus its 4-byte tag) arrives in `app.on_reply{pubkey, ok, data, rtt_ms, err}`. Default timeout 30 s, at most 40 s. | requests |
| `route(who)` | `{hops = {"3a7d", ...}, hash_size = 2}`, or `false` when the contact has no route (floods). | none |
| `set_route(who, hops)` | Installs `hops` as the contact's out path; `{}` = direct, `nil` = forget (flood and relearn). | routes |
| `path_hash_size()` | Bytes per hop this node uses, 1 to 3. | none |

`who` is a contact name, or the 8-hex pubkey prefix that `contacts()` reports.
Check `wada.sys.caps().node_requests` and `.feat1` before using these calls.

Safety rules match the rest of the SDK:
- The first `login`/`request` from an app asks the user, naming the app. The first `set_route` asks separately. A refusal is remembered. Both grants appear under Settings > App permissions.
- `login` and `request` each have a 12 s per-app floor, so at most 5 a minute, which matches the PCES server's limit.
- Both calls refuse when the TX budget is nearly spent.
- The firmware never retries.
- One request is in flight at a time. It shares the touch UI's single-flight request slot, so it waits behind (and never clobbers) a status, telemetry or admin request.

### Firmware changes

- `src/MyMesh.h`:
  - `onAdvertRecv` override that keeps feature 1, which the core parses and then drops.
  - `advertFeat1()`: a 64-entry RAM cache, falling back to the stored raw advert blob for nodes heard before boot.
  - `uiSetContactPath()`.
- `src/ui-touch/UITask.cpp`:
  - `luaHost*` glue for the calls above.
  - The two new permission prompts and their Settings rows.
  - Lua hooks in front of the repeater admin app's `onBinaryReply` / `onAdminLoginResult`.
- `src/ui-touch/LuaAppHost.cpp`: the bindings, reply polling (replies arrive inside packet reception), caps, and teardown.
- `web/mesh/WebMesh.*`: stand-ins so the browser build keeps compiling. Routes are only mirrored there.

## The PCES app

1. **Server.** The app pre-selects a contact that advertises the PCES marker (`0xC502` on the station's repeater identity). *Pick* lists every repeater and room, because the marker is optional.
2. **Contact.** Tap a contact, then **Ask PCES for a route**. The app works out A and B as the spec says:
   - A is your first-hop repeater: the first hop of the current route, or the *Home repeater* you set when there is no route yet.
   - B is the contact itself when it is a relay, else the last relay of its current route. A chat contact you have never had a route to can't be planned. Message it once so MeshCore learns a path.
3. **Login, then PATH.** The first ask in a session sends a blank login and waits for LOGIN_OK. It then waits 2.5 s, so the request doesn't collide with the re-flooded login reply, and sends `C5 01 01 [width] A B`. Later asks skip the login.
4. **Answer.** The answer screen shows the metric (0 to 100), the weakest-leg SNR, the hops PCES returned, the route that would be installed, and the current route. Nothing changes until you tap **Use this route**. The original route is kept, and **Restore route** puts it back.
5. **Server info** sends INFO and shows the link count, data window, newest measurement age and SF.

Etiquette from the Commsol spec, enforced by the app: it sends only on a tap, never on a timer or a loop over contacts, and makes one attempt per tap. A request with no answer is reported, not retried.

## Testing

- Host: `lua5.4 test/lua/pces_app_test.lua`. It covers:
  - the codec against the spec's worked example, plus LINK, INFO and error statuses;
  - A/B planning;
  - the whole tap → login → request → answer → install → restore flow against a stubbed `wada.*`, including "nothing is transmitted on open" and "no answer is not retried".
- Device: build any `sdk_ext` board, for example `pio run -e LilyGo_TDeck_companion_radio_touch`. Copy `deploy/apps/pces/0.1/pces.lua` and `pces.json` to `/meshcomod/apps/` on the SD card, or `/apps/` on boards without one. PCES then appears in the app drawer. You need a Commsol station in range with Commsol running and able to transmit.

## Not done in this proof of concept

- No "PCES" badge in the native contact list. The marker is only visible to apps.
- LINK is encoded and decoded but has no screen yet.
- The browser build doesn't push `set_route` to its companion backend.
- Hardware-tested: not yet. The firmware compiles and the app passes its host test.
