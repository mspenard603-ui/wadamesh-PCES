-- Host test for the PCES app (deploy/apps/pces): the codec against the worked
-- example in Commsol's docs/pces.md, and the whole tap -> login -> request ->
-- answer -> install flow against a stubbed wada.* API.
--   lua5.4 test/lua/pces_app_test.lua      (from the repository root)
local APP = arg and arg[1] or "deploy/apps/pces/0.1/pces.lua"

local fails = 0
local function check(cond, what)
  if not cond then fails = fails + 1; print("FAIL " .. what) else print("ok   " .. what) end
end
local function hex(b) return (b:gsub(".", function(c) return string.format("%02X ", c:byte()) end)):sub(1, -2) end
local function bin(h) return (h:gsub("%s", ""):gsub("%x%x", function(x) return string.char(tonumber(x, 16)) end)) end

-- ---- stub firmware -----------------------------------------------------------
local log = { sent = {}, logins = {}, routes = {}, buttons = {}, lists = {}, status = "" }
local store_t = {}
local timers = {}
local contacts = {
  { name = "Commsol", pubkey = "c0ffee01", type = 2, feat1 = 0xC502 },
  { name = "Hilltop", pubkey = "3a7d1122", type = 2, feat1 = 0 },
  { name = "Valley",  pubkey = "b41e3344", type = 2, feat1 = 0 },
  { name = "Alice",   pubkey = "a11ce000", type = 1, feat1 = 0 },
}
local routes = { a11ce000 = { hash_size = 2, hops = { "3a7d", "9999" } } }

local function widget(kind, text)
  local w = { kind = kind, text = text }
  function w:set(t) self.text = t; if self == log.status_lbl then log.status = t end end
  function w:color() end
  function w:width() return self end
  function w:pos() end
  function w:add(t, fn) self.rows = self.rows or {}; self.rows[#self.rows + 1] = { t = t, fn = fn } end
  return w
end
wada = {
  ui = {
    colors = { text = 1, sub = 2, accent = 3, good = 4, bad = 5, warn = 6 },
    clear = function() log.buttons = {}; log.lists = {}; log.labels = {} end,
    scroll = function() end,
    label = function(t, x, y)
      local w = widget("label", t)
      log.labels = log.labels or {}; log.labels[#log.labels + 1] = w
      if y and y > 200 then log.status_lbl = w end   -- the status line sits at H - 22
      return w
    end,
    button = function(t, x, y, w, h, fn) log.buttons[t] = fn; return widget("button", t) end,
    list = function() local l = widget("list"); log.lists[#log.lists + 1] = l; return l end,
    text_lines = function() return 1 end,
    text_h = function() return 14 end,
  },
  sys = { caps = function() return { node_requests = true, feat1 = true } end, toast = function() end },
  store = { get = function(k, d) local v = store_t[k]; if v == nil then return d end; return v end,
            set = function(k, v) store_t[k] = v end },
  timer = { after = function(ms, fn) timers[#timers + 1] = fn end, every = function() end },
  mesh = {
    NODE_CHAT = 1, NODE_REPEATER = 2, NODE_ROOM = 3, NODE_SENSOR = 4,
    contacts = function(off) if (off or 0) > 0 then return {} end; return contacts end,
    route = function(pk) return routes[pk] or false end,
    path_hash_size = function() return 2 end,
    login = function(who, pw) log.logins[#log.logins + 1] = { who = who, pw = pw }; return true end,
    request = function(who, data) log.sent[#log.sent + 1] = { who = who, data = data }; return "direct" end,
    set_route = function(pk, hops) log.routes[#log.routes + 1] = { pk = pk, hops = hops }; routes[pk] = hops and { hash_size = 2, hops = hops } or nil; return true end,
  },
}

local app = dofile(APP)
local P = app._pces

-- ---- codec: the spec's worked example ------------------------------------------
check(hex(P.path_req("3a7d", "", 2)) == "C5 01 01 02 02 3A 7D 00", "PATH request encodes as in the spec")
local d = P.decode(bin("01 01 00 50 1D 02 02 3A 7D B4 1E"), 2)
check(d.status == 0 and d.metric == 80, "PATH ok, metric 80")
check(d.weakest_snr == 7.25, "weakest leg SNR +7.25 dB")
check(d.ends_at_station and not d.starts_at_station, "flags 2: ends at the station")
check(#d.hops == 2 and d.hops[1] == "3a7d" and d.hops[2] == "b41e", "hops 3a7d, b41e")
check(P.decode(bin("01 01 01"), 2).err == "no path", "status 1 = no path")
check(P.decode(bin("01 01 00 50 80 00 00"), 2).weakest_snr == nil, "SNR -128 = no value")
local l = P.decode(bin("01 02 00 50 FF 1D 80 0A 00 00 00 05 00"), 2)
check(l.ab_metric == 80 and l.ba_metric == 255 and l.ab_snr == 7.25 and l.ba_snr == nil and l.ab_samples == 10 and l.age_min == 5,
      "LINK decodes, unmeasured direction = 255 / no SNR / 0 samples")
local i = P.decode(bin("01 00 00 A8 00 2A 00 03 00 E2 07"), 0)
check(i.window_h == 168 and i.links == 42 and i.newest_min == 3 and i.decode_limit == -7.5 and i.sf == 7, "INFO decodes")
check(hex(P.info_req()) == "C5 01 00", "INFO request")
check(hex(P.link_req("3a7d", "b41e")) == "C5 01 02 02 3A 7D 02 B4 1E", "LINK request")
check(P.is_pces_server(0xC502) and not P.is_pces_server(0xC501) and not P.is_pces_server(0x0002), "feature-1 marker")

-- ---- planning ---------------------------------------------------------------------
local plan = P.plan({ "3a7d", "9999" }, "a11c", false, "")
check(plan.a == "3a7d" and plan.b == "9999" and not plan.drop_last, "chat contact: A = first hop, B = last relay")
check(table.concat(P.route_from(plan, { hops = { "3a7d", "b41e", "9999" } }), ",") == "3a7d,b41e,9999", "route keeps both ends")
local rp = P.plan(nil, "b41e", true, "3a7d")
check(rp.a == "3a7d" and rp.b == "b41e" and rp.drop_last, "repeater contact with no route: A = home, B = contact")
check(table.concat(P.route_from(rp, { hops = { "3a7d", "b41e" } }), ",") == "3a7d", "the contact is not on its own route")
check(select(2, P.plan(nil, "a11c", false, "3a7d")) ~= nil, "chat contact with no route cannot be planned")

-- ---- flow ------------------------------------------------------------------------------
app.on_open(320, 240)
check(store_t.server == "c0ffee01", "advertised PCES server is pre-selected")
check(#log.sent == 0 and #log.logins == 0, "nothing transmitted on open")
local alice
for _, r in ipairs(log.lists[1].rows) do if r.t:find("^Alice") then alice = r end end
alice.fn()
log.buttons["Ask PCES for a route"]()
check(#log.logins == 1 and log.logins[1].who == "c0ffee01" and log.logins[1].pw == "", "tap logs in with a blank password")
check(#log.sent == 0, "request waits for LOGIN_OK")
app.on_login({ pubkey = "c0ffee01", ok = true, perms = 0 })
check(#log.sent == 0 and #timers == 1, "request deferred after LOGIN_OK")
timers[1]()
check(#log.sent == 1 and hex(log.sent[1].data) == "C5 01 01 02 02 3A 7D 02 99 99", "one PATH request 3a7d -> 9999, width 2")
app.on_reply({ pubkey = "c0ffee01", ok = true, rtt_ms = 900, data = bin("01 01 00 50 1D 00 03 3A 7D B4 1E 99 99") })
check(log.buttons["Use this route"] ~= nil, "answer screen offers the route")
check(#log.routes == 0, "nothing installed until the user taps")
log.buttons["Use this route"]()
check(#log.routes == 1 and table.concat(log.routes[1].hops, ",") == "3a7d,b41e,9999", "installs the PCES route")
check(store_t["old_a11ce000"] == "2:3a7d,9999", "old route kept for Restore")
-- A second ask reuses the session login.
log.buttons["Back"]()
log.buttons["Ask PCES for a route"]()
check(#log.logins == 1 and #log.sent == 2, "already logged in: request goes straight out")
app.on_reply({ pubkey = "c0ffee01", ok = false, err = "no reply" })
check(#log.sent == 2, "no answer is not retried")
log.buttons["Restore route"]()
check(table.concat(log.routes[#log.routes].hops, ",") == "3a7d,9999" and store_t["old_a11ce000"] == "", "Restore puts the old route back")

print(fails == 0 and "ALL PASS" or (fails .. " FAILED"))
os.exit(fails == 0 and 0 or 1)
