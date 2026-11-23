-- PCES -- wadamesh Lua app (proof of concept)
-- Ask a Commsol station's Path Computation Element Service for the best route to
-- a contact, show the answer, and on request install it as that contact's route.
-- Protocol: Commsol docs/pces.md (request type 0xC5, version 1).
--
-- On-air etiquette (from the Commsol spec, and enforced here):
--   * every request is an explicit tap: nothing is sent on a timer or in a loop;
--   * one attempt per tap: a request that gets no answer is reported, not retried;
--   * at most one request every 12 s (the server answers 5 a minute per client).
--
-- Requires firmware with wada.mesh.login/request/route/set_route
-- (caps().node_requests) and advert feature-1 in contacts() (caps().feat1).

local ui, sys, store, timer, mesh = wada.ui, wada.sys, wada.store, wada.timer, wada.mesh
local C = ui.colors

local app = {}

-- ---- PCES codec (pure; exercised by test/pces_codec_test.lua) ---------------

local P = {}
app._pces = P

P.REQ = 0xC5
P.VER = 1
P.OP_INFO, P.OP_PATH, P.OP_LINK = 0x00, 0x01, 0x02
P.STATUS = {
  [0] = "ok", [1] = "no path", [2] = "A unknown", [3] = "B unknown",
  [4] = "A ambiguous", [5] = "B ambiguous", [6] = "bad request",
  [7] = "rate limited", [8] = "no link data yet", [9] = "unsupported",
}

function P.hex2bin(h)
  return (h:gsub("%x%x", function(x) return string.char(tonumber(x, 16)) end))
end
function P.bin2hex(b)
  return (b:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

-- A node is a hex pubkey prefix (1..8 bytes); "" means the answering station.
local function node(h) local b = P.hex2bin(h or ""); return string.char(#b) .. b end

function P.info_req() return string.char(P.REQ, P.VER, P.OP_INFO) end
function P.path_req(a, b, width)
  return string.char(P.REQ, P.VER, P.OP_PATH, width) .. node(a) .. node(b)
end
function P.link_req(a, b) return string.char(P.REQ, P.VER, P.OP_LINK) .. node(a) .. node(b) end

local function s8(v) return v >= 128 and v - 256 or v end
local function snr(v) v = s8(v); if v == -128 then return nil end; return v / 4 end
local function u16(d, i) return d:byte(i) + 256 * d:byte(i + 1) end

-- d = response data after the 4-byte tag (what on_reply hands us).
-- Returns a table with .status (0 = ok) and .err text, plus the op's fields.
function P.decode(d, width)
  if type(d) ~= "string" or #d < 3 then return { status = -1, err = "short reply" } end
  local ver, op, st = d:byte(1, 3)
  local r = { ver = ver, op = op, status = st }
  if ver ~= P.VER then r.status = -1; r.err = "version " .. ver; return r end
  if st ~= 0 then r.err = P.STATUS[st] or ("status " .. st); return r end
  if op == P.OP_PATH then
    if #d < 7 then r.status = -1; r.err = "short PATH"; return r end
    r.metric = d:byte(4)
    r.weakest_snr = snr(d:byte(5))
    local flags, n = d:byte(6), d:byte(7)
    r.starts_at_station = (flags & 1) ~= 0
    r.ends_at_station = (flags & 2) ~= 0
    if #d < 7 + n * width then r.status = -1; r.err = "short hop list"; return r end
    r.hops = {}
    for i = 0, n - 1 do r.hops[#r.hops + 1] = P.bin2hex(d:sub(8 + i * width, 7 + (i + 1) * width)) end
  elseif op == P.OP_LINK then
    if #d < 13 then r.status = -1; r.err = "short LINK"; return r end
    r.ab_metric, r.ba_metric = d:byte(4), d:byte(5)
    r.ab_snr, r.ba_snr = snr(d:byte(6)), snr(d:byte(7))
    r.ab_samples, r.ba_samples, r.age_min = u16(d, 8), u16(d, 10), u16(d, 12)
  elseif op == P.OP_INFO then
    if #d < 11 then r.status = -1; r.err = "short INFO"; return r end
    r.window_h, r.links, r.newest_min = u16(d, 4), u16(d, 6), u16(d, 8)
    r.decode_limit = snr(d:byte(10))
    r.sf = d:byte(11)
  end
  return r
end

-- Advert feature 1: 0xC5xx = a Commsol node; bit 1 = answers PCES.
function P.is_pces_server(feat1) return feat1 and (feat1 & 0xFF00) == 0xC500 and (feat1 & 0x0002) ~= 0 end

-- Which A/B to ask about, and how to turn the answer into a route. Given my
-- current route to the contact (list of hex hops, or nil), the contact's pubkey
-- prefix (hex) and whether the contact itself is a relay (repeater/room).
-- Returns plan {a, b, prefix = hops before A, drop_last = bool} or nil, why.
function P.plan(route, contact_pk, contact_is_relay, home)
  local prefix = {}
  local a
  if route and #route > 0 then a = route[1]
  elseif home and home ~= "" then a = home
  else return nil, "no first hop: set a home repeater" end
  local b
  if contact_is_relay then b = contact_pk
  elseif route and #route > 0 then b = route[#route]
  else return nil, "no known last relay for this contact" end
  return { a = a, b = b, prefix = prefix, drop_last = contact_is_relay }
end

-- The route to install from a PATH answer. The answer lists relays from A to B,
-- A and B included when they are relays; the contact itself is never on its own
-- route. Width must match the hops already in the plan.
function P.route_from(plan, ans)
  local out = {}
  for _, h in ipairs(plan.prefix) do out[#out + 1] = h end
  local hops = ans.hops or {}
  local n = #hops
  if plan.drop_last and n > 0 and plan.b:sub(1, #hops[n]) == hops[n] then n = n - 1 end
  for i = 1, n do out[#out + 1] = hops[i] end
  return out
end

function P.route_text(r)
  if not r then return "flood (no route)" end
  if #r == 0 then return "direct" end
  return table.concat(r, " > ")
end

-- ---- state -------------------------------------------------------------------

local W, H
local server = store.get("server", "")       -- 8-hex pubkey of the PCES node
local home = store.get("home", "")           -- my first-hop repeater (hex, path width)
local logged_in = {}                         -- server pk -> true once LOGIN_OK this session
local pending = nil                          -- {kind, contact, plan, width}
local answer = nil                           -- last decoded PATH answer + its plan
local status_lbl

local function caps_ok()
  local c = sys.caps and sys.caps() or {}
  return c.node_requests and c.feat1
end

local function contacts_all()
  local out, off = {}, 0
  while true do
    local page = mesh.contacts(off, 100)
    for _, c in ipairs(page) do out[#out + 1] = c end
    if #page < 100 then break end
    off = off + 100
  end
  return out
end

local function by_pk(pk)
  for _, c in ipairs(contacts_all()) do if c.pubkey == pk then return c end end
end

local function is_relay(c) return c.type == mesh.NODE_REPEATER or c.type == mesh.NODE_ROOM end

local function say(text, col)
  if status_lbl then status_lbl:set(text); status_lbl:color(col or C.sub) end
end

local function save_old_route(pk, r)
  if store.get("old_" .. pk, "") ~= "" then return end   -- keep the ORIGINAL route
  local v = "flood"
  if r then v = tostring(r.hash_size) .. ":" .. table.concat(r.hops, ",") end
  store.set("old_" .. pk, v)
end

local function load_old_route(pk)
  local v = store.get("old_" .. pk, "")
  if v == "" then return nil end
  if v == "flood" then return "flood" end
  local hs, list = v:match("^(%d):(.*)$")
  local hops = {}
  for h in (list or ""):gmatch("[^,]+") do hops[#hops + 1] = h end
  return { hash_size = tonumber(hs), hops = hops }
end

-- ---- screens -------------------------------------------------------------------

local show_main, show_contact, show_answer, pick_server, pick_home

local function header(title)
  ui.clear()
  ui.scroll(true)
  ui.label(title, 8, 4, 16, C.text, true)
  status_lbl = ui.label("", 8, H - 22, 12, C.sub)
  status_lbl:width(W - 16)
end

-- Send `req` to the server, logging in first when we have not this session.
local function ask(req, kind, ctx)
  if server == "" then say("Pick a PCES server first", C.bad); return end
  local function send()
    local how, err = mesh.request(server, req, 40000)   -- the daemon may hold it up to 60 s
    if not how then
      say("Not sent: " .. tostring(err), C.bad)
      pending = nil
      return
    end
    say("Asked PCES (" .. how .. "), waiting...", C.accent)
  end
  pending = { kind = kind, ctx = ctx, send = send }
  if logged_in[server] then send(); return end
  local ok, err = mesh.login(server, "")   -- blank login = PCES-only guest at the station
  if not ok then say("Login not sent: " .. tostring(err), C.bad); pending = nil; return end
  say("Logging in to server...", C.accent)
end

function app.on_login(r)
  if not pending or r.pubkey ~= server then return end
  if not r.ok then
    say("Login failed: " .. tostring(r.err), C.bad)
    pending = nil
    return
  end
  logged_in[server] = true
  -- Not at once: the login reply is still being re-flooded by neighbours, and a
  -- request sent into that mostly collides with it (firmware #683 uses 2 s).
  timer.after(2500, function() if pending then pending.send() end end)
end

function app.on_reply(r)
  if not pending or r.pubkey ~= server then return end
  local p = pending
  pending = nil
  if not r.ok then
    say("No answer from PCES (" .. tostring(r.err) .. "). Not retried.", C.bad)
    logged_in[server] = nil   -- a reboot or a full client list drops guests
    return
  end
  if p.kind == "info" then
    local d = P.decode(r.data, 0)
    if d.status ~= 0 then say("INFO: " .. tostring(d.err), C.bad); return end
    say(string.format("%d links, %d h window, newest %d min, SF%d",
      d.links, d.window_h, d.newest_min, d.sf), C.good)
  elseif p.kind == "path" then
    local d = P.decode(r.data, p.ctx.width)
    if d.status ~= 0 then say("PCES: " .. tostring(d.err), C.bad); return end
    answer = { d = d, plan = p.ctx.plan, contact = p.ctx.contact, width = p.ctx.width, rtt = r.rtt_ms }
    show_answer()
  end
end

show_answer = function()
  local a = answer
  local c = a.contact
  header("PCES route: " .. c.name)
  local y = 28
  local function line(t, col, size)
    local l = ui.label(t, 8, y, size or 14, col or C.text)
    l:width(W - 16)
    y = y + ui.text_lines(t, W - 16, size or 14) * ui.text_h(size or 14) + 4
  end
  local d = a.d
  line(string.format("Metric %d / 100", d.metric), d.metric >= 50 and C.good or (C.warn or C.accent), 24)
  line("Weakest leg SNR: " .. (d.weakest_snr and string.format("%+.2f dB", d.weakest_snr) or "n/a"))
  line("Hops from PCES: " .. P.route_text(d.hops), C.sub, 12)
  local route = P.route_from(a.plan, d)
  a.route = route
  line("Route to install: " .. P.route_text(route))
  local cur = mesh.route(c.pubkey)
  line("Current route: " .. P.route_text(cur and cur.hops or nil), C.sub, 12)
  line("Test the route before relying on it. If it stops working the mesh floods and relearns.", C.sub, 12)
  y = y + 4
  ui.button("Use this route", 8, y, W / 2 - 12, 34, function()
    save_old_route(c.pubkey, mesh.route(c.pubkey))
    local ok, err = mesh.set_route(c.pubkey, route)
    if ok then say("Route installed. Old one kept for Restore.", C.good)
    else say("Not installed: " .. tostring(err), C.bad) end
  end)
  ui.button("Back", W / 2 + 4, y, W / 2 - 12, 34, function() show_contact(c) end)
end

show_contact = function(c)
  header(c.name)
  local y = 28
  local cur = mesh.route(c.pubkey)
  local hops = cur and cur.hops or nil
  local l = ui.label("Route now: " .. P.route_text(hops), 8, y, 14, C.text); l:width(W - 16)
  y = y + 2 * ui.text_h(14) + 6
  local width = (cur and cur.hash_size) or mesh.path_hash_size()
  local plan, why = P.plan(hops, c.pubkey:sub(1, 2 * width), is_relay(c), home)
  if plan then
    local t = "Ask: best path " .. plan.a .. " to " .. plan.b
    local q = ui.label(t, 8, y, 12, C.sub); q:width(W - 16)
  else
    local q = ui.label("Cannot ask yet: " .. why, 8, y, 12, C.bad); q:width(W - 16)
  end
  y = y + 2 * ui.text_h(12) + 6
  ui.button("Ask PCES for a route", 8, y, W - 16, 34, function()
    if not plan then say(why, C.bad); return end
    if pending then say("A request is already waiting", C.bad); return end
    -- Width: bytes per hop in the answer, matching the hops we already hold.
    local w = #plan.a // 2
    ask(P.path_req(plan.a, plan.b, w), "path", { plan = plan, contact = c, width = w })
  end)
  y = y + 40
  local old = load_old_route(c.pubkey)
  if old then
    ui.button("Restore route", 8, y, W - 16, 34, function()
      local ok, err
      if old == "flood" then ok, err = mesh.set_route(c.pubkey, nil)
      else ok, err = mesh.set_route(c.pubkey, old.hops) end
      if ok then store.set("old_" .. c.pubkey, ""); say("Old route restored", C.good)
      else say("Not restored: " .. tostring(err), C.bad) end
    end)
    y = y + 40
  end
  ui.button("Back", 8, y, W - 16, 34, function() show_main() end)
end

pick_server = function()
  header("PCES server")
  local list = ui.list(8, 28, W - 16, H - 60)
  local any = false
  for _, c in ipairs(contacts_all()) do
    if is_relay(c) then
      -- Marked servers first in spirit; unmarked relays may still answer PCES.
      local tag = P.is_pces_server(c.feat1) and "[PCES] " or ""
      any = true
      list:add(tag .. c.name .. "  " .. c.pubkey, function()
        server = c.pubkey; store.set("server", server); show_main()
      end)
    end
  end
  if not any then say("No repeaters or rooms in contacts", C.bad) end
end

pick_home = function()
  header("Home repeater (first hop)")
  local w = mesh.path_hash_size()
  local list = ui.list(8, 28, W - 16, H - 60)
  for _, c in ipairs(contacts_all()) do
    if c.type == mesh.NODE_REPEATER then
      list:add(c.name .. "  " .. c.pubkey:sub(1, 2 * w), function()
        home = c.pubkey:sub(1, 2 * w); store.set("home", home); show_main()
      end)
    end
  end
end

show_main = function()
  header("PCES")
  local y = 28
  local sc = server ~= "" and by_pk(server) or nil
  local st = sc and (sc.name .. (P.is_pces_server(sc.feat1) and "  [PCES]" or "  (no marker)")) or "none"
  local l = ui.label("Server: " .. st, 8, y, 12, C.sub); l:width(W - 90)
  ui.button("Pick", W - 76, y - 4, 68, 26, pick_server)
  y = y + 28
  local h = ui.label("Home repeater: " .. (home ~= "" and home or "none"), 8, y, 12, C.sub); h:width(W - 90)
  ui.button("Pick", W - 76, y - 4, 68, 26, pick_home)
  y = y + 28
  ui.button("Server info", 8, y, W - 16, 28, function()
    if pending then say("A request is already waiting", C.bad); return end
    ask(P.info_req(), "info", nil)
  end)
  y = y + 34
  local list = ui.list(8, y, W - 16, H - y - 28)
  for _, c in ipairs(contacts_all()) do
    if c.pubkey ~= server then
      local r = mesh.route(c.pubkey)
      local hint = r and (#r.hops .. " hop") or "flood"
      list:add(c.name .. "  (" .. hint .. ")", function() show_contact(c) end)
    end
  end
end

-- ---- lifecycle -----------------------------------------------------------------

function app.on_open(w, h)
  W, H = w, h
  if not caps_ok() then
    ui.label("This firmware has no node requests (wada.mesh.request).", 8, 8, 14, C.bad):width(w - 16)
    return
  end
  -- Pre-select an advertised PCES server if none is chosen yet.
  if server == "" then
    for _, c in ipairs(contacts_all()) do
      if P.is_pces_server(c.feat1) then server = c.pubkey; store.set("server", server); break end
    end
  end
  show_main()
end

function app.on_close() end

return app
