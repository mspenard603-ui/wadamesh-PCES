// SPDX-License-Identifier: GPL-3.0-or-later
// PCES route screen ("PCES route" on a contact's action sheet).
//
// Asks a Commsol station's Path Computation Element Service for the best path to a
// contact, shows the answer (metric, weakest-leg SNR, hops) and, only when the user
// taps "Use this route", installs it as the contact's out path. The replaced route is
// kept so "Restore route" can put it back. Codec and route planning:
// src/helpers/PcesProto.h (host test: test/test_pces_proto.cpp). Protocol of record:
// Commsol docs/pces.md. Notes: PCES.md.
//
// Native firmware on purpose: Lua apps cannot transmit requests to nodes, and PCES
// needs a login and a binary REQ. It reuses what the repeater admin app already sends:
// uiSendAdminLogin (blank password = PCES-only guest at the station) and
// uiSendBinaryRequest, with the replies routed here from UITask::onAdminLoginResult and
// UITask::onBinaryReply.
//
// Automatic: when a PCES server's advert reaches us through relays, the client asks
// that server for the best path to itself, with the relay that handed us the advert as
// A (pcesServiceTick). Once per server per boot, then not again for 6 hours, single
// attempt, only while nothing else holds the request slot. The answer is installed as
// the server's route when the server has none yet; otherwise it is kept for the PCES
// screen on that contact, where "Use this route" applies it.
//
// On-air etiquette, from the Commsol spec:
//   * nothing is sent except on a tap or on a PCES server's advert -- never on a timer
//     and never in a loop over contacts;
//   * one attempt per tap: no answer is reported, never retried;
//   * at most one request every 12 s (the server answers 5 a minute per client).
//
// NOT a normal header: UITask.cpp includes it once, right after RepeaterAdminApp.h, and
// it shares that translation unit's static helpers (settings kit, app page, popups).
#pragma once
#include <stdarg.h>
#include "helpers/PcesProto.h"

namespace {
namespace PP = PcesProto;

enum PcesPhase : uint8_t { PCES_IDLE = 0, PCES_LOGIN, PCES_GAP, PCES_REQUEST };

struct PcesState {
  bool     open = false;
  uint8_t  contact[PUB_KEY_SIZE] = {0};
  char     name[40] = "";
  uint8_t  server[PUB_KEY_SIZE] = {0};
  bool     have_server = false;
  bool     server_marked = false;          // advertises the 0xC5xx PCES marker
  // Guest session at the server: a node forgets guests on reboot, so reuse it 15 min.
  uint8_t  session_server[PUB_KEY_SIZE] = {0};
  uint32_t session_ms = 0;
  bool     session_valid = false;
  uint8_t  phase = PCES_IDLE;
  uint8_t  req_server[PUB_KEY_SIZE] = {0}; // who the login / request in flight went to
  bool     auto_job = false;               // the in-flight ask came from an advert
  // Advert-triggered asks: one queued job, and which servers were asked when.
  bool     auto_pending = false;
  uint8_t  auto_pub[PUB_KEY_SIZE] = {0};
  uint8_t  auto_hop[3] = {0};
  uint8_t  auto_width = 0;
  uint32_t auto_seen_ms = 0;
  struct { uint8_t pub4[4]; uint32_t ms; bool used; } asked[8] = {};
  uint8_t  asked_next = 0;
  uint8_t  answer_contact[PUB_KEY_SIZE] = {0};   // whose route the answer is for
  uint8_t  op = PP::OP_PATH;               // what the pending request asks
  uint32_t deadline = 0, gap_at = 0, last_req_ms = 0;
  bool     have_last_req = false;
  uint8_t  req[24] = {0};
  uint8_t  req_len = 0;
  PP::Plan plan;
  bool     have_answer = false;
  PP::PathAnswer ans;
  uint8_t  route[PP::MAX_HOPS * PP::MAX_WIDTH] = {0};
  int      route_hops = -1;                // -1 = the answer does not make a route
  bool     have_info = false;
  PP::InfoAnswer info;
  char     msg[120] = "";
  uint8_t  msg_tone = SET_TONE_HELP;
  bool     dirty = false;                  // a reply changed state: redraw on the timer
  // The route "Use this route" replaced, for "Restore route" (one contact at a time).
  bool     has_old = false;
  uint8_t  old_pub[PUB_KEY_SIZE] = {0};
  uint8_t  old_len = 0;                    // raw out_path_len (OUT_PATH_UNKNOWN = none)
  uint8_t  old_path[MAX_PATH_SIZE] = {0};
};
PcesState* s_pces = nullptr;
lv_obj_t*  s_pces_root = nullptr;
lv_obj_t*  s_pces_page = nullptr;
lv_timer_t* s_pces_timer = nullptr;   // the service tick: runs whether or not the screen is open
char       s_pces_title[48] = "";

static constexpr uint32_t kPcesReqGapMs     = 12000;   // 5 a minute, the server's limit
static constexpr uint32_t kPcesLoginWaitMs  = 20000;
static constexpr uint32_t kPcesReplyWaitMs  = 40000;   // under the UI slot's 45 s expiry
static constexpr uint32_t kPcesAfterLoginMs = 2500;    // let the re-flooded login reply pass (#683)
static constexpr uint32_t kPcesSessionMs    = 15u * 60u * 1000u;
static constexpr uint32_t kPcesAutoEveryMs  = 6u * 60u * 60u * 1000u;   // re-ask a server after 6 h
static constexpr uint32_t kPcesAutoHoldMs   = 60000;   // a queued advert job waits this long for the slot

void pcesRender();   // fwd
void pcesBack();     // fwd

void pcesSay(uint8_t tone, const char* fmt, ...) {
  va_list ap; va_start(ap, fmt);
  vsnprintf(s_pces->msg, sizeof s_pces->msg, fmt, ap);
  va_end(ap);
  s_pces->msg_tone = tone;
}

ContactInfo* pcesContact() { return the_mesh.lookupContactByPubKey(s_pces->contact, PUB_KEY_SIZE); }
ContactInfo* pcesServer() {
  return s_pces->have_server ? the_mesh.lookupContactByPubKey(s_pces->server, PUB_KEY_SIZE) : nullptr;
}
bool pcesIsRelay(const ContactInfo& c) { return c.type == ADV_TYPE_REPEATER || c.type == ADV_TYPE_ROOM; }

// The first contact advertising "answers PCES", unless the user picked one.
void pcesFindServer() {
  if (s_pces->have_server && pcesServer()) {
    s_pces->server_marked = PP::answersPces(the_mesh.advertFeat1(s_pces->server));
    return;
  }
  s_pces->have_server = false;
  const uint32_t n = the_mesh.getNumContacts();
  for (uint32_t i = 0; i < n; ++i) {
    ContactInfo c;
    if (!the_mesh.getContactByIdx(i, c) || !pcesIsRelay(c)) continue;
    if (PP::answersPces(the_mesh.advertFeat1(c.id.pub_key))) {
      memcpy(s_pces->server, c.id.pub_key, PUB_KEY_SIZE);
      s_pces->have_server = s_pces->server_marked = true;
      return;
    }
  }
}

// "3A7D > B41E", or the route's state when it has no relays.
void pcesFmtRoute(const uint8_t* path, int hops, int width, char* out, size_t cap) {
  if (hops < 0) { snprintf(out, cap, "%s", TR("None (floods)")); return; }
  if (hops == 0) { snprintf(out, cap, "%s", TR("Direct")); return; }
  size_t o = 0;
  out[0] = 0;
  for (int i = 0; i < hops && o + 12 < cap; ++i) {
    if (i) o += snprintf(out + o, cap - o, " > ");
    for (int k = 0; k < width; ++k) o += snprintf(out + o, cap - o, "%02X", path[i * width + k]);
  }
}
void pcesFmtContactRoute(const ContactInfo& c, char* out, size_t cap) {
  if (c.out_path_len == OUT_PATH_UNKNOWN || !mesh::Packet::isValidPathLen(c.out_path_len)) {
    pcesFmtRoute(nullptr, -1, 1, out, cap);
    return;
  }
  pcesFmtRoute(c.out_path, c.out_path_len & 63, (c.out_path_len >> 6) + 1, out, cap);
}

// ---- sending -------------------------------------------------------------------------
bool pcesCanSendNow() {
  if (s_pces->phase != PCES_IDLE) { pcesSay(SET_TONE_WARN, "%s", TR("A request is already waiting.")); return false; }
  if (s_pces->have_last_req) {
    const uint32_t since = millis() - s_pces->last_req_ms;
    if (since < kPcesReqGapMs) {
      pcesSay(SET_TONE_WARN, TR("Wait %u s: PCES answers 5 requests a minute."),
              (unsigned)((kPcesReqGapMs - since + 999) / 1000));
      return false;
    }
  }
  if (the_mesh.hasUIRequestPending()) { pcesSay(SET_TONE_WARN, "%s", TR("Another request is in flight. Try again shortly.")); return false; }
  // Duty cycle is the radio dispatcher's job, as for every other UI request; the 12 s
  // floor above is the PCES-specific limit.
  return true;
}

void pcesSendRequestNow() {
  ContactInfo* srv = the_mesh.lookupContactByPubKey(s_pces->req_server, PUB_KEY_SIZE);
  if (!srv) { s_pces->phase = PCES_IDLE; pcesSay(SET_TONE_DANGER, "%s", TR("PCES server is no longer a contact.")); return; }
  const int r = the_mesh.uiSendBinaryRequest(*srv, s_pces->req, s_pces->req_len);
  if (r != MSG_SEND_SENT_FLOOD && r != MSG_SEND_SENT_DIRECT) {
    s_pces->phase = PCES_IDLE;
    pcesSay(SET_TONE_DANGER, "%s", TR("Not sent: radio busy."));
    return;
  }
  s_pces->phase = PCES_REQUEST;
  s_pces->deadline = millis() + kPcesReplyWaitMs;
  s_pces->last_req_ms = millis(); s_pces->have_last_req = true;
  pcesSay(SET_TONE_INFO, "%s", r == MSG_SEND_SENT_FLOOD ? TR("Asked PCES (flood). Waiting for the answer...")
                                                        : TR("Asked PCES. Waiting for the answer..."));
}

// Send s_pces->req to `srv`, logging in as a guest first unless a session is fresh.
void pcesAsk(ContactInfo* srv) {
  if (!srv) { pcesSay(SET_TONE_DANGER, "%s", TR("No PCES server. Choose one below.")); return; }
  memcpy(s_pces->req_server, srv->id.pub_key, PUB_KEY_SIZE);
  const bool session = s_pces->session_valid && memcmp(s_pces->session_server, srv->id.pub_key, PUB_KEY_SIZE) == 0 &&
                       (uint32_t)(millis() - s_pces->session_ms) < kPcesSessionMs;
  if (session) { pcesSendRequestNow(); return; }
  const int r = the_mesh.uiSendAdminLogin(*srv, "");   // blank = PCES-only guest at the station
  if (r != MSG_SEND_SENT_FLOOD && r != MSG_SEND_SENT_DIRECT) {
    pcesSay(SET_TONE_DANGER, "%s", TR("Login not sent: radio busy."));
    return;
  }
  s_pces->phase = PCES_LOGIN;
  s_pces->deadline = millis() + kPcesLoginWaitMs;
  s_pces->last_req_ms = millis(); s_pces->have_last_req = true;
  pcesSay(SET_TONE_INFO, "%s", TR("Logging in to the PCES server..."));
}

void pcesAskPathCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  ContactInfo* c = pcesContact();
  if (!c) { pcesSay(SET_TONE_DANGER, "%s", TR("Contact gone.")); pcesRender(); return; }
  const bool have_path = c->out_path_len != OUT_PATH_UNKNOWN && mesh::Packet::isValidPathLen(c->out_path_len);
  const uint8_t hops = have_path ? (uint8_t)(c->out_path_len & 63) : 0;
  const uint8_t width = have_path ? (uint8_t)((c->out_path_len >> 6) + 1) : 0;
  const PP::PlanErr pe = PP::makePlan(c->out_path, hops, width, c->id.pub_key, pcesIsRelay(*c), s_pces->plan);
  if (pe != PP::PLAN_OK) {
    if (pe == PP::PLAN_NO_LAST_RELAY)
      pcesSay(SET_TONE_WARN, "%s", TR("The route is a single relay: there is nothing between it and the contact to compute."));
    else
      pcesSay(SET_TONE_WARN, "%s", TR("No route to this contact yet. Message it once so the mesh learns one, then ask."));
    pcesRender();
    return;
  }
  if (!pcesCanSendNow()) { pcesRender(); return; }
  const PP::Plan& p = s_pces->plan;
  s_pces->req_len = (uint8_t)PP::encodePath(s_pces->req, p.a, p.alen, p.b, p.blen, p.width);
  s_pces->op = PP::OP_PATH;
  s_pces->auto_job = false;
  s_pces->have_answer = false;
  memcpy(s_pces->answer_contact, c->id.pub_key, PUB_KEY_SIZE);
  pcesAsk(pcesServer());
  pcesRender();
}

void pcesInfoCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  if (!pcesCanSendNow()) { pcesRender(); return; }
  s_pces->req_len = (uint8_t)PP::encodeInfo(s_pces->req);
  s_pces->op = PP::OP_INFO;
  s_pces->auto_job = false;
  s_pces->have_info = false;
  pcesAsk(pcesServer());
  pcesRender();
}

// ---- installing ------------------------------------------------------------------------
void pcesUseCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  ContactInfo* c = pcesContact();
  if (!c || !s_pces->have_answer || s_pces->route_hops < 0 ||
      memcmp(s_pces->answer_contact, c->id.pub_key, PUB_KEY_SIZE) != 0) return;
  // Keep the ORIGINAL route: a second install over our own must not overwrite it.
  if (!(s_pces->has_old && memcmp(s_pces->old_pub, c->id.pub_key, PUB_KEY_SIZE) == 0)) {
    memcpy(s_pces->old_pub, c->id.pub_key, PUB_KEY_SIZE);
    s_pces->old_len = c->out_path_len;
    memcpy(s_pces->old_path, c->out_path, sizeof s_pces->old_path);
    s_pces->has_old = true;
  }
  if (the_mesh.uiSetContactPath(c->id.pub_key, s_pces->route, (uint8_t)s_pces->route_hops, s_pces->plan.width))
    pcesSay(SET_TONE_OK, "%s", TR("Route installed. Test it before relying on it; Restore puts the old one back."));
  else
    pcesSay(SET_TONE_DANGER, "%s", TR("Route not installed: it does not fit."));
  pcesRender();
}

void pcesRestoreCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces || !s_pces->has_old) return;
  const uint8_t len = s_pces->old_len;
  bool ok;
  if (len == OUT_PATH_UNKNOWN || !mesh::Packet::isValidPathLen(len))
    ok = the_mesh.uiResetContactPath(s_pces->old_pub);
  else
    ok = the_mesh.uiSetContactPath(s_pces->old_pub, s_pces->old_path, (uint8_t)(len & 63), (uint8_t)((len >> 6) + 1));
  if (ok) { s_pces->has_old = false; pcesSay(SET_TONE_OK, "%s", TR("Previous route restored.")); }
  else pcesSay(SET_TONE_DANGER, "%s", TR("Could not restore: contact gone."));
  pcesRender();
}

// ---- server choice -----------------------------------------------------------------------
void pcesPickServerCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  const uint32_t idx = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
  ContactInfo c;
  if (!the_mesh.getContactByIdx(idx, c)) return;
  memcpy(s_pces->server, c.id.pub_key, PUB_KEY_SIZE);
  s_pces->have_server = true;
  s_pces->server_marked = PP::answersPces(the_mesh.advertFeat1(c.id.pub_key));
  s_pces->msg[0] = 0;
  pcesRender();
}

// ---- automatic: best path to a PCES server whose advert we heard ---------------------------
bool pcesAskedRecently(const uint8_t* pub) {
  for (auto& a : s_pces->asked)
    if (a.used && memcmp(a.pub4, pub, 4) == 0) return (uint32_t)(millis() - a.ms) < kPcesAutoEveryMs;
  return false;
}
void pcesMarkAsked(const uint8_t* pub) {
  for (auto& a : s_pces->asked)
    if (a.used && memcmp(a.pub4, pub, 4) == 0) { a.ms = millis(); return; }
  auto& a = s_pces->asked[s_pces->asked_next];
  s_pces->asked_next = (uint8_t)((s_pces->asked_next + 1) % 8);
  memcpy(a.pub4, pub, 4); a.ms = millis(); a.used = true;
}

// The station's answer to "best path from <the relay next to us> to you".
void pcesAutoAnswer(const uint8_t* data, size_t len) {
  ContactInfo* srv = the_mesh.lookupContactByPubKey(s_pces->req_server, PUB_KEY_SIZE);
  char nm[40];
  snprintf(nm, sizeof nm, "%s", srv ? contactShownName(srv->id.pub_key, srv->name) : "PCES");
  if (!PP::decodePath(data, len, s_pces->plan.width, s_pces->ans))
    return;   // no path known yet, unknown relay...: nothing to offer, nothing to say
  s_pces->have_answer = true;
  memcpy(s_pces->answer_contact, s_pces->req_server, PUB_KEY_SIZE);
  s_pces->route_hops = PP::routeFromAnswer(s_pces->plan, s_pces->ans, s_pces->route);
  if (s_pces->route_hops < 0 || !srv || !g_lv.task) return;
  char msg[96];
  if (srv->out_path_len == OUT_PATH_UNKNOWN &&
      the_mesh.uiSetContactPath(srv->id.pub_key, s_pces->route, (uint8_t)s_pces->route_hops, s_pces->plan.width)) {
    snprintf(msg, sizeof msg, TR("PCES: route to %s installed (metric %u)"), nm, (unsigned)s_pces->ans.metric);
  } else {
    snprintf(msg, sizeof msg, TR("PCES: best path to %s, metric %u. See PCES route."), nm, (unsigned)s_pces->ans.metric);
  }
  g_lv.task->showAlert(msg, 2600);
}

// Pick up a PCES server advert from MyMesh and, when the slot is free, ask it.
void pcesAutoTick() {
  uint8_t pub[PUB_KEY_SIZE], hop[3], width = 0, hops = 0;
  if (the_mesh.takePcesAdvert(pub, hop, &width, &hops)) {
    if (!pcesAskedRecently(pub) && the_mesh.lookupContactByPubKey(pub, PUB_KEY_SIZE)) {
      memcpy(s_pces->auto_pub, pub, PUB_KEY_SIZE);
      memcpy(s_pces->auto_hop, hop, 3);
      s_pces->auto_width = width;
      s_pces->auto_seen_ms = millis();
      s_pces->auto_pending = true;
    }
  }
  if (!s_pces->auto_pending) return;
  if ((uint32_t)(millis() - s_pces->auto_seen_ms) > kPcesAutoHoldMs) { s_pces->auto_pending = false; return; }
  // Never under the user: not while the screen is open or anything is in flight.
  if (s_pces->open || s_pces->phase != PCES_IDLE || the_mesh.hasUIRequestPending()) return;
  if (s_pces->have_last_req && (uint32_t)(millis() - s_pces->last_req_ms) < kPcesReqGapMs) return;
  ContactInfo* srv = the_mesh.lookupContactByPubKey(s_pces->auto_pub, PUB_KEY_SIZE);
  s_pces->auto_pending = false;
  if (!srv) return;
  pcesMarkAsked(s_pces->auto_pub);   // asked, answered or not: once per server per 6 h
  PP::Plan& p = s_pces->plan;
  p = PP::Plan();
  p.width = s_pces->auto_width;
  memcpy(p.a, s_pces->auto_hop, p.width); p.alen = p.width;
  p.blen = 0;                        // B = the answering station itself
  s_pces->req_len = (uint8_t)PP::encodePath(s_pces->req, p.a, p.alen, nullptr, 0, p.width);
  s_pces->op = PP::OP_PATH;
  s_pces->auto_job = true;
  s_pces->have_answer = false;
  pcesAsk(srv);
}

// ---- replies (from UITask's mesh callbacks) ---------------------------------------------
bool pcesOnLoginResult(const ContactInfo& contact, bool success, uint8_t) {
  if (!s_pces || s_pces->phase != PCES_LOGIN ||
      memcmp(contact.id.pub_key, s_pces->req_server, PUB_KEY_SIZE) != 0) return false;
  if (!success) {
    s_pces->phase = PCES_IDLE;
    pcesSay(SET_TONE_DANGER, "%s", TR("Login refused by the PCES server."));
  } else {
    memcpy(s_pces->session_server, s_pces->req_server, PUB_KEY_SIZE);
    s_pces->session_ms = millis(); s_pces->session_valid = true;
    s_pces->phase = PCES_GAP;
    s_pces->gap_at = millis() + kPcesAfterLoginMs;
    pcesSay(SET_TONE_INFO, "%s", TR("Logged in. Asking PCES..."));
  }
  s_pces->dirty = true;
  return true;
}

bool pcesOnBinaryReply(const ContactInfo& contact, const uint8_t* data, size_t len) {
  if (!s_pces || s_pces->phase != PCES_REQUEST ||
      memcmp(contact.id.pub_key, s_pces->req_server, PUB_KEY_SIZE) != 0) return false;
  s_pces->phase = PCES_IDLE;
  if (s_pces->auto_job) { pcesAutoAnswer(data, len); s_pces->dirty = true; return true; }
  if (s_pces->op == PP::OP_INFO) {
    if (PP::decodeInfo(data, len, s_pces->info)) { s_pces->have_info = true; s_pces->msg[0] = 0; }
    else pcesSay(SET_TONE_DANGER, "PCES: %s", TR(PP::statusText(s_pces->info.status)));
  } else {
    if (PP::decodePath(data, len, s_pces->plan.width, s_pces->ans)) {
      s_pces->have_answer = true;
      s_pces->route_hops = PP::routeFromAnswer(s_pces->plan, s_pces->ans, s_pces->route);
      if (s_pces->route_hops < 0) pcesSay(SET_TONE_WARN, "%s", TR("The answer does not start at your first hop, so it cannot be used as a route."));
      else s_pces->msg[0] = 0;
    } else {
      pcesSay(SET_TONE_WARN, "PCES: %s", TR(PP::statusText(s_pces->ans.status)));
    }
  }
  s_pces->dirty = true;
  return true;
}

void pcesTimerCb(lv_timer_t*) {
  if (!s_pces) return;
  const uint32_t now = millis();
  if (s_pces->phase == PCES_GAP && (int32_t)(now - s_pces->gap_at) >= 0) {
    pcesSendRequestNow();
    s_pces->dirty = true;
  } else if ((s_pces->phase == PCES_LOGIN || s_pces->phase == PCES_REQUEST) && (int32_t)(now - s_pces->deadline) >= 0) {
    if (s_pces->phase == PCES_REQUEST) the_mesh.cancelUIPingPending();   // free the slot we hold
    s_pces->session_valid = false;   // a station restart, or a full client list, drops guests
    s_pces->phase = PCES_IDLE;
    if (!s_pces->auto_job)
      pcesSay(SET_TONE_WARN, "%s", TR("No answer from PCES. Not retried: check the station is in range and running."));
    s_pces->dirty = true;
  }
  if (s_pces->phase == PCES_IDLE) pcesAutoTick();
  if (s_pces->dirty) { s_pces->dirty = false; if (s_pces->open) pcesRender(); }
}

// ---- page ----------------------------------------------------------------------------------
lv_obj_t* pcesText(lv_obj_t* parent, const char* text, const lv_font_t* f, uint32_t color) {
  lv_obj_t* l = lv_label_create(parent);
  lv_obj_set_width(l, s_settings_content_w);
  lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
  lv_label_set_text(l, text ? text : "");
  lv_obj_set_style_text_font(l, f, LV_PART_MAIN);
  lv_obj_set_style_text_color(l, lv_color_hex(color), LV_PART_MAIN);
  return l;
}
uint32_t pcesToneColor(uint8_t tone) {
  switch (tone) {
    case SET_TONE_OK: return COLOR_STATUS_OK;
    case SET_TONE_WARN: return COLOR_STATUS_WARN;
    case SET_TONE_DANGER: return COLOR_STATUS_DANGER;
    case SET_TONE_INFO: return COLOR_STATUS_INFO;
    default: return COLOR_SUB;
  }
}
void pcesFmtSnr(int8_t snr4, char* out, size_t cap) {
  if (snr4 == PP::SNR_NONE) snprintf(out, cap, "-");
  else snprintf(out, cap, "%+.2f dB", snr4 / 4.0);
}

void pcesRender() {
  if (!s_pces || !s_pces_root) return;
  if (s_pces_page) {
    lv_indev_t* a = lv_indev_get_act();
    if (a) lv_indev_wait_release(a);
    lv_obj_del_async(s_pces_page);   // async: this may run from a button on the old page
    s_pces_page = nullptr;
  }
  s_pces_page = lv_obj_create(s_pces_root);
  lv_obj_remove_style_all(s_pces_page);
  lv_obj_set_size(s_pces_page, lv_disp_get_hor_res(nullptr), appPageContentH());
  const lv_coord_t bar_cover = (lv_coord_t)(STATUSBAR_H * 2) - appPageContentTop();
  lv_obj_set_style_pad_top(s_pces_page, LV_MAX(0, bar_cover) + 8, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(s_pces_page, 12, LV_PART_MAIN);
  lv_obj_set_style_pad_left(s_pces_page, 8, LV_PART_MAIN);
  lv_obj_set_flex_flow(s_pces_page, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scroll_dir(s_pces_page, LV_DIR_VER);

  ContactInfo* c = pcesContact();
  char buf[200];
  const bool busy = s_pces->phase != PCES_IDLE;

  // Contact and its route now.
  lv_obj_t* card = setCardIn(s_pces_page, TR("Contact"));
  if (c) {
    pcesFmtContactRoute(*c, buf, sizeof buf);
    setInfoRow(card, TR("Route now"), buf);
  } else {
    setNote(card, "Contact gone.", SET_TONE_DANGER);
  }
  if (c) setButton(card, UI_ICON_RADAR, "Ask PCES for a route", pcesAskPathCb, nullptr, busy ? SET_NORMAL : SET_PRIMARY);
  if (busy && s_pces->auto_job) pcesText(card, TR("Asking a PCES server for the path to it (its advert just arrived)..."), &g_font_12, COLOR_SUB);
  if (c && s_pces->has_old && memcmp(s_pces->old_pub, c->id.pub_key, PUB_KEY_SIZE) == 0)
    setButton(card, UI_ICON_HISTORY, "Restore route", pcesRestoreCb);
  if (s_pces->msg[0]) pcesText(card, s_pces->msg, &g_font_12, pcesToneColor(s_pces->msg_tone));

  // The answer, and the one action that changes anything.
  if (s_pces->have_answer && c && memcmp(s_pces->answer_contact, c->id.pub_key, PUB_KEY_SIZE) == 0) {
    lv_obj_t* ac = setCardIn(s_pces_page, TR("PCES answer"));
    const PP::PathAnswer& a = s_pces->ans;
    snprintf(buf, sizeof buf, "%u / 100", (unsigned)a.metric);
    lv_obj_t* m = pcesText(ac, buf, &g_font_semi_16, a.metric >= 50 ? COLOR_STATUS_OK : COLOR_STATUS_WARN);
    (void)m;
    pcesFmtSnr(a.weakest_snr4, buf, sizeof buf);
    setInfoRow(ac, TR("Weakest leg"), buf);
    pcesFmtRoute(a.hop, a.hops, a.width, buf, sizeof buf);
    setInfoRow(ac, TR("PCES path"), buf);
    if (s_pces->route_hops >= 0) {
      pcesFmtRoute(s_pces->route, s_pces->route_hops, s_pces->plan.width, buf, sizeof buf);
      setInfoRow(ac, TR("New route"), buf);
      setButton(ac, UI_ICON_CHECK, "Use this route", pcesUseCb, nullptr, SET_PRIMARY);
      setNote(ac, "Metric 0-100: the weakest leg's SNR margin over the decode limit, weighted by how much "
                  "recent data backs it. If the route stops working the mesh floods and relearns one.");
    }
  }

  // Server.
  lv_obj_t* sc = setCardIn(s_pces_page, TR("PCES server"));
  ContactInfo* srv = pcesServer();
  if (srv) {
    snprintf(buf, sizeof buf, "%s%s", contactShownName(srv->id.pub_key, srv->name),
             s_pces->server_marked ? "" : TR(" (no PCES marker)"));
    setInfoRow(sc, TR("Server"), buf);
    setButton(sc, UI_ICON_INFO, "Server info", pcesInfoCb);
    if (s_pces->have_info) {
      const PP::InfoAnswer& i = s_pces->info;
      snprintf(buf, sizeof buf, TR("%u links over %u h, newest %u min old, SF%u"),
               (unsigned)i.links, (unsigned)i.window_h, (unsigned)i.newest_min, (unsigned)i.sf);
      pcesText(sc, buf, &g_font_12, COLOR_SUB);
    }
  } else {
    setNote(sc, "No contact advertises PCES. Pick the Commsol station (or a room it hosts):");
  }
  // Other candidates: every repeater and room (the marker is optional), capped.
  int shown = 0;
  const uint32_t n = the_mesh.getNumContacts();
  for (uint32_t i = 0; i < n && shown < 12; ++i) {
    ContactInfo k;
    if (!the_mesh.getContactByIdx(i, k) || !pcesIsRelay(k)) continue;
    if (s_pces->have_server && memcmp(k.id.pub_key, s_pces->server, PUB_KEY_SIZE) == 0) continue;
    if (c && memcmp(k.id.pub_key, c->id.pub_key, PUB_KEY_SIZE) == 0) continue;
    if (srv && !PP::answersPces(the_mesh.advertFeat1(k.id.pub_key))) continue;   // a server is set: offer only marked ones
    snprintf(buf, sizeof buf, "%s%s", contactShownName(k.id.pub_key, k.name),
             PP::answersPces(the_mesh.advertFeat1(k.id.pub_key)) ? "  [PCES]" : "");
    setButton(sc, UI_ICON_ANTENNA, buf, pcesPickServerCb, (void*)(uintptr_t)i);
    ++shown;
  }

  if (g_statusbar.root) lv_obj_move_foreground(g_statusbar.root);
}

// Closing only takes the page down: a request in flight finishes in the background
// (its answer is kept for the next open), and the service tick keeps running.
void pcesClose() {
  if (!s_pces_root) return;
  if (s_pces) s_pces->open = false;
  s_pces_page = nullptr;
  popupClose(&s_pces_root);
  appPageEnd(pcesBack);
}
void pcesBack() { pcesClose(); }

}  // namespace

// From UITask::loop: start the service tick once the UI is up (it is what notices PCES
// server adverts). Cheap after the first call.
static void pcesServiceEnsure() {
  if (s_pces_timer || !g_lv.task) return;
  if (!s_pces) s_pces = (PcesState*)psAlloc(sizeof(PcesState));
  if (!s_pces) return;
  s_pces_timer = lv_timer_create(pcesTimerCb, 250, nullptr);
}

// Open the PCES screen for a contact.
static void openPces(const ContactInfo& c) {
  if (!s_pces) s_pces = (PcesState*)psAlloc(sizeof(PcesState));
  if (!s_pces) { if (g_lv.task) g_lv.task->showAlert(TR("Out of memory"), 1400); return; }
  pcesClose();
  if (memcmp(s_pces->contact, c.id.pub_key, PUB_KEY_SIZE) != 0) s_pces->msg[0] = 0;   // another contact
  memcpy(s_pces->contact, c.id.pub_key, PUB_KEY_SIZE);
  pcesServiceEnsure();
  copyUtf8ReplacingMissingGlyphs(&g_font_16, s_pces->name, sizeof s_pces->name, contactShownName(c.id.pub_key, c.name));
  s_pces->open = true;
  s_pces->dirty = false;
  pcesFindServer();
  snprintf(s_pces_title, sizeof s_pces_title, "PCES: %s", s_pces->name);
  s_pces_root = appPageCreateRoot(COLOR_BG);
  appPageBegin(s_pces_title, pcesBack);
  pcesRender();
}
