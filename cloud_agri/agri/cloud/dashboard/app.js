/* AGRI-CLOUD dashboard.
 *
 * One state object, polled; everything else is a pure render of it. No
 * framework: the whole UI is four render functions and a fetch on a timer,
 * and a framework would add a build step to a page that has none.
 *
 * The colour rule, restated because it is the only thing here that is a
 * design decision rather than plumbing: a station's fill is its value on a
 * SEQUENTIAL ramp for the selected quantity, scaled between that quantity's
 * plausible bounds -- not between the observed min and max. Auto-scaling to
 * the data would make a greenhouse where everything is fine look exactly
 * like one where everything is wrong.
 */

const S = { state: null, quantity: "temperature", selected: null,
            row: "", side: "", token: "", parked: null };

/* AUTH. The token normally arrives as an HttpOnly cookie: the operator
   opens the ?token= URL the Cloud printed exactly once, the server sets the
   cookie and redirects to a clean address, and from then on the browser
   sends the credential by itself. That is why S.token is usually empty and
   nothing here breaks -- the token has simply stopped being this page's
   business, and stopped living in the history and in every proxy log
   between here and the operator.

   The ?token= path below survives for the cases the cookie cannot cover: a
   browser with cookies disabled, and the 401 prompt further down. */
const CRED = { credentials: "same-origin" };

(function () {
  const p = new URLSearchParams(window.location.search);
  if (p.has("token")) S.token = p.get("token");
  /* Only needed when there is no cookie. With one, /table authorises
     itself and the plain href in the HTML is right. */
  const nav = document.getElementById("nav-table");
  if (nav && S.token) nav.href = "/table?token=" + encodeURIComponent(S.token);
})();

/* TEXT INSIDE THE FLIPPED MAP.
 *
 * The map is drawn in world metres and then flipped with scale(1,-1), so
 * that +y is up and the picture matches the building. Everything inherits
 * that flip -- including glyphs, which came out upside down and backwards.
 * The row labels shipped like that.
 *
 * The fix is per-text and not per-group: translate the element's own
 * baseline to the origin, flip back, and translate out again, which is what
 * translate(0, 2y) scale(1,-1) does in one attribute. Un-flipping the whole
 * text layer instead would mean maintaining a second set of coordinates,
 * and the two would drift the first time a mark moved. */
const upright = y => `transform="translate(0,${2 * y}) scale(1,-1)"`;

/* Sequential ramps, dark -> bright, one per quantity. Each stays within a
 * single hue family so the eye reads magnitude rather than category. */
const RAMPS = {
  temperature: ["#0b2a3a", "#1d6b7a", "#3fb8a6", "#a8f0c8", "#fff2b0"],
  humidity:    ["#2a1c0b", "#6b4a1d", "#3f7ab8", "#7fc6f0", "#d8f4ff"],
  luminosity:  ["#191a06", "#4d4a10", "#9c8f1c", "#e0cf46", "#fff8c4"],
  co2:         ["#0d2016", "#1c5236", "#4a8f4a", "#c9c256", "#ff9d4d"],
  ph:          ["#3a0f2a", "#7a2a5e", "#b06aa0", "#7fb0d8", "#bfe8ff"],
};

function lerpHex(a, b, t) {
  const p = h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16));
  const [ar, ag, ab] = p(a), [br, bg, bb] = p(b);
  const c = (x, y) => Math.round(x + (y - x) * t).toString(16).padStart(2, "0");
  return `#${c(ar, br)}${c(ag, bg)}${c(ab, bb)}`;
}

function ramp(name, t) {
  const stops = RAMPS[name] || RAMPS.temperature;
  t = Math.max(0, Math.min(1, t));
  const seg = t * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(seg));
  return lerpHex(stops[i], stops[i + 1], seg - i);
}

const q = () => (S.state?.quantities || []).find(x => x.name === S.quantity);

function norm(value) {
  const Q = q();
  if (!Q || value == null) return null;
  return (value - Q.lo) / (Q.hi - Q.lo);
}

/* ------------------------------------------------------------- fetch */
function apiUrl(path) {
  return S.token ? `${path}${path.includes("?") ? "&" : "?"}token=${S.token}` : path;
}

async function poll() {
  try {
    const r = await fetch(apiUrl("/api/state"), CRED);
    if (r.status === 401) {
      S.token = prompt("Dashboard token:") || "";
      return poll();
    }
    S.state = await r.json();
    render();
    /* The parked station and its offset, in the footer, in millimetres.
       The highlight on the map says WHICH mark; this says HOW WELL, which
       is the number the evaluation is actually about -- and it is the same
       number the report carries as parking_error_m. */
    const p = S.parked
      ? ` // parked on ${S.parked.label}, ${(S.parked.d * 1000).toFixed(0)} mm off`
      : "";
    const act = S.state.active_station
      ? ` // collecting ${S.state.active_station}` : "";
    document.getElementById("foot").textContent =
      `link up // ${S.state.summary.visits} visits filed${p}${act} // `
      + `cloud since ${S.state.since}`;
  } catch (e) {
    document.getElementById("foot").textContent = `cloud unreachable: ${e}`;
  }
}

/* ------------------------------------------------------------ render */
function render() {
  if (!S.state) return;
  renderStrip();
  renderPicker();
  renderJump();
  renderMap();
  renderRequests();
  renderRejects();
  renderLink();
  renderMode();
  if (S.selected) renderDetail(S.selected);
}

function renderLink() {
  const nodes = S.state.nodes || {};
  const ids = Object.keys(nodes);
  if (!ids.length) {
    document.getElementById("linkdot").className = "dot";
    document.getElementById("linktext").textContent = "no node";
    return;
  }
  const anyOn = ids.some(k => nodes[k].online);
  document.getElementById("linkdot").className = "dot" + (anyOn ? " on" : "");
  document.getElementById("linktext").textContent = ids.map(k => {
    const r = nodes[k];
    const kind = r.node_kind || "?";
    const where = r.pose ? ` x ${r.pose.x} y ${r.pose.y}` : "";
    const fast = r.velocity ? ` ${r.velocity.speed.toFixed(2)} m/s` : "";
    return `${r.robot || k} (${kind}) ${r.online ? "online" : "OFFLINE"}${where}${fast}`;
  }).join("  |  ");
}

function tile(k, v, sub, warn) {
  return `<div class="tile${warn ? " warn" : ""}"><div class="k">${k}</div>
          <div class="v">${v}${sub ? ` <small>${sub}</small>` : ""}</div></div>`;
}

function renderStrip() {
  const s = S.state.summary;
  const nodes = S.state.nodes || {};
  const first = Object.values(nodes)[0] || {};
  const v = first.velocity;
  const [done, total] = s.coverage;
  document.getElementById("strip").innerHTML = [
    tile("coverage", `${done}<small>/${total}</small>`, "stations"),
    tile("visits", s.visits, "filed"),
    tile("flagged", s.flagged, "out of range", s.flagged > 0),
    tile("accepted", s.accepted, "reports"),
    tile("rejected", s.rejected, "refused", s.rejected > 0),
    tile("parking", s.mean_parking_m == null ? "—" : s.mean_parking_m,
         s.worst_parking_m == null ? "m" : `m mean / ${s.worst_parking_m} worst`),
    tile("speed", v == null ? "—" : v.speed, "m/s now"),
  ].join("");
}

/* Short enough to fit beside a cross, honest enough to act on. Luminosity
   runs to 80 000 and CO2 to 5 000; printed in full they overlap the next
   station and the map becomes a wall of digits. */
function fmt(v) {
  if (v == null) return "";
  const a = Math.abs(v);
  if (a >= 10000) return (v / 1000).toFixed(1) + "k";
  if (a >= 1000) return Math.round(v).toString();
  if (a >= 100) return v.toFixed(0);
  return v.toFixed(1);
}

function renderPicker() {
  const p = document.getElementById("picker");
  if (p.dataset.built) return;
  p.innerHTML = S.state.quantities.map(x =>
    `<button data-q="${x.name}" aria-pressed="${x.name === S.quantity}">${x.name}</button>`
  ).join("");
  p.dataset.built = "1";
  p.addEventListener("click", e => {
    const b = e.target.closest("button");
    if (!b) return;
    S.quantity = b.dataset.q;
    [...p.children].forEach(c => c.setAttribute("aria-pressed", c === b));
    renderMap();
  });
}

/* Which stations the filters let through. The selected one always is:
   hiding the station whose detail panel is open would leave the panel
   describing a mark that is not on the map. */
const visible = s => (!S.row || String(s.row) === S.row)
                  && (!S.side || s.side === S.side)
                  || s.label === S.selected;

/* Every station by name, for when clicking the right one of a touching pair
   is more trouble than reading a list. Rebuilt only when the set changes,
   so it does not reset itself under the cursor on every 2 s poll. */
function renderJump() {
  const el = document.getElementById("f-jump");
  const labels = (S.state.stations || []).map(s => s.label);
  const key = labels.join(",");
  if (el.dataset.key !== key) {
    el.dataset.key = key;
    el.innerHTML = `<option value="">station…</option>` +
      labels.map(l => `<option value="${l}">${l}</option>`).join("");
  }
  el.value = S.selected || "";
}

/* The greenhouse, to scale. 9.90 x 4.90 m interior. */
function renderMap() {
  const st = S.state.stations, Q = q();
  const rows = [-1.2, 0, 1.2];
  let svg = `<rect class="wall" x="-4.95" y="-2.45" width="9.9" height="4.9"/>`;
  rows.forEach(y => {
    svg += `<rect class="gutter" x="-4.6" y="${y - 0.2}" width="9.2" height="0.4"/>`;
  });
  st.forEach(s => {
    if (s.side === "R") {
      svg += `<circle class="plant" cx="${s.plant_x}" cy="${s.plant_y}" r="0.15"/>`;
    }
  });
  rows.forEach((y, i) => {
    /* -0.08, not +0.07: a baseline sits BELOW the glyphs, and "below" on
       screen is the smaller world y once the map is flipped. */
    const yy = y - 0.08;
    svg += `<text class="rowlab" x="-4.85" y="${yy}" ${upright(yy)}>R${i + 1}</text>`;
  });

  st.filter(visible).forEach(s => {
    const isPred = s.measured && (s.predicted || []).includes(S.quantity);
    const v = s.measured ? s.values[S.quantity] : null;
    const t = norm(v);
    const colour = s.measured ? ramp(S.quantity, t) : "none";
    const stroke = s.measured ? colour : "var(--ink-dim)";
    /* Half the arm of the drawn cross, in metres. Bounded above by the
       0.10 m between the two stations of an inner aisle: at 0.055 the two
       crosses run into each other and read as one X, which is the same
       mistake the PAINTED crosses were making at 0.09 (catalogue.py). */
    const a = 0.045;
    const sel = S.selected === s.label ? " sel" : "";
    /* WHICH SIDE OF THE CROSS THE CHIP GOES ON.
       Away from the plant: R is south of its row, so its chip goes further
       south; L is north, so its chip goes north. This is not decoration.
       P1,jL and P2,jR share an inner aisle 0.10 m apart, and a chip is
       0.22 m tall -- put both on the same side and one covers the other
       exactly, which is the overlap that made the map unreadable. Pushed
       apart this way they end up 0.50 m clear, and the chip becomes the
       station's real click target, far easier to hit than the mark. */
    const away = (s.side === "R" ? -1 : +1) * 0.30;
    const cy = s.y + away;
    /* The value, in figures, next to the mark.
       A ramp answers "more or less than its neighbours"; only a number
       answers "how much", and the operator is being asked to act on the
       value, not to admire the gradient. The chip is filled with the ramp
       colour so the two readings agree by construction -- and the text is
       black or white depending on that fill, which is the only way a
       number on a coloured chip stays readable at both ends of a scale. */
    const label = v == null ? "" : fmt(v);
    const ink = v == null ? "" : (norm(v) > 0.55 ? "#0b1416" : "#f2fbfa");
    const chip = v == null ? "" : `
      <rect class="chip${isPred ? " predicted" : ""}" x="${s.x - 0.30}" y="${cy - 0.11}" width="0.60"
            height="0.22" rx="0.05" fill="${colour}"/>
      <line class="stem" x1="${s.x}" y1="${s.y}" x2="${s.x}" y2="${cy}"
            stroke="${colour}"/>
      <text class="val" x="${s.x}" y="${cy - 0.06}" fill="${ink}"
            ${upright(cy - 0.06)}>${label}${isPred ? " ⚠" : ""}</text>`;
    svg += `<g class="station${sel}" data-label="${s.label}">
      <title>${s.label}${
        v == null ? " — not measured" :
        ` — ${S.quantity} ${v} ${Q ? Q.unit : ""}${isPred ? " (predicted by the robot)" : ""}`}</title>
      ${s.flags && s.flags.length ? `<circle class="halo" cx="${s.x}" cy="${s.y}" r="0.13"/>` : ""}
      ${S.selected === s.label ? `<circle class="halo" cx="${s.x}" cy="${s.y}" r="0.16"/>` : ""}
      <line class="cross" x1="${s.x - a}" y1="${s.y}" x2="${s.x + a}" y2="${s.y}" stroke="${stroke}"/>
      <line class="cross" x1="${s.x}" y1="${s.y - a}" x2="${s.x}" y2="${s.y + a}" stroke="${stroke}"/>
      ${chip}
      <circle cx="${s.x}" cy="${s.y}" r="0.05" fill="transparent"/>
    </g>`;
  });

  /* WHERE THE ROBOT IS, AND WHICH MARK IT IS STANDING ON.
   *
   * The map showed 48 crosses and no robot, so "did it park on the mark"
   * was a question only Gazebo could answer -- and the number the report
   * carries, which is the one being evaluated, appeared nowhere near the
   * picture it describes.
   *
   * The pose in a status message is the SENSOR point (the boom tip). The
   * chassis is SENSOR_OFFSET_X behind it, and that is what lands on the
   * paint, so the base is drawn where the base is. Getting this backwards
   * would draw the robot half a metre past every mark it is parked on. */
  const BOOM = 0.50;
  svg += robotLayer(st);
  svg += activeStationLayer(st);
  svg += predictedLayer(st);

  const map = document.getElementById("map");
  /* SVG y grows downward; the greenhouse's +y is north. Flip so the map
     reads the way the operator stands in the building. */
  map.innerHTML = `<g transform="scale(1,-1)">${svg}</g>`;
  map.onclick = e => {
    const g = e.target.closest(".station");
    if (g) select(g.dataset.label);
  };

  const Ql = Q || { lo: 0, hi: 1, unit: "" };
  document.getElementById("scale").innerHTML =
    `<span>${Ql.lo} ${Ql.unit}</span>
     <span class="ramp" style="background:linear-gradient(90deg,${
       [0, .25, .5, .75, 1].map(t => ramp(S.quantity, t)).join(",")})"></span>
     <span>${Ql.hi} ${Ql.unit}</span>`;
}

/* ------------------------------------------------------- the two modes */
/* The console has had `mode` and `pause`/`resume` all along. A console verb
   is invisible to somebody watching a screen, and the two modes are what
   the demonstration is about -- so they are buttons, and the handshake they
   produce is printed beside them. Switching mid-session is the point: a
   jury that has to watch the Cloud be restarted to see the second mode has
   been shown two programs, not one system with two modes. */
async function setMode(body) {
  const hint = document.getElementById("modehint");
  hint.className = "hint";
  hint.textContent = "asking the Cloud…";
  try {
    const r = await fetch(apiUrl("/api/mode"), {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body), ...CRED,
    });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || r.statusText);
    if (S.state) { S.state.mode = d.mode; S.state.receiving = d.receiving; }
    renderMode();
    poll();
  } catch (e) {
    hint.className = "hint bad";
    hint.textContent = String(e);
  }
}

const MODE_WHY = {
  command: "the operator asks and the Cloud relays. Each reading comes "
         + "straight back, because the Cloud asked for it and is waiting.",
  collector: "the Cloud runs the campaign: it polls the node until it is "
           + "genuinely stopped, issues the order itself, and negotiates "
           + "every handover. Nobody types anything after the first order.",
};

function renderMode() {
  if (!S.state) return;
  const mode = S.state.mode || "command";
  document.querySelectorAll("#seg-mode button").forEach(b => {
    b.classList.toggle("on", b.dataset.mode === mode);
  });
  const rx = S.state.receiving !== false;
  const btn = document.getElementById("btn-receiving");
  btn.textContent = rx ? "EN RÉCEPTION" : "EN PAUSE — le robot garde";
  btn.classList.toggle("paused", !rx);
  /* The pause only MEANS anything in collector mode: in command mode the
     robot was asked for the reading and sends it. Saying so beats a button
     that silently does nothing. */
  btn.disabled = mode !== "collector";
  const hint = document.getElementById("modehint");
  hint.className = "hint";
  hint.textContent = MODE_WHY[mode]
    + (mode === "collector"
        ? (rx ? "" : " Paused: every offer is answered HOLD and the readings "
                   + "pile up in the robot's outbox until you resume.")
        : " (the pause button belongs to collector mode)");

  const ol = document.getElementById("dialogue");
  const d = S.state.dialogue || [];
  ol.innerHTML = d.map(e =>
    `<li class="say ${e.who === "cloud" ? "cloud" : "node"}">
       <span class="who">${e.who === "cloud" ? "CLOUD ──►" : "◄── NODE"}</span>
       <span class="what">${e.text}</span></li>`).join("")
    || `<li class="say quiet">nothing yet — the handshake appears here as it happens</li>`;
  ol.scrollTop = ol.scrollHeight;
}

/* The live robot, and the station it is standing on. Returns SVG. */
function robotLayer(stations) {
  const BOOM = 0.50, ARM = 0.045;
  const node = Object.values(S.state.nodes || {}).find(n => n.online && n.pose);
  if (!node) { S.parked = null; return ""; }

  /* The status pose is the boom tip; the chassis is one boom behind it,
     along the robot's own heading. */
  const yaw = (node.pose.yaw_deg || 0) * Math.PI / 180;
  const bx = node.pose.x - BOOM * Math.cos(yaw);
  const by = node.pose.y - BOOM * Math.sin(yaw);

  /* Which mark is it on? Nearest, and only if it is genuinely on it --
     PARK_TOLERANCE is 0.04 m, and calling a robot "parked" at 0.30 m
     would make the highlight say the opposite of the truth. */
  let best = null, bestD = Infinity;
  stations.forEach(s => {
    const d = Math.hypot(s.x - bx, s.y - by);
    if (d < bestD) { bestD = d; best = s; }
  });
  const parked = best && bestD <= 0.12;
  S.parked = parked ? { label: best.label, d: bestD } : null;

  let g = "";
  if (parked) {
    /* The mark it is on, in its own colour, over the top of the one drawn
       in the station layer. Plus the offset in millimetres: the highlight
       says WHICH, the number says HOW WELL, and the evaluation is about
       the second. */
    g += `<g class="parked">
      <circle class="parkhalo" cx="${best.x}" cy="${best.y}" r="0.20"/>
      <line x1="${best.x - ARM}" y1="${best.y}" x2="${best.x + ARM}" y2="${best.y}"/>
      <line x1="${best.x}" y1="${best.y - ARM}" x2="${best.x}" y2="${best.y + ARM}"/>
    </g>`;
  }
  /* The chassis footprint, to scale (0.58 x 0.38 m), so that "the mark is
     under the robot" is something the picture shows rather than claims. */
  g += `<g class="robot${parked ? " on" : ""}"
           transform="rotate(${node.pose.yaw_deg || 0} ${bx} ${by})">
    <rect x="${bx - 0.29}" y="${by - 0.19}" width="0.58" height="0.38" rx="0.04"/>
    <line class="boom" x1="${bx}" y1="${by}" x2="${bx + BOOM}" y2="${by}"/>
    <circle class="lens" cx="${bx + BOOM}" cy="${by}" r="0.045"/>
  </g>`;
  return g;
}

/* The station being collected RIGHT NOW -- pulsing ring + dashed line from
   the robot so the map shows the intent, not only the result. */
function activeStationLayer(stations) {
  const label = S.state.active_station;
  if (!label) return "";
  const s = stations.find(st => st.label === label);
  if (!s) return "";
  const node = Object.values(S.state.nodes || {}).find(n => n.online && n.pose);
  let g = `<g class="collecting">
    <circle class="collect-ring" cx="${s.x}" cy="${s.y}" r="0.22"/>`;
  if (node) {
    const yaw = (node.pose.yaw_deg || 0) * Math.PI / 180;
    const bx = node.pose.x - 0.50 * Math.cos(yaw);
    const by = node.pose.y - 0.50 * Math.sin(yaw);
    g += `<line class="collect-line" x1="${bx}" y1="${by}" x2="${s.x}" y2="${s.y}"/>`;
  }
  g += `</g>`;
  return g;
}

/* A station whose LATEST report contains at least one value the robot
   predicted rather than measured -- real, from the wire, never a demo
   toggle (see renderDetail). Marked on the map so a reading recovered
   from a sensor gap is visible without opening the panel. */
function predictedLayer(stations) {
  const flagged = stations.filter(s => s.predicted && s.predicted.length);
  if (!flagged.length) return "";
  let g = "";
  flagged.forEach(s => {
    const ty = s.y + 0.30;
    g += `<g class="predicted-station">
      <circle class="fault-ring" cx="${s.x}" cy="${s.y}" r="0.18"/>
      <text class="fault-mark" x="${s.x}" y="${ty}" ${upright(ty)}>⚠ PRÉDIT</text>
    </g>`;
  });
  return g;
}

/* ISO 8601 UTC -> the operator's own wall clock, to the second.
   The stamps travel as UTC because the greenhouse and the Cloud need not be
   in the same time zone (see measurement.utc_now). Showing that raw would
   make everyone reading the page do the arithmetic, so it is done once here
   and the full stamp stays in the title attribute for anyone who wants it. */
function clock(iso) {
  if (!iso) return "—";
  const d = new Date(iso);
  return isNaN(d) ? iso : d.toLocaleTimeString();
}

/* Seconds -> "42 s" / "3 min 05". Past an hour nobody counts in minutes. */
function dur(s) {
  if (s == null) return "";
  if (s < 90) return `${Math.round(s)} s`;
  const m = Math.floor(s / 60), r = Math.round(s % 60);
  if (m < 60) return `${m} min ${String(r).padStart(2, "0")}`;
  return `${Math.floor(m / 60)} h ${String(m % 60).padStart(2, "0")}`;
}

function renderRequests() {
  const el = document.getElementById("requests");
  const rs = S.state.requests || [];
  const now = Date.now();
  el.innerHTML = rs.length ? rs.slice().reverse().map(r => {
    const pct = r.total ? Math.round(100 * r.done / r.total) : 0;
    /* Elapsed comes from the Cloud once the request is complete, and is
       counted here while it is still running -- the browser cannot be the
       authority on a duration that has to survive the page being reloaded,
       but it is the only thing ticking between two 2 s polls. */
    const live = r.completed_at == null && r.issued_at
      ? (now - new Date(r.issued_at)) / 1000 : null;
    /* A request that was ABANDONED is closed too -- it has an end time and a
       duration like any other -- but calling that "done" would report a
       failure as a success in the one place an operator looks for the
       answer. Same numbers, different verb. */
    const timing = r.completed_at
      ? `${r.state === "failed" ? "gave up" : "done"} ${clock(r.completed_at)}`
        + ` // after ${dur(r.elapsed_s)}`
      : `running // ${dur(live)} so far`;
    /* Distance arrives only on the closing ack, and only from a robot that
       counts it, so this line appears when there is something to say and is
       absent otherwise -- rather than showing a reassuring 0.0 m for a robot
       that never reported one. The energy is flagged "est" here, and only
       here, because it is the one number on this page that is modelled
       rather than measured. */
    const trip = r.metres == null ? "" :
      `<div class="when">drove ${r.metres.toFixed(1)} m`
      + (r.planned_m != null ? ` // planned ${r.planned_m.toFixed(1)} m` : "")
      + (r.energy_wh != null ? ` // ~${r.energy_wh.toFixed(1)} Wh est` : "")
      + `</div>`;
    return `<div class="req">
      <div><span class="id">${r.request_id}</span> ${r.state}
        ${r.label ? `// ${r.label}` : ""} <span class="id">${r.done}/${r.total}</span></div>
      <div class="when" title="issued ${r.issued_at || "?"}">
        issued ${clock(r.issued_at)} // ${timing}</div>
      ${trip}
      ${r.detail ? `<div class="id">${r.detail}</div>` : ""}
      <div class="prog"><i style="width:${pct}%"></i></div>
    </div>`;
  }).join("") : `<div class="empty">no request issued yet</div>`;
}

function renderRejects() {
  const el = document.getElementById("rejects");
  const rs = S.state.rejected || [];
  el.innerHTML = rs.length ? rs.slice().reverse().map(r =>
    `<div class="rej">${r.at} // ${r.robot}<br>${r.reason}</div>`).join("")
    : `<div class="empty">nothing refused — every report verified</div>`;
}

/* ------------------------------------------------------------ detail */
async function select(label) {
  S.selected = label;
  document.getElementById("detail").hidden = false;
  renderJump();
  renderMap();
  renderDetail(label);
  const r = await fetch(apiUrl("/api/history/" + encodeURIComponent(label)), CRED);
  renderHistory(await r.json());
  document.getElementById("detail").scrollIntoView({ behavior: "smooth",
                                                     block: "nearest" });
}

function renderDetail(label) {
  const s = (S.state.stations || []).find(x => x.label === label);
  if (!s) return;
  document.getElementById("detail-label").textContent =
    `${label}  //  row ${s.row} plant ${s.plant} ${s.side === "R" ? "right" : "left"}`;

  /* THE SIGN. Every value below is exactly what the robot sent -- no
     toggle, no Cloud-side override. `predicted` names the quantities
     the robot's OWN local model filled in because its sensor could not;
     everything else on the row is a real reading. See
     agri.measurement.Measurement.predicted and agri.prediction.
     LocalPredictor -- the recovery happens on the robot, before the
     report ever reaches here. */
  const predicted = new Set(s.predicted || []);
  const rows = (S.state.quantities || []).map(Q => {
    const v = s.values ? s.values[Q.name] : null;
    const isPred = predicted.has(Q.name);
    const t = v == null ? 0 : Math.max(0, Math.min(1, (v - Q.lo) / (Q.hi - Q.lo)));
    const flagged = (s.flags || []).some(f => f.endsWith(":" + Q.name));
    const tag = isPred ? ` <small class="tag">prédit par le robot</small>` : "";
    return `<div class="reading${flagged ? " flagged" : ""}${isPred ? " predicted" : ""}">
      <span class="n">${Q.name}</span>
      <span class="bar-t"><i style="width:${(t * 100).toFixed(1)}%;
        background:${ramp(Q.name, t)}"></i></span>
      <span class="val">${v == null ? "—" : v} <small>${Q.unit}</small>${tag}</span>
    </div>`;
  }).join("");
  const park = s.parking_error_m == null ? "—" : `${s.parking_error_m} m`;
  const chain = s.measured
    ? `ordered ${clock(s.request_issued_at)} // measured ${clock(s.timestamp)}`
      + ` // filed ${clock(s.received_at)}`
      + (s.latency_s == null ? "" : ` // ${dur(s.latency_s)} end to end`)
    : "never measured";
  document.getElementById("detail-readings").innerHTML = rows +
    `<div class="hint" title="${s.timestamp || ""}">${chain}<br>` +
    `parked ${park} from the cross</div>`;

  const photo = document.getElementById("detail-photo");
  const qr = document.getElementById("detail-qr");
  photo.src = s.photo ? apiUrl("/media/" + s.photo) : "";
  photo.style.visibility = s.photo ? "visible" : "hidden";
  qr.src = s.qr ? apiUrl("/media/" + s.qr) : "";
  qr.style.visibility = s.qr ? "visible" : "hidden";
}

function renderHistory(rows) {
  const el = document.getElementById("detail-history");
  if (!rows.length) { el.innerHTML = `<div class="empty">no history</div>`; return; }
  const qs = S.state.quantities.map(x => x.name);
  el.innerHTML = `<table><thead><tr><th>ordered</th><th>measured</th><th>filed</th>${
    qs.map(n => `<th>${n}</th>`).join("")}<th>park m</th></tr></thead><tbody>${
    rows.slice(-12).reverse().map(r => `<tr>
      <td title="${r.request_issued_at || ""}">${clock(r.request_issued_at)}</td>
      <td title="${r.timestamp}">${clock(r.timestamp)}</td>
      <td title="${r.received_at || ""}">${clock(r.received_at)}</td>${
      qs.map(n => `<td>${r.values[n] ?? "—"}</td>`).join("")
    }<td>${r.parking_error_m ?? "—"}</td></tr>`).join("")}</tbody></table>`;
}

/* ----------------------------------------------------------- command */
async function request(targets) {
  const hint = document.getElementById("cmdhint");
  hint.className = "hint";
  hint.textContent = "signing and publishing…";
  try {
    const r = await fetch(apiUrl("/api/request"), {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ targets }), ...CRED,
    });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || r.statusText);
    hint.textContent = `request ${d.request_id} issued // ${d.targets.length} station(s)`;
    poll();
  } catch (e) {
    hint.className = "hint bad";
    hint.textContent = String(e.message || e);
  }
}

/* ----------------------------------------------------------- session */
function grab(url, name) {
  const a = document.createElement("a");
  a.href = apiUrl(url);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
}

async function save() {
  const hint = document.getElementById("sessionhint");
  hint.className = "hint";
  hint.textContent = "exporting…";
  try {
    const r = await fetch(apiUrl("/api/export"), { method: "POST", ...CRED });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || r.statusText);
    hint.textContent = `${d.visits} visit(s), ${d.measured}/${d.total} stations`
      + ` and ${d.requests} request(s) → ${d.path}`;
    // Both files are already on the Cloud's disk; this only puts copies in
    // the browser's downloads, which is what someone looking at the
    // dashboard from another machine actually wants. Sequentially, with a
    // pause: two synthetic clicks in the same tick and Firefox keeps one.
    await grab(d.url, "measurements.csv");
    await new Promise(r => setTimeout(r, 400));
    await grab(d.requests_url, "requests.csv");
  } catch (e) {
    hint.className = "hint bad";
    hint.textContent = String(e.message || e);
  }
}

async function quit() {
  if (!confirm("Stop everything?\n\nBoth CSVs are exported first, then the "
               + "Cloud stops and takes the simulation with it — Gazebo, "
               + "RViz, the bridge and the robot node. This page will go "
               + "dead.")) return;
  const hint = document.getElementById("sessionhint");
  hint.className = "hint";
  hint.textContent = "stopping…";
  try {
    const r = await fetch(apiUrl("/api/quit"), { method: "POST", ...CRED });
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || r.statusText);
  } catch (e) {
    // A Cloud that dies before the reply is flushed looks exactly like a
    // network error, and reporting one for a shutdown that worked would be
    // worse than saying nothing: the next poll settles it either way.
    hint.textContent = "stop requested";
    return;
  }
  hint.textContent = "stopped — restart with ./run_sim.sh and ./run_cloud.sh";
}

/* ------------------------------------------------------------ filters */
document.getElementById("f-row").onchange = e => {
  S.row = e.target.value; renderMap();
};
document.getElementById("f-side").onchange = e => {
  S.side = e.target.value; renderMap();
};
document.getElementById("f-jump").onchange = e => {
  if (e.target.value) select(e.target.value);
};

document.getElementById("btn-all").onclick = () => request("ALL");
document.getElementById("btn-one").onclick = () =>
  request(document.getElementById("target").value.trim());
document.getElementById("target").addEventListener("keydown", e => {
  if (e.key === "Enter") document.getElementById("btn-one").click();
});
document.querySelectorAll("#seg-mode button").forEach(b => {
  b.onclick = () => setMode({ mode: b.dataset.mode });
});
document.getElementById("btn-receiving").onclick = () =>
  setMode({ receiving: !(S.state && S.state.receiving !== false) });
document.getElementById("btn-save").onclick = save;
document.getElementById("btn-quit").onclick = quit;
document.getElementById("detail-close").onclick = () => {
  S.selected = null;
  document.getElementById("detail").hidden = true;
  document.getElementById("f-jump").value = "";
  renderMap();
};

/* -------------------------------------------------------- LSTM predict */
async function predictStation() {
  if (!S.selected) return;
  const hint = document.getElementById("predict-status");
  const box = document.getElementById("predict-result");
  hint.textContent = "training model…";
  hint.className = "hint working";
  box.hidden = true;
  try {
    const r = await fetch(apiUrl("/api/predict"), {
      method: "POST", ...CRED,
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({label: S.selected})
    });
    const d = await r.json();
    if (!r.ok) { hint.textContent = d.error || "failed"; hint.className = "hint err"; return; }
    const qs = S.state.quantities || [];
    const s = (S.state.stations || []).find(x => x.label === S.selected);
    let html = `<table><thead><tr><th>quantity</th><th>measured</th>` +
      `<th>predicted</th><th>delta</th></tr></thead><tbody>`;
    for (const Q of qs) {
      const m = s && s.values ? s.values[Q.name] : null;
      const p = d.predicted[Q.name];
      const delta = m != null && p != null ? (p - m).toFixed(Q.decimals ?? 1) : "—";
      html += `<tr><td>${Q.name}</td><td>${m ?? "—"} ${Q.unit}</td>` +
        `<td class="pred">${p ?? "—"} ${Q.unit}</td>` +
        `<td class="${Math.abs(parseFloat(delta)) > (Q.hi - Q.lo) * 0.15 ? "warn" : ""}">${delta}</td></tr>`;
    }
    html += `</tbody></table>`;
    html += `<div class="hint">model: ${d.model.epochs} epochs, ` +
      `loss ${d.model.loss.toFixed(6)}, ${d.model.samples} samples, ` +
      `${d.model.missing} missing values</div>`;
    box.innerHTML = html;
    box.hidden = false;
    hint.textContent = "done";
    hint.className = "hint ok";
  } catch (e) {
    hint.textContent = e.message;
    hint.className = "hint err";
  }
}

poll();
setInterval(poll, 2000);
