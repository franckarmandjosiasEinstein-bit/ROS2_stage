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
            row: "", side: "", token: "" };

/* Auth token: read from the URL's ?token= parameter if present. Stored in
   memory and sent with every API fetch. The page itself loads without it;
   only the /api/* endpoints check. */
(function () {
  const p = new URLSearchParams(window.location.search);
  if (p.has("token")) S.token = p.get("token");
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
    const r = await fetch(apiUrl("/api/state"));
    if (r.status === 401) {
      S.token = prompt("Dashboard token:") || "";
      return poll();
    }
    S.state = await r.json();
    render();
    document.getElementById("foot").textContent =
      `link up // ${S.state.summary.visits} visits filed // cloud since ${S.state.since}`;
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
    const v = s.values ? s.values[S.quantity] : null;
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
      <rect class="chip" x="${s.x - 0.30}" y="${cy - 0.11}" width="0.60"
            height="0.22" rx="0.05" fill="${colour}"/>
      <line class="stem" x1="${s.x}" y1="${s.y}" x2="${s.x}" y2="${cy}"
            stroke="${colour}"/>
      <text class="val" x="${s.x}" y="${cy - 0.06}" fill="${ink}"
            ${upright(cy - 0.06)}>${label}</text>`;
    svg += `<g class="station${sel}" data-label="${s.label}">
      <title>${s.label}${v == null ? " — not measured" :
        ` — ${S.quantity} ${v} ${Q ? Q.unit : ""}`}</title>
      ${s.flags && s.flags.length ? `<circle class="halo" cx="${s.x}" cy="${s.y}" r="0.13"/>` : ""}
      ${S.selected === s.label ? `<circle class="halo" cx="${s.x}" cy="${s.y}" r="0.16"/>` : ""}
      <line class="cross" x1="${s.x - a}" y1="${s.y}" x2="${s.x + a}" y2="${s.y}" stroke="${stroke}"/>
      <line class="cross" x1="${s.x}" y1="${s.y - a}" x2="${s.x}" y2="${s.y + a}" stroke="${stroke}"/>
      ${chip}
      <circle cx="${s.x}" cy="${s.y}" r="0.05" fill="transparent"/>
    </g>`;
  });

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
  const r = await fetch(apiUrl("/api/history/" + encodeURIComponent(label)));
  renderHistory(await r.json());
  document.getElementById("detail").scrollIntoView({ behavior: "smooth",
                                                     block: "nearest" });
}

function renderDetail(label) {
  const s = (S.state.stations || []).find(x => x.label === label);
  if (!s) return;
  document.getElementById("detail-label").textContent =
    `${label}  //  row ${s.row} plant ${s.plant} ${s.side === "R" ? "right" : "left"}`;

  const rows = (S.state.quantities || []).map(Q => {
    const v = s.values ? s.values[Q.name] : null;
    const t = v == null ? 0 : Math.max(0, Math.min(1, (v - Q.lo) / (Q.hi - Q.lo)));
    const flagged = (s.flags || []).some(f => f.endsWith(":" + Q.name));
    return `<div class="reading${flagged ? " flagged" : ""}">
      <span class="n">${Q.name}</span>
      <span class="bar-t"><i style="width:${(t * 100).toFixed(1)}%;
        background:${ramp(Q.name, t)}"></i></span>
      <span class="val">${v == null ? "—" : v} <small>${Q.unit}</small></span>
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
      body: JSON.stringify({ targets }),
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
    const r = await fetch(apiUrl("/api/export"), { method: "POST" });
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
    const r = await fetch(apiUrl("/api/quit"), { method: "POST" });
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
document.getElementById("btn-save").onclick = save;
document.getElementById("btn-quit").onclick = quit;
document.getElementById("detail-close").onclick = () => {
  S.selected = null;
  document.getElementById("detail").hidden = true;
  document.getElementById("f-jump").value = "";
  renderMap();
};

poll();
setInterval(poll, 2000);
