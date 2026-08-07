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

const S = { state: null, quantity: "temperature", selected: null };

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
async function poll() {
  try {
    const r = await fetch("/api/state");
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
  renderMap();
  renderRequests();
  renderRejects();
  renderLink();
  if (S.selected) renderDetail(S.selected);
}

function renderLink() {
  const r = S.state.robot || {};
  const on = !!r.online;
  document.getElementById("linkdot").className = "dot" + (on ? " on" : "");
  /* Position AND speed: the robot transmits both, so the link line shows
     both. A moving robot with a stale-looking position is the one thing
     this line has to be able to tell you at a glance. */
  const where = r.pose ? ` // x ${r.pose.x} y ${r.pose.y}` : "";
  const fast = r.velocity ? ` // ${r.velocity.speed.toFixed(2)} m/s` : "";
  document.getElementById("linktext").textContent =
    on ? `${r.robot} online${where}${fast}` : "no robot";
}

function tile(k, v, sub, warn) {
  return `<div class="tile${warn ? " warn" : ""}"><div class="k">${k}</div>
          <div class="v">${v}${sub ? ` <small>${sub}</small>` : ""}</div></div>`;
}

function renderStrip() {
  const s = S.state.summary;
  const v = (S.state.robot || {}).velocity;
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
    svg += `<text class="rowlab" x="-4.85" y="${y + 0.07}">R${i + 1}</text>`;
  });

  st.forEach(s => {
    const v = s.values ? s.values[S.quantity] : null;
    const t = norm(v);
    const colour = s.measured ? ramp(S.quantity, t) : "none";
    const stroke = s.measured ? colour : "var(--ink-dim)";
    const a = 0.085;   // half the arm of the drawn cross, in metres
    const sel = S.selected === s.label ? " sel" : "";
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
      <rect class="chip" x="${s.x - 0.30}" y="${s.y - 0.44}" width="0.60"
            height="0.22" rx="0.05" fill="${colour}"/>
      <text class="val" x="${s.x}" y="${s.y - 0.27}" fill="${ink}"
            transform="translate(0,${2 * (s.y - 0.27)}) scale(1,-1)">${label}</text>`;
    svg += `<g class="station${sel}" data-label="${s.label}">
      <title>${s.label}${v == null ? " — not measured" :
        ` — ${S.quantity} ${v} ${Q ? Q.unit : ""}`}</title>
      ${s.flags && s.flags.length ? `<circle class="halo" cx="${s.x}" cy="${s.y}" r="0.17"/>` : ""}
      ${S.selected === s.label ? `<circle class="halo" cx="${s.x}" cy="${s.y}" r="0.2"/>` : ""}
      <line class="cross" x1="${s.x - a}" y1="${s.y}" x2="${s.x + a}" y2="${s.y}" stroke="${stroke}"/>
      <line class="cross" x1="${s.x}" y1="${s.y - a}" x2="${s.x}" y2="${s.y + a}" stroke="${stroke}"/>
      ${chip}
      <circle cx="${s.x}" cy="${s.y}" r="0.13" fill="transparent"/>
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

function renderRequests() {
  const el = document.getElementById("requests");
  const rs = S.state.requests || [];
  el.innerHTML = rs.length ? rs.slice().reverse().map(r => {
    const pct = r.total ? Math.round(100 * r.done / r.total) : 0;
    return `<div class="req">
      <div><span class="id">${r.request_id}</span> ${r.state}
        ${r.label ? `// ${r.label}` : ""} <span class="id">${r.done}/${r.total}</span></div>
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
  renderMap();
  renderDetail(label);
  const r = await fetch("/api/history/" + encodeURIComponent(label));
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
  document.getElementById("detail-readings").innerHTML = rows +
    `<div class="hint">measured ${s.timestamp || "never"} // parked ${park} from the cross</div>`;

  const photo = document.getElementById("detail-photo");
  const qr = document.getElementById("detail-qr");
  photo.src = s.photo ? "/media/" + s.photo : "";
  photo.style.visibility = s.photo ? "visible" : "hidden";
  qr.src = s.qr ? "/media/" + s.qr : "";
  qr.style.visibility = s.qr ? "visible" : "hidden";
}

function renderHistory(rows) {
  const el = document.getElementById("detail-history");
  if (!rows.length) { el.innerHTML = `<div class="empty">no history</div>`; return; }
  const qs = S.state.quantities.map(x => x.name);
  el.innerHTML = `<table><thead><tr><th>timestamp</th>${
    qs.map(n => `<th>${n}</th>`).join("")}<th>park m</th></tr></thead><tbody>${
    rows.slice(-12).reverse().map(r => `<tr><td>${r.timestamp}</td>${
      qs.map(n => `<td>${r.values[n] ?? "—"}</td>`).join("")
    }<td>${r.parking_error_m ?? "—"}</td></tr>`).join("")}</tbody></table>`;
}

/* ----------------------------------------------------------- command */
async function request(targets) {
  const hint = document.getElementById("cmdhint");
  hint.className = "hint";
  hint.textContent = "signing and publishing…";
  try {
    const r = await fetch("/api/request", {
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

document.getElementById("btn-all").onclick = () => request("ALL");
document.getElementById("btn-one").onclick = () =>
  request(document.getElementById("target").value.trim());
document.getElementById("target").addEventListener("keydown", e => {
  if (e.key === "Enter") document.getElementById("btn-one").click();
});
document.getElementById("detail-close").onclick = () => {
  S.selected = null;
  document.getElementById("detail").hidden = true;
  renderMap();
};

poll();
setInterval(poll, 2000);
