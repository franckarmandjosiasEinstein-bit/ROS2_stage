/* AGRI-CLOUD measurement table.
 *
 * One row per plant, the two sides side by side, media as links. The page
 * is a pure render of /api/state, exactly like the map view -- there is no
 * second source of truth and no second poll loop worth the name.
 *
 * WHY THE PLANT IS THE ROW
 *
 * Both stations of a plant measure the same plant from opposite sides. The
 * comparison between them is the reading that carries information: a dry
 * patch shows up as R and L both departing from the field, and a sensor
 * fault shows up as one of them departing alone. Laid out one station per
 * row, those two cases look identical unless you scroll.
 */

const T = { token: "", state: null, onlyMeasured: false };

/* The token normally rides in an HttpOnly cookie the server set on
   first contact; credentials:"same-origin" is what sends it. The
   ?token= path below is the fallback for a browser without cookies. */
const CRED = { credentials: "same-origin" };

(function () {
  const p = new URLSearchParams(window.location.search);
  if (p.has("token")) T.token = p.get("token");
  /* And back again, so the two pages are navigable in both directions
     without the operator re-pasting a token. */
  for (const a of document.querySelectorAll('a.navlink[href="/"]')) {
    if (T.token) a.href = "/?token=" + encodeURIComponent(T.token);
  }
})();

/* Same helper as the map page: the token rides on every API and media URL.
   Written out rather than imported because these two pages load
   independently and a shared module would need a build step. */
const apiUrl = path => path + (T.token ? (path.includes("?") ? "&" : "?")
                                       + "token=" + encodeURIComponent(T.token) : "");

const clock = iso => iso ? new Date(iso).toLocaleString() : "—";

/* A value cell. Missing is an em dash, never a zero: a station that was
   never visited and one that read 0.0 are different facts. */
function val(v, digits) {
  return v === null || v === undefined ? "—" : Number(v).toFixed(digits);
}

/* Media cells are LINKS, not thumbnails. Ninety-six images inline would
   make the page unusable on the laptop it is demonstrated from, and the
   thing wanted here is the path to the evidence, openable in a tab. */
function media(path, label) {
  if (!path) return `<td class="mono dim">—</td>`;
  return `<td class="mono"><a href="${apiUrl("/media/" + path)}"`
       + ` target="_blank" rel="noopener">${label}</a></td>`;
}

function sideCells(s) {
  if (!s || !s.measured) {
    return `<td class="dim">not measured</td>`
         + `<td class="dim" colspan="5">—</td>`
         + `<td class="dim">—</td><td class="dim">—</td>`;
  }
  const v = s.values || {};
  return `<td class="mono when">${clock(s.timestamp)}</td>`
       + `<td class="mono num">${val(v.temperature, 1)}</td>`
       + `<td class="mono num">${val(v.humidity, 1)}</td>`
       + `<td class="mono num">${val(v.luminosity, 0)}</td>`
       + `<td class="mono num">${val(v.co2, 0)}</td>`
       + `<td class="mono num">${val(v.ph, 2)}</td>`
       + media(s.photo, "photo")
       + media(s.qr, "QR");
}

function render() {
  if (!T.state) return;
  const by = {};
  for (const s of T.state.stations) by[s.label] = s;

  const rows = [];
  let shown = 0, measured = 0;
  /* Row order is the catalogue's own survey order -- row 1 plant 1 through
     row 3 plant 8 -- and NOT the order the robot drove. The robot reorders
     to save the battery (it sweeps by band); a table that followed the
     driving order would change shape between campaigns for a reason that
     has nothing to do with the plants. */
  for (let r = 1; r <= 3; r++) {
    for (let p = 1; p <= 8; p++) {
      const R = by[`P${r},${p}R`], L = by[`P${r},${p}L`];
      const any = (R && R.measured) || (L && L.measured);
      if (any) measured++;
      if (T.onlyMeasured && !any) continue;
      shown++;
      rows.push(`<tr class="${any ? "" : "empty-row"}">`
        + `<th class="stick">P${r},${p}</th>`
        + sideCells(R) + sideCells(L) + `</tr>`);
    }
  }
  document.getElementById("rows").innerHTML = rows.join("");
  document.getElementById("tablefoot").textContent =
    `${measured} of 24 plants have at least one side measured`
    + ` // ${shown} row(s) shown`
    + ` // right-hand station first, then left, on the same line —`
    + ` store/measurements.csv is written in this same shape`;
}

async function poll() {
  try {
    const r = await fetch(apiUrl("/api/state"), CRED);
    if (r.status === 401) {
      document.getElementById("tablefoot").textContent =
        "401 — this page needs the ?token= the Cloud printed at startup";
      return;
    }
    T.state = await r.json();
    render();
  } catch (e) {
    document.getElementById("tablefoot").textContent = "cloud unreachable: " + e;
  }
}

document.getElementById("only-measured").onchange = e => {
  T.onlyMeasured = e.target.checked;
  render();
};
document.getElementById("dl-csv").onclick = async e => {
  e.preventDefault();
  const r = await fetch(apiUrl("/api/export"), { method: "POST", ...CRED });
  const d = await r.json();
  if (d.url) window.location = apiUrl(d.url);
};

poll();
setInterval(poll, 4000);
