#!/usr/bin/env python3
"""Generate an HTML spec sheet (dimensioned drawings + tables) for the YouBot,
matching the Gazebo URDF. Rendered to PDF with Chromium headless."""

import os

# ---- helpers to draw dimensioned SVG (real mm -> px) ------------------------

def dim_h(x1, x2, y, label, above=True, col="#c0392b"):
    """Horizontal dimension line between x1,x2 at height y with arrow ticks."""
    o = -6 if above else 14
    ty = y + (-9 if above else 22)
    return f'''
      <line x1="{x1}" y1="{y}" x2="{x2}" y2="{y}" class="dim"/>
      <line x1="{x1}" y1="{y-4}" x2="{x1}" y2="{y+4}" class="dim"/>
      <line x1="{x2}" y1="{y-4}" x2="{x2}" y2="{y+4}" class="dim"/>
      <polygon points="{x1},{y} {x1+7},{y-3} {x1+7},{y+3}" fill="{col}"/>
      <polygon points="{x2},{y} {x2-7},{y-3} {x2-7},{y+3}" fill="{col}"/>
      <text x="{(x1+x2)/2}" y="{ty}" class="dimtxt" text-anchor="middle">{label}</text>'''


def dim_v(y1, y2, x, label, left=True, col="#2c6fbb"):
    """Vertical dimension line between y1,y2 at x with arrow ticks."""
    tx = x + (-8 if left else 8)
    anc = "end" if left else "start"
    return f'''
      <line x1="{x}" y1="{y1}" x2="{x}" y2="{y2}" class="dim"/>
      <line x1="{x-4}" y1="{y1}" x2="{x+4}" y2="{y1}" class="dim"/>
      <line x1="{x-4}" y1="{y2}" x2="{x+4}" y2="{y2}" class="dim"/>
      <polygon points="{x},{y1} {x-3},{y1+7} {x+3},{y1+7}" fill="{col}"/>
      <polygon points="{x},{y2} {x-3},{y2-7} {x+3},{y2-7}" fill="{col}"/>
      <text x="{tx}" y="{(y1+y2)/2+4}" class="dimtxt" text-anchor="{anc}">{label}</text>'''


# ---- SIDE VIEW (X-Z plane), scale 0.42 px/mm -------------------------------
def side_view():
    s = 0.42
    W, H = 520, 380
    gx = 210         # ground line x offset (px) for X=0 datum (base centre)
    gy = 330         # ground line y (px) = Z=0
    def px(xmm): return gx + xmm * s
    def pz(zmm): return gy - zmm * s
    # base body: X from -290..+290 (len 580), Z 30..170 (body 140 above 30mm clr)
    bx0, bx1 = px(-290), px(290)
    bz0, bz1 = pz(170), pz(30)
    # wheels dia100 centered z=50, at x=-228 and +228
    r = 50 * s
    wy = pz(50)
    wxr, wxf = px(228), px(-228)  # rear(-228) front(+228) -- +X is forward(right)
    # pedestal column x 120, from z170..550, width 100
    ped_cx = px(120)
    pw = 50 * s
    pz0, pz1 = pz(550), pz(170)
    # camera mast x=220, camera at z=600
    mx = px(220)
    cam_y = pz(600)
    # arm folded schematic from pedestal top
    return f'''
    <svg viewBox="0 0 {W} {H}" class="draw">
      <line x1="20" y1="{gy}" x2="{W-10}" y2="{gy}" class="ground"/>
      <text x="{W-10}" y="{gy+16}" class="note" text-anchor="end">sol (Z=0)</text>
      <!-- base -->
      <rect x="{bx0}" y="{bz0}" width="{bx1-bx0}" height="{bz1-bz0}" class="body"/>
      <!-- rear carrying plate -->
      <rect x="{px(-260)}" y="{pz(200)}" width="{280*s}" height="{30*s}" class="plate"/>
      <!-- wheels -->
      <circle cx="{wxr}" cy="{wy}" r="{r}" class="wheel"/>
      <circle cx="{wxf}" cy="{wy}" r="{r}" class="wheel"/>
      <!-- pedestal column -->
      <rect x="{ped_cx-pw/2}" y="{pz0}" width="{pw}" height="{pz1-pz0}" class="ped"/>
      <!-- arm folded (schematic) -->
      <line x1="{ped_cx}" y1="{pz(550)}" x2="{px(60)}" y2="{pz(720)}" class="arm"/>
      <line x1="{px(60)}" y1="{pz(720)}" x2="{px(230)}" y2="{pz(690)}" class="arm"/>
      <circle cx="{ped_cx}" cy="{pz(550)}" r="5" class="joint"/>
      <circle cx="{px(60)}" cy="{pz(720)}" r="4" class="joint"/>
      <!-- camera mast + head -->
      <line x1="{mx}" y1="{pz(160)}" x2="{mx}" y2="{cam_y}" class="mast"/>
      <rect x="{mx-4}" y="{cam_y-9}" width="20" height="18" class="cam"/>
      <text x="{mx+22}" y="{cam_y+4}" class="note">caméra</text>
      <text x="{px(120)}" y="{pz(560)}-6" class="note" text-anchor="middle" transform="translate(0,-4)">bras 5-DOF</text>
      <!-- dimensions -->
      {dim_h(bx0, bx1, pz(170)+70, "580 (longueur)")}
      {dim_v(pz(170), pz(30), px(-290)-16, "140")}
      {dim_v(gy, pz(100), px(-290)-34, "Ø100")}
      {dim_v(gy, cam_y, px(240)+95, "600 (caméra)", left=False)}
      {dim_v(gy, pz(550), px(120)-82, "550 (épaule)")}
      <text x="{gx}" y="{gy+16}" class="datum" text-anchor="middle">X=0</text>
    </svg>'''


# ---- TOP VIEW (X-Y plane) --------------------------------------------------
def top_view():
    s = 0.42
    W, H = 470, 330
    cx, cy = 235, 165
    def px(xmm): return cx + xmm * s   # X horizontal (forward = right)
    def py(ymm): return cy - ymm * s   # Y vertical (left = up)
    bx0, bx1 = px(-290), px(290)
    by0, by1 = py(190), py(-190)
    r = 26 * s
    # wheels at (+-228, +-178)
    wheels = ""
    for xmm in (228, -228):
        for ymm in (178, -178):
            wheels += f'<rect x="{px(xmm)-50*s/2}" y="{py(ymm)-r}" width="{50*s}" height="{2*r}" rx="6" class="wheel"/>'
    # arm base circle at (120,0)
    armc = f'<circle cx="{px(120)}" cy="{py(0)}" r="{55*s}" class="ped"/>'
    cam = f'<rect x="{px(220)-5}" y="{py(0)-14}" width="16" height="28" class="cam"/>'
    return f'''
    <svg viewBox="0 0 {W} {H}" class="draw">
      <rect x="{bx0}" y="{by0}" width="{bx1-bx0}" height="{by1-by0}" class="body"/>
      {wheels}
      {armc}{cam}
      <text x="{px(120)}" y="{py(0)+4}" class="note" text-anchor="middle">bras</text>
      <line x1="{px(-330)}" y1="{cy}" x2="{px(330)}" y2="{cy}" class="axis"/>
      <line x1="{cx}" y1="{py(-230)}" x2="{cx}" y2="{py(230)}" class="axis"/>
      <text x="{px(330)}" y="{cy-6}" class="note" text-anchor="end">+X (avant)</text>
      <text x="{cx+6}" y="{py(230)+2}" class="note">+Y (gauche)</text>
      {dim_h(bx0, bx1, by0-40, "580")}
      {dim_h(px(-228), px(228), by1+38, "456 (empattement)")}
      {dim_v(py(190), py(-190), bx1+40, "380", left=False)}
      {dim_v(py(178), py(-178), bx0-16, "356 (voie)")}
    </svg>'''


# ---- ARM KINEMATICS --------------------------------------------------------
def arm_view():
    s = 0.5
    W, H = 470, 330
    x0, y0 = 120, 300   # shoulder base in px
    # draw arm extended up then out: segments 75,155,135,135,130
    def seg(x, y, dx, dy):
        return x+dx*s, y-dy*s
    pts = [(x0, y0)]
    x, y = x0, y0
    for dx, dy, in [(0,75),(20,150),(60,120),(90,95),(85,60)]:
        x, y = seg(x, y, dx, dy); pts.append((x, y))
    segs = ""
    labels = ["75 (épaule)","155","135","135","130 (poignet)"]
    jlabels = ["J1 (Z)","J2 (Y)","J3 (Y)","J4 (Y)","J5 (Z)"]
    for i in range(5):
        (ax, ay), (bx, by) = pts[i], pts[i+1]
        segs += f'<line x1="{ax}" y1="{ay}" x2="{bx}" y2="{by}" class="arm2"/>'
        segs += f'<circle cx="{ax}" cy="{ay}" r="6" class="joint"/>'
        mx, my = (ax+bx)/2, (ay+by)/2
        segs += f'<text x="{mx+10}" y="{my}" class="seglab">{labels[i]}</text>'
        segs += f'<text x="{ax-10}" y="{ay+4}" class="jlab" text-anchor="end">{jlabels[i]}</text>'
    # gripper
    gx, gy = pts[-1]
    segs += f'<circle cx="{gx}" cy="{gy}" r="6" class="joint"/>'
    segs += f'<line x1="{gx}" y1="{gy}" x2="{gx+18}" y2="{gy-24}" class="grip"/>'
    segs += f'<line x1="{gx}" y1="{gy}" x2="{gx+30}" y2="{gy-14}" class="grip"/>'
    segs += f'<text x="{gx+22}" y="{gy-28}" class="jlab">pince (~70 max)</text>'
    return f'''
    <svg viewBox="0 0 {W} {H}" class="draw">
      <line x1="30" y1="{y0}" x2="{W-20}" y2="{y0}" class="ground"/>
      <rect x="{x0-25}" y="{y0}" width="50" height="18" class="ped"/>
      {segs}
      <text x="{x0}" y="{y0+34}" class="note" text-anchor="middle">colonne / base bras</text>
      <text x="{W-20}" y="40" class="note" text-anchor="end">Portée totale ~ 630 mm</text>
    </svg>'''


# ---- tables ----------------------------------------------------------------
def table(title, rows):
    body = "".join(f"<tr><td>{a}</td><td class='v'>{b}</td></tr>" for a, b in rows)
    return f'<table><caption>{title}</caption><tbody>{body}</tbody></table>'

base_t = table("Châssis (base mobile)", [
    ("Longueur (X)", "580 mm"), ("Largeur (Y)", "380 mm"),
    ("Hauteur du corps (Z)", "140 mm"), ("Garde au sol", "~25 mm"),
    ("Plateau porteur arrière", "280 × 340 × 30 mm"), ("Masse", "~20 kg")])
wheel_t = table("Roues mécanum (× 4)", [
    ("Diamètre", "100 mm"), ("Largeur", "50 mm"),
    ("Empattement (avant↔arrière)", "456 mm (±228)"),
    ("Voie (gauche↔droite)", "356 mm (±178)"),
    ("Hauteur d'axe", "50 mm"), ("Rouleaux", "12 à 45°")])
arm_t = table("Bras 5 axes — segments", [
    ("Socle → épaule (J1)", "75 mm"), ("Épaule → coude (J2)", "155 mm"),
    ("Coude → avant-bras (J3)", "135 mm"), ("Avant-bras → poignet (J4)", "135 mm"),
    ("Poignet → bride (J5)", "130 mm"), ("Portée totale", "~630 mm"),
    ("Colonne (épaule / sol)", "550 mm")])
lim_t = table("Limites articulaires", [
    ("J1 (base, Z)", "±169°"), ("J2 (épaule, Y)", "−65° … +90°"),
    ("J3 (coude, Y)", "−151° … +146°"), ("J4 (poignet, Y)", "±102,5°"),
    ("J5 (rotation, Z)", "±167,5°")])
grip_t = table("Pince 2 doigts", [
    ("Corps", "50 × 80 × 30 mm"), ("Longueur doigts", "50 mm"),
    ("Ouverture max", "~70 mm"), ("Course / doigt", "0 → 35 mm")])
cam_t = table("Caméra + capteurs", [
    ("Hauteur caméra", "600 mm"), ("Mât", "Ø15 × 440 mm"),
    ("Boîtier caméra", "30 × 90 × 30 mm"), ("Lidar (hauteur axe)", "200 mm"),
    ("Lidar", "360°, 12 m")])

HTML = f'''<!doctype html><html><head><meta charset="utf-8"><style>
@page {{ size: A4; margin: 14mm; }}
* {{ box-sizing: border-box; }}
body {{ font-family: "DejaVu Sans", Arial, sans-serif; color:#1a1a1a; margin:0; font-size:11px; }}
h1 {{ font-size:20px; margin:0 0 2px; color:#14532d; }}
.sub {{ color:#555; margin:0 0 10px; font-size:11px; }}
h2 {{ font-size:13px; color:#14532d; border-bottom:2px solid #b7d9bf; padding-bottom:3px; margin:16px 0 8px; }}
.conv {{ background:#f0f7f1; border:1px solid #cfe6d4; border-radius:6px; padding:8px 10px; font-size:10.5px; }}
.draw {{ width:100%; height:auto; background:#fbfbfb; border:1px solid #e2e2e2; border-radius:6px; }}
.body {{ fill:#ffd9a6; stroke:#d98b2b; stroke-width:1.5; }}
.plate {{ fill:#cfcfcf; stroke:#999; stroke-width:1; }}
.wheel {{ fill:#333; stroke:#000; stroke-width:1; }}
.ped {{ fill:#b8b8be; stroke:#666; stroke-width:1.2; }}
.arm, .arm2 {{ stroke:#e07a1f; stroke-width:5; stroke-linecap:round; fill:none; }}
.grip {{ stroke:#444; stroke-width:3; stroke-linecap:round; }}
.mast {{ stroke:#777; stroke-width:3; }}
.cam {{ fill:#222; }}
.joint {{ fill:#2c6fbb; stroke:#fff; stroke-width:1.5; }}
.ground {{ stroke:#333; stroke-width:1.5; }}
.axis {{ stroke:#bbb; stroke-width:1; stroke-dasharray:4 3; }}
.dim {{ stroke:#c0392b; stroke-width:0.8; }}
.dimtxt {{ fill:#c0392b; font-size:11px; font-weight:bold; }}
.note {{ fill:#555; font-size:10px; }}
.datum {{ fill:#111; font-size:10px; font-weight:bold; }}
.seglab {{ fill:#b45309; font-size:11px; font-weight:bold; }}
.jlab {{ fill:#2c6fbb; font-size:10px; }}
.grid2 {{ display:grid; grid-template-columns:1fr 1fr; gap:10px; }}
.grid3 {{ display:grid; grid-template-columns:1fr 1fr; gap:10px; }}
table {{ width:100%; border-collapse:collapse; margin-bottom:6px; }}
caption {{ text-align:left; font-weight:bold; color:#14532d; font-size:11px; padding:4px 0; }}
td {{ border:1px solid #ddd; padding:3px 6px; }}
td.v {{ text-align:right; font-variant-numeric:tabular-nums; white-space:nowrap; }}
tr:nth-child(even) td {{ background:#f7f7f7; }}
.foot {{ margin-top:10px; color:#888; font-size:9px; text-align:center; }}
</style></head><body>

<h1>KUKA YouBot — Spécification dimensionnelle</h1>
<p class="sub">Robot mobile de cueillette autonome · Serre de fraises · PFA « Smart Agriculture — Digital Twin » · dimensions en mm</p>
<div class="conv"><b>Repère :</b> origine (0,0,0) = centre de l'empreinte du châssis, au sol. &nbsp; <b>+X</b> = avant · <b>+Y</b> = gauche · <b>+Z</b> = haut. &nbsp; Ces cotes correspondent au vrai KUKA YouBot et à l'URDF de simulation (Gazebo).</div>

<h2>1 · Vue de profil (plan X–Z)</h2>
{side_view()}

<div class="grid2">
  <div>
    <h2>2 · Vue de dessus (plan X–Y)</h2>
    {top_view()}
  </div>
  <div>
    <h2>3 · Cinématique du bras</h2>
    {arm_view()}
  </div>
</div>

<h2>4 · Tableaux de cotes</h2>
<div class="grid3">
  <div>{base_t}{wheel_t}{grip_t}</div>
  <div>{arm_t}{lim_t}{cam_t}</div>
</div>

<p class="foot">Généré pour le rapport PFA · dépôt ROS2_stage · les meshes CAO (FreeCAD/CATIA) se substitueront aux primitives sans changer la cinématique.</p>
</body></html>'''

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "youbot_dimensions.html")
with open(out, "w") as f:
    f.write(HTML)
print("wrote", out)
