# cloud_agri — smart agriculture: a mobile node and a Cloud

A greenhouse with 24 plants and 48 red crosses painted on the floor. A Cloud
asks for one plant or all of them. A robot drives onto the crosses, reads the
plant's environment, photographs it, turns the numbers into a QR code, seals
the lot with elliptic-curve cryptography and sends it back. The Cloud opens
it, checks it, files it, and draws it.

```
   CLOUD  ──── signed request ────►  NODE(S)    agri/v1/request
     ▲                                 │
     │                                 ├─ drive to the cross (odometry, then
     │                                 │   the floor camera on the marker)
     │                                 ├─ read t / RH / lux / CO2 / pH
     │                                 ├─ photograph the plant
     │                                 ├─ numbers ─► QR code
     │                                 └─ {QR, photo, numbers} ─► ECC seal
     └──── sealed JSON ────────────────┘         agri/v1/report/<node_id>
```

Topics are namespaced by `node_id` for robot→Cloud messages
(`agri/v1/status/<node_id>`, `.../ack/<node_id>`, `.../report/<node_id>`);
the Cloud subscribes with MQTT wildcards (`+`) to hear all nodes. Requests
stay flat because they address the fleet. Every status message carries a
`node_kind` field (`"mobile"` / `"fixed"`) so the Cloud can tell a driving
robot from a stationary ESP.

---

## 1. What is where

| | |
|---|---|
| `agri/` | everything that is not ROS and not a transport. Labels, geometry, crypto, QR, sensors, envelope, the robot's brain, the Cloud, the dashboard. |
| `agri/cloud/` | the Cloud: MQTT client, HTTP server, store, dashboard |
| `agri/world/` | generators: the world with its crosses, the robot with its floor camera |
| `ros2/src/agri_robot/` | the ROS 2 package: the body. Wheels, cameras, launch file. |
| `worlds/`, `urdf/` | **generated** — do not edit, regenerate |
| `tests/check_cloud.py` | 235 pre-flight checks, none of which need a broker, ROS or a network |

The split is deliberate. Everything that can be tested without a simulator
lives outside ROS and is tested on every run of `check_cloud.py`; the part
that genuinely needs Gazebo is one file, `ros2/src/agri_robot/agri_robot/driver.py`.
The offline demo and the real robot run **the same** mission code, the same
crypto and the same store — only the body and the transport are swapped.

The Phase B harvesting project (`../src/`, `../scripts/`) is untouched and
still works. This project *reads* its greenhouse and its robot to generate
its own; it never writes to them, and the test suite asserts that.

---

## 2. Install

```bash
cd cloud_agri
sudo apt install mosquitto mosquitto-clients      # the MQTT broker

# Ubuntu 24.04 and any other PEP 668 distribution refuse a system-wide pip
# install ("externally-managed-environment"). Use a virtual environment --
# and create it with --system-site-packages, or ROS 2 will vanish from it.
python3 -m venv --system-site-packages ~/.venvs/agri
source ~/.venvs/agri/bin/activate
pip install -e ".[cloud]"        # the [cloud] extra adds the QR decoders
```

**`--system-site-packages` is not optional if you intend to run the robot.**
`rclpy`, `sensor_msgs` and the rest of ROS 2 are installed system-wide by
apt; a plain `python3 -m venv` hides them, and `ros2 launch agri_robot ...`
then fails to import `rclpy` — or, with the venv deactivated, fails to import
`agri`. With `--system-site-packages` one interpreter sees both. Order for a
session that uses Gazebo:

```bash
source ~/.venvs/agri/bin/activate     # first: gives you agri
source /opt/ros/jazzy/setup.bash      # then:  gives you ros2
source install/setup.bash             # then:  gives you agri_robot
```

If you only want the offline demo and the dashboard, a plain venv is fine and
ROS is never needed.

`[cloud]` pulls in **zxing-cpp** and **opencv-python-headless**. The Cloud
re-reads every QR image it receives and checks it against the numbers that
travelled with it, so it needs a decoder. Both are listed because OpenCV
fails to locate 2 of the 48 station codes and zxing reads all 48; either
alone works, both is what the tests run against.

**OpenCV is pinned below 5 on purpose.** `opencv-python-headless` 5.x
declares `numpy>=2`, and installing that into a `--system-site-packages` venv
on top of ROS 2 Jazzy shadows the system numpy 1.26 that every rosidl Python
extension was *compiled* against. That is not a warning — it fails when the
first `LaserScan` or `Odometry` is deserialised, inside a callback, after the
simulator is already up. If pip has already done it to you:

```bash
pip install 'numpy<2' 'opencv-python-headless<5'
python3 -c "import numpy, sensor_msgs.msg, nav_msgs.msg; print(numpy.__version__, 'ok')"
```

`check_cloud.py` checks the pin, and checks the loaded numpy whenever rclpy
is importable in the same interpreter.

---

## 3. Five minutes, no simulator, no broker

```bash
python3 -m agri.demo --explain
```

Runs the whole chain in one process: the Cloud signs a request, the robot
verifies it, drives (simulated), reads, photographs, QR-encodes, seals,
transmits; the Cloud decrypts, checks and files. `--explain` then prints the
QR payload as a scanner reads it, the sealed envelope exactly as it goes on
the wire, the same message after the Cloud opens it, and replays two attacks
that are both refused.

```bash
python3 -m agri.demo ALL --serve       # all 48, then a dashboard on :8088
```

The dashboard's buttons issue **real** signed requests that the robot really
verifies. Nothing about the chain is faked; only the wheels and the broker
are.

---

## 4. The real thing: Gazebo + MQTT + the Cloud

Four terminals. Order matters only for the broker.

```bash
# 0. once: generate the world, the robot, and both key pairs
cd cloud_agri
python3 -m agri.world.make_world      # -> worlds/greenhouse_cloud.sdf (48 crosses)
python3 -m agri.world.make_robot      # -> urdf/youbot_agri.urdf (floor camera)
python3 -m agri.keys                  # -> keys/{cloud,robot}_{private,public}.pem

# 1. the broker
mosquitto -p 1883 -v

# 2. the Cloud -- this is the operator's seat
agri-cloud --keys keys --store store            # console here, dashboard on :8088

# 3. the robot, in Gazebo -- from the WORKSPACE root, not from cloud_agri/
cd ..
colcon build --symlink-install \
    --base-paths cloud_agri/ros2/src --packages-select agri_robot
source install/setup.bash
ros2 launch agri_robot agri.launch.py
```

RViz opens beside Gazebo with the **2D view**: the outline the lidar actually
perceives, drawn on top of where the catalogue says the walls, gutters and 48
crosses are, plus the robot's own footprint including its sensor boom. That
comparison is the point — a scan on its own is a pretty shape with no scale,
and the moment the two stop lying on top of each other, either the odometry
or the catalogue is wrong. `rviz:=false` turns it off.

The **Gazebo camera locks onto the robot** and stays locked. It has to be
re-asserted rather than requested once: gz answers "yes" to the tracking
request before its render scene actually contains the entity, and it drops
the track again on its own several minutes in — well inside a 48-station
sweep. If the lock never takes, the camera falls back to an overview of the
whole greenhouse from (0, −7, 11), which cannot fail because it names a pose
rather than an entity.

### Giving orders

`agri-cloud` is a prompt, not just a daemon. The orders come from the process
that holds the private key:

```
cloud> P2,4
  request 7c1e0f42a9d3 signed and published -> 2 station(s): P2,4R, P2,4L
cloud> status
  robot   youbot-01  online  (busy)
  at      x=-0.44  y=-1.75  yaw=0.0 deg
  moving  0.312 m/s   (vx=0.312 vy=-0.004 wz=0.0)
  request 7c1e0f42a9d3  driving   0/2  P2,4R
cloud> show P2,4R
  P2,4R   2026-08-05T11:42:07Z
    temperature       22.4 degC
    humidity          48.2 %RH
    ...
    parked       0.008 m from the cross
    speed        0.002 m/s   (vx=0.002 vy=0.001 wz=0.0)
cloud> ALL
cloud> coverage
cloud> csv
cloud> quit
```

Anything that is not a verb is read as a target, because a plant name is what
gets typed nine times out of ten. The dashboard's buttons do exactly the same
thing over HTTP; the console exists because the operator's seat belongs in
the process that signs.

### What the Cloud can ask for

The brief is written in terms of plants, so the request is too:

| asked for | stations visited |
|---|---|
| `ALL` | all 48, in survey order |
| `P2,5` | **a plant**: `P2,5R` then `P2,5L` |
| `P2,5R` | one side of one plant |
| `P1,3;P3,7` | any mixture |

A plant is not a station — it has a measurement point on each side of it — so
asking for `P2,5` sends the robot to both. Press **SWEEP ALL 48** on the
dashboard, type a plant into the box beside REQUEST, or issue it from the
command line:

```bash
agri-cloud --keys keys --store store --request "P2,4"
```

Every one of those is signed by the Cloud's private key before it leaves, and
the robot refuses anything that is not.

`--base-paths` is needed because `agri_robot` lives under
`cloud_agri/ros2/src/`, not in the workspace's own `src/`, which is Phase B's
and stays Phase B's. `install/`, `build/` and `log/` are shared, so the two
projects coexist in one workspace without either knowing about the other.

The node does **not** inherit your virtual environment. colcon writes the
console script with the shebang of the interpreter that ran `setup.py`, and
colcon is a system package, so the node starts under `/usr/bin/python3` no
matter what is activated. The launch file therefore hands it a `PYTHONPATH`
built from `cloud_agri/` and from `$VIRTUAL_ENV`'s site-packages. Nothing to
do — but it is why `VIRTUAL_ENV` must be set when you launch, i.e. keep the
venv activated.

`--symlink-install` is worth the flag here: the launch file then resolves back
into the source tree and finds the generated world and robot even if
`setup.py`'s data files did not land where you expected. If it ever cannot
find them, it says so and tells you to set `AGRI_HOME`:

```bash
export AGRI_HOME=/path/to/cloud_agri
```

Stopping the launch stops everything, including the processes `gz sim` forks
behind itself — it reuses Phase B's `kill_sim.sh`.

Step 0's keys are generated on first use anyway, by whichever side starts
first; running it explicitly just means you can see the files, and see that
`keys/` is in `.gitignore`. **A private key that has been committed has been
published** — rotating is the only fix, so the cheap defence is to never let
it happen. `check_cloud.py` asserts no `.pem` is in the tree.

### Putting the Cloud on another computer

This is the deployment the brief actually describes, and it needs no code
change — MQTT was chosen for exactly this. Neither side ever learns the
other's address; both only need the **broker's**.

```bash
# on the CLOUD machine (say 192.168.1.20)
mosquitto -c mosquitto.conf              # see the listener note below
agri-cloud --broker localhost --keys keys --store store

# on the ROBOT machine
ros2 launch agri_robot agri.launch.py broker:=192.168.1.20
```

The broker can equally live on a third machine, or on the robot: pass the
same `--broker` / `broker:=` address to both sides and it works.

Three things to get right, and only the first has ever been anyone's
fault twice:

**Copy the public keys.** Each side needs its own private key and the
*other side's public* one. On the robot machine:

```bash
scp cloudpc:cloud_agri/keys/cloud_public.pem  keys/
scp keys/robot_public.pem  cloudpc:cloud_agri/keys/
```

Nothing is generated for you across two machines. That is deliberate: a key
invented locally cannot match, and the failure has no symptom of its own —
the robot refuses every order as unsigned and the Cloud rejects every
report, both silently, which reads as a broker fault. `agri.keys` now
refuses instead and prints the `scp` line you need. A directory that is
completely empty is still bootstrapped with both pairs, so the
one-machine case stays one command.

**Mosquitto listens only on loopback by default** (2.x). A remote robot
connecting to it gets a bare connection refusal. The smallest config that
works on a lab network:

```
listener 1883 0.0.0.0
allow_anonymous true
```

**There is no TLS and no broker password.** Anyone on the network can watch
the traffic and publish on the topics. That is survivable here only because
the payloads are already ECIES-sealed and signed — an eavesdropper reads
ciphertext, and a forged request is refused for having no valid Cloud
signature. But the *metadata* is in clear, and an attacker can flood the
topics. On a real deployment, put mosquitto behind TLS with a password
file; nothing in this project has to change for that.

The dashboard binds every interface, so `http://192.168.1.20:8088` works
from the robot's machine or a phone on the same network. `agri-cloud`
prints that address at startup rather than `localhost`.

**Dashboard authentication.** By default `agri-cloud` generates a random
token and prints the full URL including it — share that URL with anyone who
needs the dashboard. The token is checked on every API request and on
`/media/` (query parameter `?token=…` or `Authorization: Bearer …`), because
a photograph and a CSV row are measurements just as much as the JSON that
describes them, and a token that guards one but not the other guards
nothing. To choose your own token, pass `--dashboard-token SECRET`; to
disable authentication entirely (useful for local-only demos), pass
`--dashboard-token ''`. The offline demo (`agri.demo --serve`) runs without
a token.

ROS 2's own DDS traffic never crosses the network: Gazebo, RViz and the
bridge all stay on the robot's machine. So `ROS_DOMAIN_ID` is irrelevant
here, and the two computers do not need to be on the same subnet for ROS's
sake — only reachable from the broker.

### Driving without a Cloud

To watch the robot find its crosses before any of the rest exists:

```bash
ros2 launch agri_robot agri.launch.py targets:="P1,1R;P1,1L;P2,4R"
```

It drives, measures and photographs, prints how far it parked from each
cross and what the floor camera saw, and throws the reports away.

---

## 5. The parts, and why they are the way they are

### The label — `Pi,jR/L`

`P2,5R` is row 2, plant 5, right-hand side. Rows run along x; standing at the
start of a row and looking down it, your left hand points to +y, so **L is
+y**. The side is fixed to the *world*, not to the robot — a robot driving
back the other way still collects `P2,5R` at the same physical place. One
parser, one formatter, one set of bounds (`agri/labels.py`), because four
programs have to agree on this string and the day two of them disagree it
looks like a network fault.

### The 48 stations — `agri/catalogue.py`

3 rows × 8 plants × 2 sides. Every position is computed from the greenhouse's
own dimensions, and the test suite reads the Phase B world file and checks
the plants really are where the catalogue says. The tightest station has
**0.16 m** to the nearest gutter.

Two stations in an inner aisle are **0.10 m** apart. That single number
drives three other decisions: the cross arms are 0.04 m so the two markers do
not merge into one red blob; the park tolerance is 0.04 m so a visit cannot
be filed under its neighbour; and the floor camera is never allowed to move
the robot more than 0.04 m.

### Which point of the robot goes on the cross

**Not the middle of the base.** A robot with a body cannot see the floor
underneath itself, so a marker under the centre can be driven to but never
verified. The reference point is a **sensor head on a short boom** 0.50 m
ahead of the base, with a camera looking straight down at it. "The cross is
in the middle of the picture" and "the sensor is on the cross" are then the
same statement. Every pose in a report is that point.

### Finding the cross — `agri/vision.py`

The Phase B camera is at 0.78 m pitched 16° **up**, to look over a 0.80 m
gutter at the fruit; the floor only enters its view about four metres ahead.
So `make_robot.py` adds a second, cheap camera looking straight down.

A red mask, connected components, and then two tests that matter: the blob
must fill *under half* its bounding box (a cross does, a stray red object
does not) and it must not touch the frame edge (a clipped cross has a biased
centroid and would stop the robot short). Of the candidates, the one
**nearest the image centre** wins — that is what separates two stations 0.10 m
apart. The marker's *orientation* comes out of the four-fold symmetry of the
shape, which is the reason it is a cross and not a dot.

The correction is capped at 0.04 m. If the detector is ever wrong — a
reflection, a threshold, a sign nobody caught — the worst it can do is
nothing, and it says so in the log.

### Getting around — `agri/aisles.py`

Four free bands in y, a headland at each end past the ends of the 8 m
gutters. A route is at most three legs: out, across, in. No planner: there is
no question to plan, and Phase B's A* is in the repository for anyone who
wants to see what happens when a robot is free to be creative in a 0.16 m
gap.

The two headlands are **not** at symmetric coordinates, because the robot is
not symmetric — the boom always points +x. The first version used one number
for both and put the chassis alongside the middle gutter while the robot
strafed across the greenhouse, straight through it. Nothing would have
reported it (this robot has no collision geometry). It was caught by
`route_clearance()`, which the test suite now runs over all 2 256
station-to-station routes on every run.

### What the robot says about itself

Every report carries the robot's **position and speed** at the instant of the
reading, in the QR code as well as in the JSON:

```
AGRI1|P2,4R|2026-08-05T09:14:45Z|t=22.4|rh=48.2|lux=32001|co2=486|ph=6.26|x=-0.438|y=-0.547|yaw=0.0|vx=0.002|vy=0.001|w=0.000
```

The speed should be near zero — the robot stops, settles, then measures — and
that is exactly why it is worth transmitting. A reading taken at 0.3 m/s is a
reading taken somewhere between two places, and without this field nothing
downstream could ever tell. It is unfiltered, straight from the odometry: a
smoothed number would look tidier and would hide the thing the field exists
to catch.

Between measurements the same two numbers go out on
`agri/v1/status/<node_id>` every five seconds, retained by the broker, so the
Cloud can watch the robot cross the greenhouse rather than hear from it only
on arrival. `cloud> status` and the dashboard's header line both read that.
With multiple nodes the Cloud tracks each one separately in a
`nodes: dict[node_id, status]`.

### The measurement — `agri/measurement.py`, `agri/qrcodec.py`

Five quantities: temperature, humidity, luminosity, CO₂, pH. The QR payload
is deliberately human-readable, 73 characters:

```
AGRI1|P2,5R|2026-08-04T18:22:31Z|t=21.4|rh=63.2|lux=12480|co2=431|ph=6.42
```

Anyone can hold a phone to the dashboard and get the readings back. The Cloud
decodes the QR **image** it received and refuses the report if it disagrees
with the numbers travelling beside it.

### The seal — `agri/crypto_ecc.py`

ECIES: an ephemeral ECDH key agreement on P-256, HKDF-SHA256 (salted with the
ephemeral public key), AES-256-GCM, and an ECDSA signature **over the
ciphertext** so a forgery is rejected before any private-key work happens.
Fresh ephemeral key per message, so recovering the robot's key later does not
open yesterday's traffic.

Requests are **signed but not encrypted**: "measure P2,5R" is not a secret,
but issuing one drives a robot around a greenhouse, so what matters is
authenticity. The robot refuses an unsigned request and one signed by the
wrong key. Reports need both and get both. The test suite checks four
attacks are refused, and `--explain` replays two of them live.

### The Cloud — `agri/cloud/`

One process with two faces: an MQTT client talking to the robot, an HTTP
server talking to the operator. It opens each report and runs three checks
that decrypting does *not* do: the sender named outside the envelope must
match the one inside, the QR must agree with the numbers, and the pose must
actually be nearest the station being claimed. Anything that fails is
**counted and shown**, never dropped quietly — a pipeline that hides its
rejections is one that will one day be receiving nothing and look perfectly
healthy.

Storage is JSONL plus a `photos/` and `qr/` directory, with a CSV export.

### The dashboard

Science-fiction look, one rule underneath it: **every colour means
something**. One dark base, one accent, one alert — so anything coloured is
data. Each quantity gets its own *sequential* ramp, dark and desaturated at
the low end, bright at the high end; a rainbow would be prettier and would
invent boundaries that are not in the numbers. Stations are scaled between
each quantity's plausible bounds, not between the observed minimum and
maximum, so a quiet day does not look like a crisis. An out-of-range station
gets an amber **ring**, never a fill, so its value is still readable on the
same scale as its neighbours. The **light theme is the default** and the dark
one follows the operating system, because a projector in a bright room is
where this gets looked at.

Colour alone is not enough, so **every measured station also carries its
value in figures**, on a chip filled with that same ramp colour — the two
readings come from one number and cannot disagree. A ramp answers "more or
less than its neighbours"; only a figure answers "how much", which is what
the operator is being asked to act on.

The chips are what forced the last piece of layout. The two stations of an
inner aisle (`P1,jL` and `P2,jR`) are **0.10 m apart** — that is real
geometry, not a drawing choice, and it is why the crosses are found visually
rather than driven to on odometry. At map scale their marks touch, so each
chip is pushed 0.30 m *away from its own plant row*, with a hairline back to
its cross: the pair ends up 0.50 m apart and the chip becomes the station's
click target. Where that is still not enough, the map itself can be narrowed
— **row**, **side**, or straight to a **named station** — from the three
selectors above it.

Two buttons under **session** end a run without going back to the terminal,
which is the whole point of having a dashboard on a second machine.
**ENREGISTRER** writes `store/measurements.csv` and downloads a copy through
the browser — the same export the console's `csv` verb and the shutdown path
produce, so a run stopped from the browser and a run stopped with Ctrl-C
leave identical files behind. **QUITTER** exports first and then stops the
Cloud; it is the one control on the page that cannot be undone, so it is the
one button drawn in the alert colour and the only one that asks first. The
robot is deliberately left running: it holds its own keys and its own
mission, and killing it from a web page is not something the Cloud is
entitled to do.

---

## 6. Four honest limitations

**The readings are synthesised.** Gazebo has no humidity probe. Temperature,
humidity, luminosity, CO₂ and pH come from `agri/sensors.py` — a seeded field
with gradients across the greenhouse, a diurnal cycle, per-plant bias and six
deliberate anomalies (a dry patch at P2,4, a CO₂ pocket at P3,7, acid soil at
P1,2). The node says so in its first log line. Everything *around* the number
— when it was taken, where the robot was, what it photographed, how it was
sealed, who verified it, what happens when it does not verify — is real.

**Localisation is Gazebo's ground-truth odometry.** Phase B's homemade SLAM
is in this repository and it diverges: 0.10 m of error growing to 1.70 m over
an 18-minute run, because the across-track correction has no ratchet and the
map is rebuilt at the corrected pose. Running this project on top of that
would mean the robot missing its crosses for a reason that has nothing to do
with what is being demonstrated here. The floor camera is the honest part of
the localisation story: it is the only sensor that actually verifies the
robot is where it says it is, and its residual is reported for every single
visit.

**The store is append-only JSONL.** Queries scan linearly, and the CSV
export rebuilds from scratch each time. For 48 stations measured once or
twice a day this is fine; for a production system with hundreds of nodes
running continuously, a proper database (SQLite at least) would be the
natural evolution. The interface is narrow enough — `file()`, `all_visits()`,
`coverage()` — that the swap is local to `agri/cloud/store.py`.

**`driver.py` has no offline test.** It is the one file that genuinely needs
Gazebo — it talks to ROS topics, reads laser scans and camera images, and
commands wheel velocities. Everything around it (the mission logic, the
measurement, the crypto, the store) is tested offline in `check_cloud.py`;
the driver is tested by running the simulator. Writing a mock odometry
source and a mock camera feed for it is a natural next step, but it was not
prioritised over getting the 235 other checks to pass first.

---

## 7. Before you demonstrate

```bash
python3 tests/check_cloud.py
```

235 checks, about ten seconds, nothing installed beyond the dependencies. It
covers the labels, the geometry against the real world file, the crypto and
its four refusals, all 48 QR codes through real PNG images, the sensor field,
every one of the 2 256 routes, the floor camera against rendered frames, the
generated world, the ROS package read as text (topic names, bridge entries,
spawn point, entry points, the RViz layout, the Gazebo camera watchdog), the
two-machine key handover including the copy nobody remembers, the dashboard's
own geometry, its ENREGISTRER and QUITTER buttons driven over a real socket,
the multi-node protocol (topic namespacing, wildcard matching, `node_kind`,
the per-node status dict, dashboard auth), the whole chain end to end
through a loopback broker, and the offline demo run as a subprocess.

Every check in it is there because something actually went wrong.
