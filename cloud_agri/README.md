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
| `worlds/`, `urdf/` | **generated, and not in git.** `run_sim.sh` builds them; see `agri/world/ensure.py` |
| `run_sim.sh`, `run_cloud.sh` | the two commands that start everything, and stop it |
| `tools/prettylog.py` | turns the launch's 140-line firehose into the eight lines that matter |
| `tests/check_cloud.py` | 691 pre-flight checks, none of which need a broker, ROS or a network |
| `tests/check_live.sh` | the one diagnostic that needs the system **running** |

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

**Two commands, two terminals.**

```bash
# once, from the WORKSPACE root: build the ROS package
colcon build --symlink-install \
    --base-paths cloud_agri/ros2/src --packages-select agri_robot
```

```bash
# terminal 1 -- the greenhouse, the robot, RViz, and a broker
cd cloud_agri && ./run_sim.sh

# terminal 2 -- the Cloud: console here, dashboard on :8088
cd cloud_agri && ./run_cloud.sh
```

That is the whole procedure. `run_sim.sh` activates the virtualenv, sources
ROS 2 and the workspace, **regenerates the world and the robot if they are
missing or out of date — plant meshes included**, starts `mosquitto` if
nothing is already listening on 1883, and launches Gazebo,
`robot_state_publisher`, the gz bridge, `agri_viz`, RViz and the robot node. `run_cloud.sh` waits for the broker and opens the
Cloud. Both refuse to start with an explanation rather than half-starting.

**Everything stops together.** Ctrl-C in either window, or **QUITTER** on the
dashboard, exports both CSVs and then takes the whole simulation down —
Gazebo's forked server and GUI, the camera watchdog, RViz, the bridge, the
robot node, and the broker if `run_sim.sh` started it. Nothing is left behind
to confuse the next run. `./run_cloud.sh --keep-sim` opts out.

**You cannot forget to build the world**, and that is deliberate. `worlds/`
and `urdf/` used to be committed while this README told you to regenerate
them — two things that cannot both be right. A regenerated world is a
locally-modified tracked file, so the next `git pull` aborts; and a pasted
block of commands carries on past an aborted pull, because every line of a
paste runs independently and `&&` protects nothing past a newline. That
combination once left a demonstration four commits behind, showing green
spheres, with nothing on screen saying so. `agri/world/ensure.py` now
decides what to rebuild, and it catches the case Gazebo is silent about: a
mesh URI is an ABSOLUTE path, so a world generated on another machine points
into empty space and renders nothing, with no error in any log.

The two scripts stay two scripts on purpose: the Cloud is meant to be able to
run on another computer, which is the only reason there is a broker between
them. `agri/session.py` explains how they signal each other without either
being the other's parent — and why that signal is a file rather than a PID.

<details>
<summary>The long way, if you want to see each piece</summary>

```bash
# 0. once: generate the world, the robot, and both key pairs
cd cloud_agri
python3 -m agri.world.ensure          # -> worlds/, urdf/, WITH the plant meshes
python3 -m agri.keys                  # -> keys/{cloud,robot}_{private,public}.pem

# ensure is make_world + make_plants + make_robot, and it only rebuilds what
# is missing or out of date. run_sim.sh runs it for you; this line is here
# for the long way. The individual generators still exist:
#     python3 -m agri.world.make_world    # 48 crosses, plants as spheres
#     python3 -m agri.world.make_plants --meshes meshes/strawberry_[1-4].glb
#     python3 -m agri.world.make_robot    # the floor camera

# 1. the broker
mosquitto -p 1883 -v

# 2. the Cloud -- this is the operator's seat
agri-cloud --keys keys --store store            # console here, dashboard on :8088

# 3. the robot, in Gazebo -- from the WORKSPACE root, not from cloud_agri/
cd ..
source ~/.venvs/agri/bin/activate     # WITHOUT THIS the robot node dies on
source install/setup.bash             # `No module named 'qrcode'`
ros2 launch agri_robot agri.launch.py
```

That `source ~/.venvs/agri/bin/activate` is the step everyone forgets. The
robot node then starts under the system Python, dies a second after Gazebo
opens, and the simulation looks perfect while every order goes nowhere.
`run_sim.sh` exists mostly to make that impossible.
</details>

### Reading the log

`run_sim.sh` filters the launch output. Bringing this system up prints about
140 lines and roughly eight of them are about the robot; the rest is Gazebo
listing every service it advertises, Qt complaining about a file dialog nobody
opened, and SDF noting that `gz_frame_id` is not in its schema (it is not; it
is a Gazebo extension and it works). What comes out instead:

```
14:09:49 ▶     started: gazebo, robot_state_publisher, create, parameter_bridge, viz_node, rviz2, robot_node
14:09:50 ok    robot description loaded
14:09:51 ok    gz bridge: 8 topics (clock, cmd_vel, odom, scan, camera, floor_cam, camera_pan_cmd, joint_state)
14:09:52 ok    world 'greenhouse' running
14:09:52 ok    youbot spawned in the greenhouse
14:09:54 ok    agri_robot youbot-01: keys in …/keys, readings are SYNTHESISED
14:09:54 ok    RViz has its fixed frame -- first /odom at x=-4.58 y=-1.85
14:09:56 ·     Gazebo camera is following the robot (confirmed).
```

The **unfiltered** log is written to `cloud_agri/logs/sim-<date>.log` first, so
the filter is never the only copy of what happened. `./run_sim.sh --raw` shows
everything live instead.

The filter is not cosmetic. Three lines decide whether a run will work at all
— the robot could not reach the broker, `agri_viz` never heard `/odom`, a
process died — and each of them used to scroll past inside the noise. Each now
comes out in red with the fix attached.

RViz opens beside Gazebo with the **2D view**: the outline the lidar actually
perceives, drawn on top of where the catalogue says the walls, gutters and 48
crosses are, plus the robot's own footprint including its sensor boom. That
comparison is the point — a scan on its own is a pretty shape with no scale,
and the moment the two stop lying on top of each other, either the odometry
or the catalogue is wrong. `rviz:=false` turns it off.

**If RViz is empty**, the launch window says why. `agri_viz` publishes the
`map -> base_link` transform from `/odom`, and RViz greys out every display
when its fixed frame has no transform — so an empty window has five possible
causes that look identical. The node therefore logs when it starts, logs the
first odometry it receives, and after 12 s of silence says so outright:

```
[viz_node-5] agri_viz: first /odom at x=-4.58 y=-1.85; map -> base_link is
             live and RViz has its fixed frame
```

If that line never appears, run the live diagnostic **in a second terminal,
while the launch is still running**:

```bash
bash cloud_agri/tests/check_live.sh
```

It checks, in order, that the launch is up, that `/odom` arrives, that the
transform resolves, and that there is something to draw — and stops at the
first thing that is not true. Run against a stopped launch it says so
instead of reporting "frame map does not exist", which is true, useless, and
indistinguishable from the real fault.

**If `check_live.sh` is all green and RViz still shows only a grid**, the
fault is the *layout*, not the data — and it was, for two sessions. A plain
`colcon build` copies `agri.rviz` into `share/`, the launch used to hand RViz
that copy unconditionally, and an install made before the layout was fixed
kept serving the broken one for ever. RViz then loads one display out of six
and falls back to its factory Orbit view, which looks exactly like a robot
publishing nothing. The launch now prefers the source tree — the same
preference the world and the URDF always had — and says which copy it used:

```
[view] RViz config: …/cloud_agri/ros2/src/agri_robot/config/agri.rviz
[view] (the installed copy under share/ DIFFERS and is being ignored --
        rebuild to refresh it: colcon build --symlink-install)
```

If you see that second line, rebuild. If RViz *still* disagrees, it has its
own cache: `rm -rf ~/.rviz2/` and relaunch.

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
          issued  2026-08-05T11:41:52Z   still running (37 s so far)
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

**That broker has no TLS and no password**, which is the right amount of
ceremony for a demonstration on one laptop and not for anything else.
Anyone on the network can watch the traffic and publish on the topics. The
payloads are already ECIES-sealed and signed, so an eavesdropper reads
ciphertext and a forged request is refused — but the *metadata* is in
clear, and for a commercial greenhouse the pattern of who published what
and when is a map of the operation.

Both programs can close that, and the step is configuration rather than a
rewrite:

```bash
agri-cloud --broker gh.example \
           --broker-ca ca.crt --broker-cert cloud.crt --broker-key cloud.key
```

Naming a CA turns TLS on and moves the port from 1883 to 8883 by itself.
**Neither side ever falls back to plaintext**: a misconfiguration is fatal,
because a system that silently downgrades is worse than one with no TLS at
all. If you use `--broker-user`, the password comes from
`$AGRI_BROKER_PASSWORD` and there is no flag for it —
`/proc/<pid>/cmdline` is world-readable. A username with no TLS is refused
outright, since MQTT sends the password in the CONNECT packet in clear.

`deploy/` carries a broker configuration and an ACL that do this properly,
with the `openssl` commands to produce the certificates.

The dashboard binds every interface, so `http://192.168.1.20:8088` works
from the robot's machine or a phone on the same network. `agri-cloud`
prints that address at startup rather than `localhost`.

**Dashboard authentication.** By default `agri-cloud` generates a random
token and prints the full URL including it. **Open that URL once.** The
server hands the browser an `HttpOnly; SameSite=Strict` cookie and
redirects to the same page with the token stripped out, so from then on the
credential is not in the address bar, not in the browser history, not in
the `Referer` of any outbound link, and not in the logs of any proxy in
between. Every page also carries `Referrer-Policy: no-referrer`.

The token is checked on every API request and on `/media/`, because a
photograph and a CSV row are measurements just as much as the JSON that
describes them, and a token that guards one but not the other guards
nothing. It is read from the cookie, from `Authorization: Bearer …`, or
from `?token=…` — in that order, so `curl` and a cookie-less browser still
work. Ten wrong tokens from one address inside a minute earn a 60-second
`429`: the token is 16 random bytes, so this is not about guessing it in
ten tries, it is about not leaving a machine where a script can try a
million overnight.

To choose your own, pass `--dashboard-token SECRET`. Authentication can
still be turned off with `--dashboard-token ''`, but **only together with
`--http-host 127.0.0.1`** — otherwise the Cloud refuses to start and prints
why. That flag used to emit a warning and serve anyway, and a warning at
startup is read once and scrolls away. The offline demo
(`agri.demo --serve`) runs without a token.

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

### The order of the stations, and the battery — `agri/survey.py`

The Cloud lists stations in survey order — `P1,1R`, `P1,1L`, `P1,2R`, … —
which is right for a human reading a list and wrong for a robot driving a
building. The two sides of one plant are separated by a gutter 8 m long, so
going from `R` to `L` means driving to the end of the row, crossing, and
coming back. Over 48 stations the survey order asks for that 45 times.

Measured against the real geometry, from the dock:

| order | distance | gutter crossings |
|---|---|---|
| survey, as the Cloud issues it | 312 m | 45 |
| swept band by band | 40 m | 3 |

**272 m, 87 %.** At 0.45 m/s that is about ten minutes of a battery spent
arriving at places the robot had already been standing next to. The first
recorded full campaign took 1 505 s; most of it was this.

The fix is not a planner. `aisles.py` already divides the free space into
four bands, and every station is in exactly one:

    band 0   P1,*R                      8 stations
    band 1   P1,*L and P2,*R           16 stations
    band 2   P2,*L and P3,*R           16 stations
    band 3   P3,*L                      8 stations

Within a band the robot drives freely; leaving one costs a headland trip
whatever the destination. So the cheap route is to sweep a band end to end,
cross once, and sweep the next one back. The only decisions left are which
end to start at and which way to go — four combinations, all four measured
against the same `route()` the driver will actually fly, shortest wins.

**The order as issued is the fifth candidate, and it wins ties.** That makes
"this can never lengthen a mission" a property of the code rather than a
claim about it, and the suite checks it over 720 random subsets. A two-station
request is returned untouched.

The objection the old code recorded — that a robot which silently reorders
makes the ack stream stop matching the request — was real, so it is answered
rather than accepted: the mission keeps `issued` alongside `targets`, every
ack still names its own station, and nothing an operator reads was ever
indexed by position.

#### What is measured and what is modelled

This matters for the report, so it is worth being exact about.

**Measured.** The distance driven, integrated from odometry in the driver
(`distance_m()`), summed from successive positions rather than from the
reported speed. It inherits the odometry's drift, which is the honest thing
for it to do — it is what the robot believes it drove. A single step longer
than `ODOM_JUMP_M` is a respawn, not a drive, and is not counted.

**Modelled.** The energy. Gazebo models no battery and this robot has no
current sensor, so a percentage ticking down would be a fabricated number
sitting beside forty-eight real ones. Instead `agri/survey.py` states four
constants in the open — `IDLE_W`, `DRIVE_W`, `CRUISE_MPS`, `BATTERY_WH` —
labelled `ASSUMPTIONS, NOT MEASUREMENTS`, for someone to replace with a bench
reading. The suite asserts `CRUISE_MPS` equals the driver's `V_MAX`, so the
estimate cannot quietly drift away from the robot it describes.

The shape of the model is the part worth trusting: idle power is drawn
whether or not the robot moves, drive power only while it moves.

    energy = IDLE_W x mission_seconds  +  DRIVE_W x driving_seconds

**And that shape carries the real finding**, which is more useful in a report
than the 87 %: on a 48-station sweep the robot spends far longer standing at
crosses — settling, focusing, photographing, encoding a QR — than driving
between them. Shortening the route removes drive energy and leaves idle
energy untouched, so it attacks the smaller half of the bill. Cutting the
mission further means attacking the per-station time, not the path.

`requests.csv` keeps the two apart by name: `driven_m` and `planned_m` beside
`energy_wh_est`, `idle_wh_est`, `drive_wh_est`. A request whose robot did not
report a distance leaves those cells **empty**, not zero — "this robot does
not count" and "it drove nothing" are different facts.

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

### Security posture — what is enforced, and what is not

Encryption answers a narrow question: can a third party read this, and did
the named sender write it. A deployed system fails in ways that leave both
answers *yes*. What the code now enforces beyond the seal:

| control | where | what it stops |
|---|---|---|
| freshness window + seen-set on requests | `agri/replay.py` | a captured signed order being published again next week and obeyed |
| queries and replies signed both ways | `agri/protocol.py` | a forged `hold` answered to every offer: the outbox fills, the oldest readings drop, and the robot reports itself healthy the whole time |
| MQTT over TLS + client certificates + ACL | `deploy/` | a stranger on the network reading the traffic pattern, or publishing orders |
| token in an `HttpOnly` cookie, not the URL | `agri/cloud/server.py` | the dashboard credential living in the history, the `Referer` and every proxy log |
| refusing to run unauthenticated off loopback | `agri/cloud/server.py` | `--dashboard-token ''` quietly leaving the greenhouse open |
| hash-chained store + `--verify` | `agri/cloud/store.py` | a filed reading edited on disk, undetected |
| fingerprints, revocation, `--rotate` | `agri/trust.py` | a stolen or decommissioned key staying valid forever |

The rule the protocol follows, so the message table can be read without the
code: **everything that can change what the other side does is signed;
everything that only reports is not.** Requests, queries and replies decide
something. Acks and status are observations.

Deliberately still open, each with what closing it would cost: the
revocation list is not distributed (needs a real PKI); key age is an
unsigned reminder, not a validity period (same); the hash chain sits on the
same disk as the data, so it detects a surgical edit but not a full rewrite
(needs the tip pinned off-box — `--verify` prints it); the dashboard is
plain HTTP (needs a TLS proxy, no code change); one shared token with no
per-operator identity; and dependencies are unpinned. `docs/report`,
section "Security Posture", states all of these with the threat model they
are judged against.

Check any of it:

```bash
python3 -m agri.keys --list                 # fingerprints, ages, revocations
python3 -m agri.cloud.store --verify        # the chain, and the tip to record
```

### The Cloud — `agri/cloud/`

One process with two faces: an MQTT client talking to the robot, an HTTP
server talking to the operator. It opens each report and runs three checks
that decrypting does *not* do: the sender named outside the envelope must
match the one inside, the QR must agree with the numbers, and the pose must
actually be nearest the station being claimed. Anything that fails is
**counted and shown**, never dropped quietly — a pipeline that hides its
rejections is one that will one day be receiving nothing and look perfectly
healthy.

Storage is JSONL plus a `photos/` and `qr/` directory, with two CSV exports.

**Three clocks, all recorded.** A reading has three interesting moments, and
the store used to keep one of them — so "how long did that sweep take" could
only be answered by watching a clock while it ran:

| | whose clock | what it means |
|---|---|---|
| `request_issued_at` | the Cloud | it signed the order. This is the stamp that gets **signed and sent**, so the Cloud's note of when it asked and the robot's copy of the order are the same number by construction. |
| `measured_at` | the robot | it read the station |
| `received_at` | the Cloud | it opened, verified and filed the report |

The first and last share a clock, so `latency_s` — order to evidence — is a
real duration and not an artefact of two machines disagreeing about the time.
All four columns are in `store/measurements.csv`; the column formerly called
`timestamp` is now `measured_at`, because a column called "timestamp" sitting
next to two other times is a column whose meaning has to be looked up in the
source.

`store/requests.csv` answers the other question — **orders**, not readings —
with `issued_at`, `completed_at`, `elapsed_s`, `state`, `done/total` and the
targets. One request covers up to 48 stations, so "when did the sweep finish"
is not answerable by taking a maximum over 48 measurement rows: a sweep that
was *abandoned* has no such row at all. A request that fails halfway is
closed with the time it gave up, rather than left looking as though it is
still running.

The dashboard shows the same three moments in the reader's own time zone —
the stamps travel as UTC because the greenhouse and the Cloud need not be in
the same one.

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
**ENREGISTRER** writes `store/measurements.csv` and `store/requests.csv` and
downloads both through the browser — the same export the console's `csv` verb
and the shutdown path produce, so a run stopped from the browser and a run
stopped with Ctrl-C leave identical files behind. **QUITTER** exports first,
then stops the Cloud **and the simulation with it**: Gazebo, RViz, the bridge
and the robot node all go down, because the operator who pressed it meant
"stop", not "stop half of it", and a laptop that collects orphaned Gazebo
servers over an afternoon of demonstrations behaves inexplicably by evening.
It is the one control on the page that cannot be undone, so it is the one
button drawn in the alert colour and the only one that asks first.

It reaches the simulation through a **file**, not a signal — the two are
separate processes on purpose, and a PID can be reused between being written
down and being killed. It also fails in the right direction: with the Cloud
on a second machine the file lands on *that* machine's disk, nobody reads it,
and the robot keeps running. A Cloud on another computer has no business
killing a robot. See `agri/session.py`.

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
prioritised over getting the other 690 checks to pass first.

---

## 7. Before you demonstrate

```bash
python3 tests/check_cloud.py
```

383 checks, about fifteen seconds, nothing installed beyond the dependencies
— 492 on a machine with no ROS 2, where the numpy-against-rclpy clash is the
one thing that cannot be tested because there is no rclpy to clash with. It
covers the labels, the geometry against the real world file, the crypto and
its four refusals, all 48 QR codes through real PNG images, the sensor field,
every one of the 2 256 routes, the floor camera against rendered frames, the
generated world, the ROS package read as text (topic names, bridge entries,
spawn point, entry points, the RViz layout **and which copy of it gets
loaded**, the Gazebo camera watchdog, the viz node's own startup and
no-odometry logging, the robot node's broker retry), the two-machine key
handover including the copy nobody remembers, the dashboard's own geometry,
its ENREGISTRER and QUITTER buttons driven over a real socket, the multi-node
protocol (topic namespacing, wildcard matching, `node_kind`, the per-node
status dict, dashboard auth), the three clocks from signing to CSV including
a request that is abandoned halfway, the two launch scripts (valid bash, the
virtualenv, the broker ordering, the cleanup, and the stop-file path spelled
identically in shell and in Python), the log filter **run against real launch
output**, the whole chain end to end through a loopback broker, and the
offline demo run as a subprocess.

Every check in it is there because something actually went wrong.

The one thing it cannot tell you is whether the *running* system is healthy.
That is `tests/check_live.sh`, in a second terminal, while `./run_sim.sh` is
up.
