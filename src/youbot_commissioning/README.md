# `youbot_commissioning` — taking the stack from simulation to a real greenhouse

This package is the field procedure. It is ten numbered stages, each a runnable
ROS 2 node with a written procedure, a numeric pass/fail criterion, and an
archived report. It is deliberately separate from `youbot_control`: the
mission stack is what gets tested, this is what does the testing, and mixing
the two is how a test ends up passing because it shares a bug with the thing
it is testing.

**The rule: stages run in order, and a stage that has not passed blocks every
stage after it.** Not as bureaucracy — each stage removes exactly one class of
unknown, and running stage 6 before stage 0 means driving a robot whose
stopping distance nobody has measured.

---

## Why staged bring-up at all

The simulation is a kinematic model on a perfect floor with opaque walls and
uniformly lit fruit. Every one of those four is false in a greenhouse. Turning
the whole stack on at once produces a robot that misbehaves for four reasons
simultaneously, and no way to attribute any of it.

This is the same method that worked in Phase A, where `wheel_test`,
`base_test` and `odom_test` each isolated one subsystem. It is that method
applied to hardware, with the criteria written down in advance so they cannot
be adjusted afterwards to match what happened.

---

## The ten stages

| # | Stage | Removes the unknown | Passes when |
|---|---|---|---|
| 0 | `stage0_estop` | Can this machine be stopped? | Wheels stop within the measured time, every burst; timeout stop works |
| 1 | `stage1_wheels` | Do the wheels do what they are told? | Three canonical twists, within direction and coupling tolerance |
| 2 | `stage2_umbmark` | How badly does odometry drift **on this floor**? | `E_max` under budget after correction; random spread bounded |
| 3 | `stage3_lidar` | **Does the lidar see the glass?** | Return rate and range error acceptable at every station |
| 4 | `stage4_manual_map` | Can the greenhouse be mapped at all? | Map dimensions match the tape; coverage complete; no aisle clutter |
| 5 | `stage5_localisation` | Does the pose estimate stay bounded? | Worst error under budget **and the drift slope is flat** |
| 6 | `stage6_navigation` | Can it drive itself without hitting anything? | 20 round trips, zero contacts, zero interventions, clearance respected |
| 7 | `stage7_dataset` | Do we have real images? | Enough frames over enough distinct poses, conditions recorded |
| 8 | `stage8_detector` | Is the learned detector actually better? | Precision/recall above target **and beats the colour baseline** |
| 9 | `stage9_pick` | Can it pick a real berry? | Success rate above target, zero damage events |

Stages 3 and 5 are the two most likely to fail, and they are the two whose
failure changes the plan rather than the tuning:

- **Stage 3 failing means the greenhouse must be modified**, not the software.
  Glass at a glancing angle reflects the beam away and the wall becomes
  invisible. The fix is an opaque band at lidar height (0.20 m) along every
  pane the robot can approach. Tell the site early; it is their work, not yours.
- **Stage 5 failing means the localisation design is inadequate**, and the
  answer is an IMU and an absolute correction (AMCL against the stage 4 map,
  or surveyed AprilTags), not a better tuned odometry.

---

## Operator interface

Every stage shares the same three controls, so there is one thing to remember
under pressure.

```bash
# ARM  — nothing moves until this is published
ros2 topic pub -1 /commissioning/arm std_msgs/Bool "{data: true}"

# DISARM / ABORT — after the hardware E-stop, never instead of it
ros2 topic pub -1 /commissioning/arm std_msgs/Bool "{data: false}"

# FIELD NOTE — goes into the archived report, typed while you remember it
ros2 topic pub -1 /commissioning/note std_msgs/String \
  "{data: 'north row in full sun, condensation on the west pane'}"
```

Stage-specific topics (`/commissioning/measurement`, `/commissioning/expect`,
`/commissioning/truth`, `/commissioning/event`, `/commissioning/attempt`,
`/commissioning/session`, `/commissioning/conditions`) are documented in each
stage's `PROCEDURE`, which is printed to the console when the node starts.
Read it there; it is written for someone standing next to the robot.

### Safety, stated plainly

The arm topic and the speed clamp in `lib/stage.py` are **functional** safety:
they make the tests well-behaved. They are Python in a non-real-time process
and **they are not a protective device**. What protects people is the hardware
E-stop of stage 0 — a mushroom button that cuts motor power without the
computer's participation — plus a speed bridle and, ideally, a certified safety
scanner independent of ROS entirely (ISO 3691-4). Nothing in this package
substitutes for that.

---

## Running a stage

```bash
source install/setup.bash
CONF=$(ros2 pkg prefix youbot_commissioning)/share/youbot_commissioning/config/limits.yaml

# In simulation, to rehearse the procedure before touching hardware:
ros2 run youbot_commissioning stage1_wheels --ros-args -p profile:=sim

# On the robot, with the hardware profile:
ros2 run youbot_commissioning stage1_wheels --ros-args \
  --params-file "$CONF" -p profile:=hardware

# Stage 8 is offline and takes plain arguments, no ROS graph needed:
ros2 run youbot_commissioning stage8_detector -- \
  --labels   ~/dataset/labels \
  --predict  ~/runs/yolov8n/labels \
  --baseline ~/runs/colour_threshold/labels
```

**Rehearse every stage in simulation first.** Not to validate the robot — to
validate the procedure, so that the first time you read a stage's instructions
is not while standing in a greenhouse with a running machine.

---

## Reports

Every stage writes a timestamped JSON and Markdown report:

```
~/youbot_commissioning_results/stage2_umbmark_20260412_101533.json
~/youbot_commissioning_results/stage2_umbmark_20260412_101533.md
```

Override the location for a field laptop or a USB stick:

```bash
export YOUBOT_COMMISSIONING_RESULTS=/media/usb/serre_2026_04
```

The JSON is not decoration. Stage 2 measures the real odometry drift, stage 5
compares against it, and the written report quotes both. Retyping numbers off a
terminal is how measurements get corrupted, so nothing here is only printed.

A stage interrupted with Ctrl-C **still writes its report**. "Inconclusive,
stopped at lap 3" is information; a silent exit is not.

---

## The one file that holds every physical number

`config/limits.yaml`, with two profiles.

The `hardware` protective distance is `0.35 m` against the simulation's
`0.12 m`. That factor of three is not caution, it is arithmetic — ISO 13855
with the latency and inertia a kinematic Gazebo model does not have:

```
S = K · (t_react + t_stop) + C
  = 0.30 · (0.20 + 0.30) + 0.20
  = 0.35 m
```

**Replace `t_stop` with what stage 0 actually measures on your robot.** The
value shipped here is a placeholder with a defensible derivation, not a
measurement.

This file exists because the project already lost a full run to parameter
drift: `youbot_params.yaml` kept the pre-bumper clearances after the code had
moved to bumper-relative ones, the YAML won at runtime, every threshold
silently doubled, and the robot spent 58 % of that run braked for obstacles
half a metre away. The lesson was not "be careful". It was "one place, checked
automatically".

---

## What this package deliberately does **not** do

- It does not drive the base in stages 6, 7 and 9 — the mission stack does,
  and these stages only observe. A test that also controls cannot tell you
  whether the controller works.
- It does not decide whether a detection is correct (stage 8 needs human
  annotations) or whether a pick succeeded (stage 9 needs a human to look).
  Anywhere a judgement is needed, a person makes it and the node records it.
- It does not certify anything for safety. It measures, it records, and it
  refuses to continue when a criterion fails.
