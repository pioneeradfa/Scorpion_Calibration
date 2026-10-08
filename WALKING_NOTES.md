# Six-Leg Walking — Bring-Up Guide

Focus: getting the six legs walking smoothly on the Pi 5 + LSC-24 + HPS-2027.
The tail (channels 19–22) and claws (23–24) are allocated in the channel map but
nothing here drives them.

---

## 1. Run these four first — no hardware needed

```bash
python3 walk.py --self-test      # IK math, must say SELF-TEST PASS
python3 walk.py --check          # margins and torque for the standing pose
python3 walk.py --trajectory     # the foot path, segment by segment
python3 walk.py --dry-run --cycles 2
```

`--dry-run` computes a whole walk and sends nothing. If `ServoControl.py` cannot
be imported, every command falls back to dry run automatically and says so, so
you cannot accidentally move hardware from a laptop.

## 2. Before power-on

| check | setting | why |
|---|---|---|
| per-channel over-current jumpers | **K (1.5 A)** on all 24 | F/H/C trip below the servo's loaded current |
| low-voltage alarm jumper | **removed → 6.4 V** | at 5 V a 2S pack is already damaged when it beeps |
| servo supply | 7.4 V LiPo into the LSC-24 | HPS-2027 working range is 6–8.4 V |
| Pi 5 supply | **separate**, 5 V / 5 A PD | the LSC-24 has no 5 V BEC and cannot power a Pi 5 |
| grounds | **common** between Pi and LSC-24 | otherwise the serial line floats |
| serial port | `ls -l /dev/ttyAMA* /dev/ttyS*` | Hiwonder's two variants disagree — one opens `/dev/ttyS0`, the other `/dev/ttyAMA0`; Pi 5 assigns these differently from Pi 4 |
| serial console | disabled in `raspi-config` → Interface Options → Serial Port (no login shell, yes hardware serial) | otherwise the console owns the UART |
| **TTL levels** | meter the LSC-24 TX line | Pi 5 GPIO are 3.3 V. If TX idles at 5 V, put a divider or level shifter on the Pi's RX before connecting |

Baud is fixed at **9600** in `ServoControl.py`. Do not raise it — the board's
firmware expects 9600.

## 3. Map the channels

The old wiring cannot be reused: it addressed 24 servos on channels 1–31, and
channels 24, 25, 28, 29, 30, 31 do not exist on a 24-channel board. The new map
is contiguous and **1-based**:

```
RF coxa=1  femur=2  tibia=3        LF coxa=10 femur=11 tibia=12
RM coxa=4  femur=5  tibia=6        LM coxa=13 femur=14 tibia=15
RR coxa=7  femur=8  tibia=9        LR coxa=16 femur=17 tibia=18
                                   tail 19-22   claws 23-24
```

Then:

```bash
python3 walk.py --identify
```

It sweeps each channel through ±20° around 135° one at a time and waits for you
to press Enter. Disconnect all servos but the one under test, or prop the robot
so it cannot walk off the bench. Edit `LEG_CHANNELS` in `servo_lsc24.py` to
match whatever you actually find.

**Never use channel 0.** Hiwonder's driver remaps `id < 1` to 254, which is the
broadcast address — it would drive all 24 servos at once. `ServoBus` raises
`ServoLimitError` if you try.

## 4. Verify the joint signs on one leg

This is the step that has never been done. Prop the robot up so RF swings free.

```python
import ik_module as ik
from servo_lsc24 import ServoBus
bus = ServoBus()

ik.set_foot_position_body("RF", 115.0, -204.5, -155.0, duration=2000, bus=bus)  # planted
ik.set_foot_position_body("RF", 115.0, -204.5, -130.0, duration=2000, bus=bus)  # lifted 25mm
```

Expected angles and what each one should do:

| joint | ch | planted | lifted | Δ | must |
|---|---|---|---|---|---|
| coxa | 1 | 147.00 | 147.00 | +0.00 | not move — the foot went straight up |
| femur | 2 | 108.12 | 131.70 | **+23.59** | raise the leg. If it drops → flip `FEMUR_SIGN` |
| tibia | 3 | 84.15 | 58.11 | **−26.03** | curl the knee, i.e. the raw angle must FALL. If it rises → flip `TIBIA_SIGN` |

The tibia row is the one the hand-calibrated stand pose settled: the calibration
pose is the leg dead straight, stand sits 88.5° below the calibrated tibia, and
a hinge only bends one way — so bending the knee must decrease the raw angle.
See `RF_LEG_IK.md` §2.

Then the coxa direction, foot swept 25 mm forward:

| leg | ch | before | after | Δ | meaning |
|---|---|---|---|---|---|
| RF (right) | 1 | 147.00 | 156.82 | **+9.82** | right leg forward = raw increases |
| LF (left) | 10 | 172.50 | 162.68 | **−9.82** | left leg forward = raw decreases |

Also look at the knee: it should end up **above** the coxa plane with the tibia
reaching down. If the knee ends up below, the elbow branch is wrong — change
`femur_angle = alpha + beta` to `alpha - beta` in `ik_module.leg_ik()`.

Sign constants are at the top of `ik_module.py`. Flip one, re-run
`python3 walk.py --self-test`, and repeat this step.

## 5. First motion

Single leg first, exactly as `RF_LEG_IK.md` describes — prop the robot so only
RF is free:

```bash
python3 walk.py --user-stand --force-dry   # feel the pace, send nothing
python3 walk.py --rf-cycle --force-dry     # RF swing/stance cycle, nothing sent
python3 walk.py --user-stand               # rise to your hand-calibrated pose
python3 walk.py --rf-cycle 2               # two smooth RF cycles, legs 2-6 hold
```

Then, once RF has been watched through a cycle and nothing grinds:

```bash
python3 walk.py --belly      # fold out to the calibration pose, 2.5s
python3 walk.py --stand      # rise to the gait stance, 3s
python3 walk.py --walk 1     # one gait cycle = 50mm forward, then sits down
python3 walk.py --demo       # belly -> rise -> walk -> sit, one command
```

Every one of these asks for confirmation before moving 18 servos; `--yes` skips
the prompt. `--walk` refuses to run if `check_cycle()` says the geometry is
infeasible, so it cannot start a stride it cannot finish.

## 6. The default standing geometry

```
reach_out    80 mm    femur pivot -> planted foot
body_height 155 mm    coxa axis above ground
stride       50 mm    body advance per gait cycle
lift         25 mm    peak foot clearance
cycle      2000 ms    in 16 segments of 125 ms
```

Foot 144.5 mm outboard of each coxa axis → **409 mm stance width** on a 230 mm
body. Knee works between 78% and 90% of full stretch. Tightest servo margin
anywhere in the cycle is **33.6° on LF**. Peak femur torque **11.5 kg·cm = 58%**
of the 20 kg·cm stall figure at an assumed 3.0 kg and ×1.4 dynamic.

These came from a constrained scan that maximised the tightest margin across all
six legs and the whole stride, subject to torque staying under 60% of stall.

### Retune it for your real robot

**Weigh it first.** Nothing else matters as much. The estimate is 2.5–3.6 kg, of
which the 24 servos are 1.58 kg — 44–62% of the whole robot. Then:

```bash
python3 walk.py --check --mass 3.4
python3 walk.py --check --mass 3.4 --reach 70      # shorten the moment arm
python3 walk.py --check --mass 3.4 --reach 70 --height 145
```

Knobs and what they do:

| knob | effect | cost |
|---|---|---|
| `--reach` ↓ | less femur torque (torque = leg load × reach, independent of height) | more knee bend → less tibia margin, narrower stance |
| `--height` ↑ | straighter leg → more tibia margin | higher centre of gravity, closer to the full-stretch singularity |
| `--stride` ↓ | smaller coxa sweep | less distance per cycle |
| `--lift` ↓ | less tibia excursion at peak swing | poorer ground clearance |

LF is always the binding leg — its calibrated tibia (151.5°) sits 25.5° further
round than RM/LM's (126.0°), giving it the highest reach floor. If you want more
headroom, re-seat LF's tibia horn one tooth (14.4° on a 25T spline) and
re-calibrate that leg; it would lift the tightest margin from 33.6° to about 48°.

Torque reference at 3.0 kg, tripod, ×1.4 dynamic:

| reach_out | peak femur torque | % of stall |
|---|---|---|
| 60 mm | 8.0 kg·cm | 40% |
| 80 mm | 11.5 kg·cm | 58% |
| 100 mm | 14.0 kg·cm | 70% |
| 120 mm | 16.0 kg·cm | 80% |

## 7. Why it is smooth

**One array frame per segment.** `setPWMServoMoveByArray` puts all 18 leg servos
in a single 61-byte frame (63.5 ms at 9600 baud) instead of 18 separate frames
(169 ms). The board's STM32 interpolates over the duration you give it.

**Pipelined, not slept.** A frame takes 63.5 ms to arrive and the board only
starts moving once it has the whole thing. Naively sending a command, sleeping
for its duration, then sending the next leaves the board idle for 63.5 ms between
every segment and the motion staircases. `ServoBus.send_pose()` instead sleeps
for exactly the commanded duration, so the next frame finishes arriving at the
precise moment the previous segment ends. Segments chain with no gap.

That imposes a floor: a segment cannot be shorter than its own frame.
`validate_timing()` enforces it — 31 sub-phases is the most a 2000 ms cycle
allows.

**Trajectory shape.** Stance sweeps the foot backward at a constant rate, so the
body advances at constant velocity. Swing uses a smoothstep fore/aft and
`sin(πt)` for lift, both with zero velocity at touchdown and lift-off, so the
foot does not jerk at the transitions and does not drag — the old code returned
the coxa to base *while* lifting, sweeping the toe along the ground.

**Body actually translates.** The old `caterpillar_walk` and `tripod_walk` never
updated `base`, so each cycle returned the feet to the same absolute position and
the robot walked in place. They also applied `coxa += forward` to both sides,
which pushes right legs forward and left legs backward and twists the body. Here
the targets are body-frame positions solved through the IK, so the mirroring is
the solver's job.

## 8. Serial budget

| operation | bytes | time | max pipelined rate |
|---|---|---|---|
| single servo | 9 | 9.4 ms | — |
| 9-servo array (one tripod) | 34 | 35.4 ms | 28.2 Hz |
| 18-servo array (all legs) | 61 | 63.5 ms | 15.7 Hz |
| 24-servo array | 79 | 82.3 ms | 12.2 Hz |

A 2-cycle walk at the defaults costs 2033 ms of wire time inside 4000 ms of
motion — about 51% utilisation. There is headroom, but not enough for
Python-side interpolation at any useful rate. Never port `move_smooth()` from the
legacy file: stepping 18 servos 20 times each is 378 frames, 3.5 seconds.

The board can also store 230 action groups of up to 510 actions each and play
them back with `setGroupRun(group_id, count)`. That would remove the serial
bottleneck entirely for a fixed gait — but groups are authored in Hiwonder's PC
software, and the Python library only exposes run/stop/speed, not save. Worth
revisiting once the gait is tuned and you want it faster.

`LOBOT_CMD_GET_BATTERY_VOLTAGE = 15` is defined in `ServoControl.py` but has no
read implementation in any version of the library. Writing one would allow a
software low-battery guard that sits the robot down, complementing the 6.4 V
buzzer.

## 9. Known limits

- **The six legs do not share a workspace.** LF's reach floor is 117.0 mm;
  RM/LM's is 89.1 mm. `check_cycle()` validates all six over the whole stride
  before `walk()` will run.
- **The leg is always 46–100% extended.** There is less vertical compliance than
  on a hexapod with more evenly matched segments, so rough ground will show up
  as body pitch. A 25 mm lift is comfortable; much more starts binding LF.
- **Mass is assumed, not measured.** The torque figures are only as good as the
  `--mass` you pass.
- **Nothing has touched a servo yet.** §4 is the gate.
