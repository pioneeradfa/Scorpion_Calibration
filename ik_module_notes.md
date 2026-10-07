# Scorpion Hexapod — Inverse Kinematics

**Updated 2026-10-07 for the Raspberry Pi 5 / Hiwonder LSC-24 / HPS-2027 rebuild.**

This file replaces the previous version, which contained several claims that have
since been shown to be wrong. Corrections are called out explicitly in
§5 so the reasoning trail is not lost.

---

## 1. Confirmed geometry (mm)

- Coxa link: **64.5**
- Femur link: **64.5**
- Tibia link: **130.59**
- Right/left coxa-axis spacing: **120** (±60 from the centreline)
- Front/mid/rear coxa-axis spacing: **115** each → rows at x = +115 / 0 / −115
- Mount angle: all six coxa axes point straight out (90°), no radial splay
- Confirmed against the CAD drawing: row spacing 115.28 / 115.04 / 115.05 / 114.97,
  width spacing 119.96 / 120.12 / 120.03

Derived:

| quantity | value |
|---|---|
| planar max (femur pivot → foot, leg straight) | 195.09 mm |
| planar min (fully folded) | 66.09 mm |
| full reach (coxa axis → foot, leg straight and level) | 259.59 mm |

## 2. Hardware

| item | value | consequence |
|---|---|---|
| Controller | Hiwonder **LSC-24**, STM32, LOBOT protocol | TTL serial **9600 baud**, onboard interpolation |
| Servos | Hiwonder **HPS-2027**, 20 kg·cm, **270°** | **500–2500 µs = 0–270°**, not 0–180° |
| Servo horn | **25T** | one spline tooth = 14.4° |
| Supply | 2S 7.4 V 2200 mAh **50C** LiPo | 110 A available — the pack is never the limit |
| Board input ceiling | **10 A** at 7.4 V | the real limit; idle alone is 2.4 A (24 × 100 mA) |
| Per-channel protection | jumper **K=1.5 A / F=1 A / H=500 mA / C=200 mA** | **must be K** — the servo stalls at 2.4–3 A |
| Low-voltage alarm | jumper 5 V or 6.4 V | **set 6.4 V** (jumper removed) for a 2S pack |
| Pi 5 | powered **separately** | correct — the LSC-24 has no 5 V BEC; keep grounds common |
| Servo speed | 0.20 s / 60° at 7.4 V | never the bottleneck; serial is |

## 3. Joint conventions

- **Femur**: increasing raw angle lifts the leg up. `FEMUR_SIGN = +1`
- **Tibia**: increasing raw angle curls the leg up. `TIBIA_SIGN = +1` ← **was −1, see §5.1**
- **Coxa**: increasing raw angle rotates CCW on every servo, which is *forward* on
  right legs and *backward* on left legs. `COXA_SIGN = {R: +1, L: −1}`

**Elbow branch**: the solver uses `femur_angle = alpha + beta` (knee *above* the
coxa plane, tibia reaching down). Verified by elimination — the alternative
branch drives the femur raw angle negative (−7° to −57°) for every plausible
standing target, which no servo can represent.

## 4. Units

**Everything above `servo_lsc24.py` is in TRUE PHYSICAL SHAFT DEGREES (0–270).**
Pulse widths never leave that module.

`leg_calibration.json` still holds the numbers recorded through the legacy /180
mapping, so they are multiplied by `LEGACY_CALIBRATION_SCALE = 1.5` on load:

| leg | coxa | femur | tibia |
|---|---|---|---|
| RF | 98 → 147.0 | 88 → 132.0 | 94 → 141.0 |
| RM | 90 → 135.0 | 92 → 138.0 | 84 → 126.0 |
| RR | 103 → 154.5 | 87 → 130.5 | 90 → 135.0 |
| LR | 87 → 130.5 | 90 → 135.0 | 91 → 136.5 |
| LM | 90 → 135.0 | 90 → 135.0 | 84 → 126.0 |
| LF | 115 → **172.5** | 93 → 139.5 | 101 → **151.5** |

Once you re-calibrate with the corrected mapping in place, set the scale to 1.0.

## 5. Corrections to the previous version of this file

### 5.1 `TIBIA_SIGN` was −1 and its comment described the opposite

The comment read *"knee bending (knee_angle below 180) → raw increases (up/curl)"*
while the code computed `cal − phi`, which **decreases**. Now `+1`, matching both
the comment and the hardware-observed convention.

### 5.2 The old self-test was a tautology

It evaluated the solver at full leg extension — the **singular** point, where
`beta = 0.008°` and the knee bend `phi = 0.012°`. Both the elbow branch and
`TIBIA_SIGN` were multiplied by approximately zero, so the test could not fail
and could not detect either error. It also never called `body_to_leg_local()`,
leaving `MOUNT_POS` and the `COXA_SIGN` mirroring with zero coverage.

Its lift assertion was additionally backwards: lifting a foot at constant
horizontal reach near full stretch **straightens** the knee (φ 92.44° → 91.07°,
Δ = −1.36°), but the test asserted an increase. With `TIBIA_SIGN = −1` the real
−1.36° became a fake +1.36° and the test passed. Two bugs cancelling into a
green checkmark. The new self-test keeps this exact case as a regression guard.

### 5.3 "tibia ≈ 2× femur forces a narrow envelope" — wrong mechanism

The usable envelope *was* narrow, but not because of the link ratio. It was the
`ANGLE_MIN/MAX = 0/180` clamp, which threw away a third of a 270° servo's
travel. With the limits corrected:

| leg | φ_max | feasible d | extension |
|---|---|---|---|
| RM / LM | 144° | 89.1 – 195.1 mm | 46–100% |
| RR | 135° | 98.6 – 195.1 mm | 51–100% |
| LR | 133.5° | 100.3 – 195.1 mm | 51–100% |
| RF | 129° | 105.2 – 195.1 mm | 54–100% |
| **LF** | 118.5° | **117.0 – 195.1 mm** | 60–100% |

The six legs do **not** share a workspace: LF's floor is 28 mm higher than
RM/LM's because its calibrated tibia sits 25.5° further round. A pose legal for
the middle legs can be illegal for LF. `check_cycle()` samples the whole gait
for all six legs and refuses to run if any of them binds.

### 5.4 "LF is 14° past straight, geometrically impossible" — retracted

With a 25T horn each leg's femur↔tibia straight-line constant is set by its own
spline tooth, so `femur + tibia` is **not** a straightness invariant across legs.
The observed spread (cmd_t − cmd_f from −8 to +8, i.e. −12° to +12° physical) is
normal horn-mounting variation and per-leg calibration absorbs it correctly.
Same for the coxa left/right asymmetry (RF/LF 17°, RR/LR 16°, RM/LM 0°).

The self-test now proves this: at mirrored body targets the *geometric* offsets
from each leg's own calibration match to 1.4e-14, while the raw angles differ by
exactly the calibration spread.

**What still holds:** a single calibration pose cannot distinguish a horn offset
from a placement error. If LF's leg was not truly perpendicular to the body when
cal_coxa = 115 was recorded, that error is baked into LF's yaw zero and nothing
in the data can reveal it. Worth a physical square-check against the frame.

### 5.5 Silent clamping removed

`leg_ik()` used to clamp all three joints into range and return them without
complaint, so an unreachable target quietly became a saturated servo command.
It now raises `WorkspaceError` naming the joint and the reason. Pass
`allow_out_of_range=True` to get the old behaviour deliberately.

### 5.6 Channel map

The old robot addressed 24 servos on channels 1–31 with gaps — six of them
(24, 25, 28, 29, 30, 31) do not exist on a 24-channel board. `servo_lsc24.py`
uses a contiguous **1-based** map: legs 1–18, tail 19–22, claws 23–24.

**Channel 0 must never be used.** Hiwonder's driver does
`servo_id = 254 if (servo_id < 1 or servo_id > 254)`, and 254 is the broadcast
address — commanding channel 0 moves all 24 servos at once. The servo layer
rejects it explicitly.

## 6. Files

| file | role |
|---|---|
| `servo_lsc24.py` | the only place that knows pulse widths, channel numbers and serial timing |
| `ik_module.py` | IK + FK, workspace limits, calibration loading, self-test |
| `tripod_gait.py` | gait config, foot trajectory, margin/torque checking, walk runner |
| `walk.py` | bring-up CLI |
| `leg_calibration.json` | per-servo zeros, belly-down pose (legacy units, scaled on load) |
| `stand_sit_walk_tail_claw_belly.py` | **legacy**, see §7 |

Still absent from the repo: `ServoControl.py` (Hiwonder's driver — the LSC-16/32
version uses the identical LOBOT protocol and works), and `servo_calibration_gui.py`.

## 7. Do not mix the legacy file with the new stack

`stand_sit_walk_tail_claw_belly.py` still contains its own `angle_to_pulse()`
with the /180 mapping, and its `POSITIONS`, `TAIL_POSES` and `CLAW_POSES` tables
were tuned by trial and error **through that mapping**. Run on its own it behaves
exactly as it always did. But its numbers are not interchangeable with the new
modules: a stored `90` means 135 physical degrees there and 90 physical degrees
here.

Migrating the tail and claw means changing the mapping **and** multiplying every
stored angle by 1.5, then re-verifying each pose on hardware — any that were
already against a mechanical stop will now be commanded past it. Deliberately
left undone; the current focus is six-leg walking.

## 8. Verification status

Verified by math, `python3 walk.py --self-test` passes:

- FK at the calibrated angles returns u = 259.59, v = 0, z = 0 for all six legs
- IK→FK round trip to 0.000000 mm across 47 bent-knee poses
- 1 out-of-band target correctly rejected rather than clamped
- joint-direction assertions including the old lift-test regression guard
- `body_to_leg_local` / `leg_local_to_body` round trip and left/right mirror symmetry
- workspace rejection for over-stretch, over-bend and inboard-of-coxa targets

**Still needs a physical check** — none of this has touched a servo:

1. Joint signs and the knee-up branch (§3)
2. Channel-to-servo mapping (`python3 walk.py --identify`)
3. Which UART, and whether the LSC-24's TX is 3.3 V or 5 V
4. Over-current jumpers on K, low-voltage alarm on 6.4 V
5. Actual robot mass
