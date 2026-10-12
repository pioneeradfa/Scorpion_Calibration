# RIGHT FRONT leg — geometry, calibrated stand pose, inverse kinematics

**Phase: single-leg bring-up.** Companion figure:
[`rf_leg_diagram.png`](rf_leg_diagram.png).  The document itself moves nothing;
the commands in §4 do, and only after an explicit confirmation.
Scope note: tail and claw are out of scope; only RF (channels 1/2/3) is considered here.

---

## 1. Dimensions and geometry — the complete summary

### Linkage (measured, confirmed against the flat-lay photo)

| item | value | note |
|---|---|---|
| coxa link (L1) | **64.25 mm** | yaw axis → hip pitch axis |
| femur (L2) | **64.25 mm** | hip pitch → knee pitch |
| tibia + foot (L3) | **135.60 mm** | knee pivot → foot tip |
| provenance | annotated bench photo, 2026-10-10 | pivot-to-pivot dashed lines (`uploads/image-1.png`); supersedes the earlier 64.5 / 64.5 / 130.59 set |
| RF coxa mount (body frame) | x = +115, y = −60 | x forward, y left, z up; origin body centre |
| RF coxa zero direction | straight outboard (−Y) | yaw measured from there |

### Body plate as-built (CAD photo, this branch's `uploads/` WhatsApp 17.42.33)

| dimension | as-built | nominal | dev |
|---|---|---|---|
| column gap front-mid, side A / B | 115.28 / 115.05 | 115.00 | +0.28 / +0.05 |
| column gap mid-rear, side A / B | 115.04 / 114.97 | 115.00 | +0.04 / −0.03 |
| lateral gap front / mid / rear | 119.96 / 120.03 / 120.12 | 120.00 | −0.04 / +0.03 / +0.12 |
| long diagonals | 259.92 / 259.26 | 259.42 | +0.50 / −0.16 |

Worst case 0.28 mm of column error and 0.66 mm of diagonal skew over 260 mm
(0.15° of plate rotation). Propagated through the IK at the stand foot, a
0.3 mm mount error moves a solved joint angle by at most **0.26°** — under the
servo's own 0.3° accuracy and 1/58 of a horn tooth. **The nominal ±115 / ±60
map in `ik_module.MOUNT_POS` therefore stands unchanged**; there is nothing to
trim until a square-check on the robot shows a bias larger than the servo
noise.
| hip→foot straight stretch | 64.25 + 135.60 = **199.85 mm** | d max |
| yaw axis→foot full reach | 64.25 + 199.85 = **264.10 mm** | leg perfectly straight |
| knee fully folded | \|135.60 − 64.25\| = **71.35 mm** | d min at zero knee travel |

The flat-lay photo shows exactly this ratio: two equal short aluminium links
(coxa, femur) then a black truss roughly twice as long (tibia + foot).

### Servos and units

| item | value |
|---|---|
| servo | Hiwonder HPS-2027, 20 kg·cm stall, **500–2500 µs = 0–270°** |
| pulse per physical degree | 2000/270 = **7.407 µs** |
| legacy command unit (old `/180` code) | pulse = 500 + A/180·2000 → **physical = 1.5 × A** |
| horn | 25T → one tooth = 14.40° physical = 9.60 legacy units |
| safe command band | 2–268° physical |

### RF joint angles, three poses, in TRUE physical degrees

| pose | source | coxa | femur | tibia |
|---|---|---|---|---|
| belly-down zero | `leg_calibration.json` 98/88/94 ×1.5 | 147.0 | 132.0 | 141.0 |
| **STAND (initial posture)** | `leg_stand.json` 97/114/27 ×1.5 (was 102/110/35) | **145.5** | **171.0** | **40.5** |
| sit | your table 102/80/60 ×1.5 | 153.0 | 120.0 | 90.0 |

### Updated six-leg STAND calibration (`leg_stand.json` / `POSITIONS["stand"]`)

| leg | legacy (coxa / femur / tibia) | physical ×1.5 (coxa / femur / tibia) | yaw | femur elev | knee bend / interior | foot local (u, v, z) mm | foot body (x, y, z) mm |
|---|---|---|---|---|---|---|---|
| **RF** | 97 / 114 / 27 | **145.5 / 171.0 / 40.5** | −1.5° | +39.0° | 100.5° / 79.5° | (178.8, −4.7, −78.7) | (+110.3, −238.8, −78.7) |
| **RM** | 88 / 117 / 19 | **132.0 / 175.5 / 28.5** | −3.0° | +37.5° | 97.5° / 82.5° | (182.8, −9.6, −78.3) | (−9.6, −242.8, −78.3) |
| **RR** | 101 / 110 / 25 | **151.5 / 165.0 / 37.5** | −3.0° | +34.5° | 97.5° / 82.5° | (178.5, −9.4, −84.4) | (−124.4, −238.5, −84.4) |
| **LR** | 89 / 110 / 30 | **133.5 / 165.0 / 45.0** | −3.0° | +30.0° | 91.5° / 88.5° | (184.3, −9.7, −87.0) | (−124.7, +244.3, −87.0) |
| **LM** | 90 / 112 / 22 | **135.0 / 168.0 / 33.0** | 0.0° | +33.0° | 93.0° / 87.0° | (185.9, 0.0, −82.4) | (+0.0, +245.9, −82.4) |
| **LF** | 113 / 116 / 39 | **169.5 / 174.0 / 58.5** | +3.0° | +34.5° | 93.0° / 87.0° | (187.8, +9.8, −79.2) | (+124.8, +247.8, −79.2) |

Compared with the earlier stand table (`102/110/35`, etc.), this updated stand calibration trims all six legs into much tighter agreement:
* **Foot height (`z`) spread** across all six legs drops from **27.0 mm** (−69.2 to −96.2 mm) to **8.7 mm** (−78.3 to −87.0 mm), with front pair RF/LF within 0.5 mm (−78.7 vs −79.2 mm).
* **Outboard reach (`u`) spread** drops from **39.8 mm** (175.0–214.8 mm) to **9.3 mm** (178.5–187.8 mm).
* **Coxa yaw** is now within ±3.0° of straight out on every leg (eliminating RF's earlier +6.0° / +20.4 mm forward toe).
* **Horizontal hip→foot moment arm (`L`)** on the right side pulls in from 129.6–150.6 mm to **114.3–118.5 mm**, reducing standing femur torque.

### What the RF stand pose means in millimetres (forward kinematics)

| quantity | value |
|---|---|
| coxa yaw | **−1.5°** (foot 4.7 mm aft of mount axis, essentially straight out) |
| femur elevation | **+39.0°** above the coxa plane |
| knee bend / interior angle | **100.5° / 79.5°** |
| tibia angle from horizontal | −61.5° |
| knee height above coxa plane | **+40.4 mm** (knee is the highest point — matches photo) |
| foot, leg-local | u = 178.8 outboard, v = −4.7 forward, z = −78.7 (L = 114.6 mm outboard of hip pivot) |
| foot, body frame | (x, y) = (+110, −239) |
| **body height at stand** | **78.7 mm** computed on RF (78.3–87.0 mm across all six legs) |
| stance width (foot to foot) | 478 mm (RF), ~486 mm average across all pairs |
| d (hip→foot) | 139.1 mm = **70 % of stretch** — comfortable mid-workspace |

---

## 2. Joint sign conventions — the finding that gates everything

Only two of the four possible sign pairs put the foot on the ground at all,
and only one of those matches the photo (knee at the top, foot far outboard):

| FEMUR_SIGN | TIBIA_SIGN | femur elev | knee bend | foot z | knee z | verdict |
|---|---|---|---|---|---|---|
| +1 | +1 | +39.0 | −100.5 | **+128.5** | +40.4 | foot ABOVE the body — impossible |
| **+1** | **−1** | +39.0 | +100.5 | **−78.7** | **+40.4** | **valid, knee UP — the photo** |
| −1 | +1 | −39.0 | −100.5 | +78.7 | −40.4 | foot above the body — impossible |
| −1 | −1 | −39.0 | +100.5 | −128.5 | −40.4 | valid but knee DOWN — contradicts photo |

So, in plain language, for this robot:

* **coxa: increasing raw angle swings a right leg FORWARD** (unchanged).
* **femur: increasing raw angle RAISES the leg** (unchanged, `FEMUR_SIGN = +1`).
* **tibia: increasing raw angle STRAIGHTENS the knee; decreasing it CURLS the knee.**
  In formula form `phi = −1 × (raw_tibia − cal_tibia)`, i.e. **`TIBIA_SIGN = −1`**.

`ik_module.py` now ships `TIBIA_SIGN = -1` (flipped in the same commit as this
document), and `WALKING_NOTES.md` §4 carries the matching expectation: on a
lift, the raw tibia angle must FALL. Consequence for reach limits: with
`TIBIA_SIGN = −1` the RF knee can bend up to `cal_tibia − 2 = 139°`, so RF's
d floor becomes **96.8 mm** with the 135.60 mm tibia. Per-leg floors now:
RF 96.8, RM/LM 113.0, RR 103.1, LR 101.5, LF 86.6; the six-leg `--check`
verdict stays FEASIBLE with the tightest margin 36.6° on RM.

### 30-second confirmation on the bench (do this before walking)

Prop the robot so RF is free. From the stand pose, change **only the tibia**:

| command (legacy units) | physical | expect if `TIBIA_SIGN = −1` is right |
|---|---|---|
| tibia 27 → 40 | 40.5 → 60.0 | knee **straightens**, foot swings out and up |
| tibia 27 → 14 | 40.5 → 21.0 | knee **curls**, foot tucks under the body |

If it is the other way round, keep `TIBIA_SIGN = +1` and tell me — then the
stand table's tibia column was recorded through a different zero and we
re-derive it.

---

## 3. Inverse kinematics for the RF leg

Body frame → leg-local (RF): `u = −(y + 60)`, `v = x − 115`, `z = z`.
All angles below in true physical degrees; `F = 64.25`, `T = 135.60`, `C = 64.25`.

```
1  yaw   = atan2(v, u)                      coxa_raw  = 147.0 + yaw
2  r     = hypot(u, v)                      (foot distance from the yaw axis)
   L     = r − C                            (horizontal, hip pivot → foot)
   d     = hypot(L, z)                      (straight line, hip pivot → foot)
       guard: 96.8 ≤ d ≤ 199.85             else WorkspaceError
3  alpha = atan2(z, L)                      (angle of d below/above horizontal)
   beta  = acos((F² + d² − T²) / (2·F·d))   (hip angle between d and femur)
   elev  = alpha + beta                     ← knee-UP branch
                                              femur_raw = 132.0 + elev
4  gamma = acos((F² + T² − d²) / (2·F·T))   (interior knee angle)
   phi   = 180 − gamma                      (bend away from straight)
                                              tibia_raw = 141.0 − phi     ← TIBIA_SIGN = −1
5  sanity: 2 ≤ each raw ≤ 268
```

Why `alpha + beta` (knee up) and not `alpha − beta`: with the knee below the
coxa plane the same stand target solves to a femur raw of −7°…−57° across
plausible stances — off the servo entirely. The photo settles it visually too:
the knee servo is the highest joint on the leg.

### Worked example — the stand pose itself

Target = the stand foot: u = 178.82, v = −4.68, z = −78.73.

```
yaw   = atan2(−4.68, 178.82)      =  −1.50°      coxa_raw  = 147.0 − 1.50  = 145.50
r     = 178.88   L = 114.63   d   = hypot(114.63, 78.73) = 139.07   (70 % of stretch)
alpha = atan2(−78.73, 114.63)     = −34.48°
beta  = acos((4128.06 + 19339.4 − 18387.36) / (2·64.25·139.07)) = acos(0.2843) = +73.48°
elev  = −34.48 + 73.48            = +39.00°      femur_raw = 132.0 + 39.00 = 171.00
gamma = acos((4128.06 + 18387.36 − 19339.4) / (2·64.25·135.60)) = acos(+0.1823) = 79.50°
phi   = 180 − 79.50               = 100.50°      tibia_raw = 141.0 − 100.50 = 40.50
```

Round trip through the code returns **145.50 / 171.00 / 40.50** —
the IK reproduces your hand-calibrated stand pose to the hundredth of a degree.

Forward kinematics (used to check, and to produce the millimetre table in §1):

```
elev = +(femur_raw − 132.0)          phi = −(tibia_raw − 141.0)
tibia_abs = elev − phi
horiz = 64.25 + 64.25·cos(elev) + 135.60·cos(tibia_abs)
z     =         64.25·sin(elev) + 135.60·sin(tibia_abs)
u = horiz·cos(yaw)   v = horiz·sin(yaw)   yaw = coxa_raw − 147.0
```

---

## 4. STAND as the initial posture, and one smooth RF cycle around it

Implemented in this commit: `walk.py --user-stand` drives all six legs to the
hand-calibrated table (RF = raw 145.5 / 171.0 / 40.5) in one onboard-interpolated
move, and `walk.py --rf-cycle` runs the cycle below on **channels 1/2/3 only**,
the other five legs holding their pose — the single-leg bring-up step.

```bash
python3 walk.py --user-stand --force-dry    # pace it, send nothing
python3 walk.py --rf-cycle --force-dry      # RF cycle, send nothing
python3 walk.py --user-stand                # for real, robot propped
python3 walk.py --rf-cycle --cycles 2       # watch RF swing/stance twice
```

`--rf-cycle` treats the stand foot (178.8, −4.7, −78.7) as the cycle centre.
One 2000 ms cycle, stride 50 mm, lift 25 mm (figure, bottom panel):

| phase | foot | joints |
|---|---|---|
| stance 0 → 0.5 | planted at z = −78.7, sweeps v +20.3 → −29.7 (body moves forward) | coxa does the work: 153.5 → 137.6 |
| swing 0.5 → 1 | smoothstep forward, z = −78.7 + 25·sin(πt) | femur 170.0 → 190.6 → 170.5, tibia 42.3 → 29.1 → 41.3 |

Resulting raw ranges for the whole cycle: **coxa 137.6–153.5, femur 170.0–190.6,
tibia 29.1–42.3** — every curve continuous with zero velocity at touch-down and
lift-off, d between 126.6 and 141.0 mm (63–71 % of stretch), and the deepest
servo position 29.1° still 27° inside the safe band. At 9600 baud a 3-servo
array frame is 14 B = 14.6 ms, so 16 segments of 125 ms pipeline with ~11 % of
each segment spent on the wire.

### One caution: the stand stance width vs femur torque

Femur torque = leg load × horizontal distance hip→foot, independent of body
height. With the updated stand table that arm on RF is L = 114.6 mm = 11.5 cm
(down from 13.1 cm in the earlier 102/110/35 table). On a 3.0 kg robot a tripod
leg carries ≈1.0 kg → **11.5 kg·cm = 57 % of stall** static (or ~80 % with ×1.4
dynamic factor).

Two levers, neither of which touches your stand pose (it stays the pre-walk
posture): tuck the feet in for the walking stance (`GaitConfig` default uses
`reach_out = 80 mm` → L = 80 mm → 11.5 kg·cm = 58 % of stall including ×1.4
dynamic), and/or verify the real mass and re-run `walk.py --check --mass <kg>`.
Recommend: stand → short transition to the tucked walking stance → gait.

---

## 5. Open points (all RF-only, all cheap to settle)

1. **Tibia jiggle test** (§2) — the code now ships `TIBIA_SIGN = -1` on the
   strength of the stand-table + photo evidence; the jiggle is the physical
   confirmation. If it disagrees, flip the constant back and re-derive.
2. **The sit row does not compute as a sit.** Under the confirmed convention it
   puts the body 134.2 mm up (higher than stand's 78.7 mm) with the leg at 92 %
   of stretch. A sit should be lower and more folded. Was the robot propped or
   lifted when that row was recorded? Not a blocker for walking, but it means
   the sit row cannot be used as a second calibration point until re-checked.
3. **RF coxa yaw resolved** — the updated stand calibration (`97` vs belly-down
   `98`) puts RF yaw at −1.5° (−4.7 mm aft), eliminating the earlier +6° (+20.4 mm)
   forward bias.
4. **All six stand rows checked** — see the six-leg table in §1; all six legs
   now sit within an 8.7 mm height band (`z` −78.3 to −87.0 mm) and a 9.3 mm
   outboard reach band (`u` 178.5–187.8 mm), and all round-trip through the IK
   to 0.00° error (`walk.py --self-test`).

---

## 6. Reconciliation with `skills.md`

`skills.md` (uploaded to the branch) agrees with this document on everything
measured: link lengths, mount positions, body frame, coxa pointing straight
out, and the calibration pose description (belly down, leg dead straight).
Four of its claims predate the 270-degree servo correction and are superseded:

| skills.md says | status | why |
|---|---|---|
| servo pins 29/30/31 for RF | **superseded** | a 24-channel board has no channels 29-31; the contiguous 1-based map puts RF on 1/2/3 (`servo_lsc24.py`). `--identify` settles the physical wiring |
| "0°-180° mechanical range", safe band 15-165° | **superseded** | HPS-2027 is 500-2500 µs = 0-270°. The 0-180 assumption is exactly the legacy `/180` bug; the safe band here is 2-268° physical |
| "Tibia: increasing raw angle curls up" | **contradicted** | incompatible with skills.md's own calibration description: cal = leg straight, and the stand pose (which skills.md does not contain) sits 88.5° BELOW cal tibia with the knee bent 88.5°. A hinge bends one way, so raw down = bend. See §2 |
| safe plateau u = 220-255 mm, stance u = 230 / z = −35 | **artifact of the 180° clamp** | with the true 270° travel the knee bends to 139°, opening d down to 97 mm; and the bench photo shows the real stand at u = 193.9 / z = −76.8 with a 91.5° knee, not a near-straight leg at 35 mm body height |

Everything else in skills.md (sections 1-4, and the plan to validate RF alone
before combining legs) matches how this branch works. Its `rf_leg_gait.py`
self-test is not in the repository; the equivalent here is
`walk.py --self-test` plus `walk.py --rf-cycle --force-dry`.

---

## 7. Tape-measure cross-check (2026-10-09) — and why "13 cm" needs a reference point

Bench tape: clearance 6.5 cm; "ground coxa axis to foot" 13 cm; sketch segments
5.35 cm (knee→hip) and 6.8 cm (hip→coxa axis). Two of those confirm the model
outright: 6.8 cm is the coxa link (64.25 mm) and 5.35 cm is the computed
knee-to-hip horizontal at stand (53.9 mm).

The 13 cm does not, read literally. Solving the linkage for each possible
reading, against the commanded stand raws (165.0 femur / 52.5 tibia):

| reading of "13 cm" | L | z | solved elev / phi | raw femur (dev) | raw tibia (dev) | verdict |
|---|---|---|---|---|---|---|
| yaw axis → foot | 65.5 | −65 | +74.2 / 143.5 | 206.2 (+41.2) | −2.5 (−55.0) | **impossible**: tibia below its safe band, knee pinned; photo disagrees |
| hip pivot → foot, tape 65 | 130.0 | −65 | +41.9 / 94.6 | 173.9 (+8.9) | 46.4 (−6.1) | only if the loaded body sags 12 mm below command |
| hip pivot → foot, computed 76.8 | 130.0 | −76.8 | +33.3 / 89.1 | 165.3 (+0.3) | 51.9 (−0.6) | **best fit: within half a degree of the table** |

So the stand pose puts the foot **193.9-194.3 mm outboard of the yaw axis**
(13 cm outboard of the *hip pivot*), and the "13 cm" span in the sketch starts
at the hip bolt. At stand, yaw-axis→foot is ~19.4 cm — worth knowing before
anyone measures "reach" again.

The remaining gap is the clearance: tape 65 vs computed 76.8, now 11.8 mm.
Taken at face value it moves the solved femur by ~9°, which is beyond tape
noise -- so either the tape reference was not the coxa shaft (deck underside,
servo centreline), or the loaded robot genuinely sags ~12 mm below the
commanded pose, which would be a finding of its own (servo gear compliance
under load). Re-measure coxa shaft -> ground to decide; until then no constant
changes, because the horizontal closure (0.35 mm) and the half-degree angle
residual at the computed clearance both say the model is right. Note what it does
NOT affect: femur torque depends on L (hip→foot horizontal), and both admitted
readings put L at 128-130 mm, i.e. the §4 torque caution stands as written.

Bench re-measure snippet — paste any (horizontal-from-hip, clearance) pair and
compare against the commanded table:

```python
import math
F, T, C = 64.25, 135.60, 64.25        # femur, tibia, coxa
def tape_check(L, clearance):
    d = math.hypot(L, -clearance)
    alpha = math.degrees(math.atan2(-clearance, L))
    beta  = math.degrees(math.acos((F*F + d*d - T*T) / (2*F*d)))
    gamma = math.degrees(math.acos((F*F + T*T - d*d) / (2*F*T)))
    return 132.0 + alpha + beta, 141.0 - (180.0 - gamma)   # raw femur, raw tibia
print(tape_check(130.0, 65.0))    # -> (173.9, 46.4)  vs stand table (165.0, 52.5)
print(tape_check(130.0, 76.8))    # -> (165.3, 51.9)  <- the computed clearance closes it
```
