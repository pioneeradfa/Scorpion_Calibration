# RIGHT FRONT leg — geometry, calibrated stand pose, inverse kinematics

**Phase: single-leg bring-up.** Nothing in this document moves a servo.
Companion figure: [`rf_leg_diagram.png`](rf_leg_diagram.png).
Scope note: tail and claw are out of scope; only RF (channels 1/2/3) is considered here.

---

## 1. Dimensions and geometry — the complete summary

### Linkage (measured, confirmed against the flat-lay photo)

| item | value | note |
|---|---|---|
| coxa link | **64.5 mm** | yaw axis → hip pitch axis |
| femur | **64.5 mm** | hip pitch → knee pitch |
| tibia + foot | **130.59 mm** | knee pitch → foot tip |
| RF coxa mount (body frame) | x = +115, y = −60 | x forward, y left, z up; origin body centre |
| RF coxa zero direction | straight outboard (−Y) | yaw measured from there |
| hip→foot straight stretch | 64.5 + 130.59 = **195.09 mm** | d max |
| yaw axis→foot full reach | 64.5 + 195.09 = **259.59 mm** | leg perfectly straight |
| knee fully folded | \|130.59 − 64.5\| = **66.09 mm** | d min at zero knee travel |

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
| **STAND (initial posture)** | your table 102/110/35 ×1.5 | **153.0** | **165.0** | **52.5** |
| sit | your table 102/80/60 ×1.5 | 153.0 | 120.0 | 90.0 |

The stand row is byte-identical to `POSITIONS["stand"]["RF"]` in
`stand_sit_walk_tail_claw_belly.py` — i.e. these are the values that put the
real robot on its feet in the photo. That makes them hardware truth, which is
why they are used to settle the joint conventions below.

### What the stand pose means in millimetres (forward kinematics)

| quantity | value |
|---|---|
| coxa yaw | **+6.0°** (foot toed 20 mm forward) |
| femur elevation | **+33.0°** above the coxa plane |
| knee bend / interior angle | **88.5° / 91.5°** |
| tibia angle from horizontal | −55.5° |
| knee height above coxa plane | **+35.1 mm** (knee is the highest point — matches photo) |
| foot, leg-local | u = 191.5 outboard, v = +20.1 forward, z = −72.5 |
| foot, body frame | (x, y) = (+135, −252) |
| **body height at stand** | **72.5 mm** (photo deck ≈ 70 mm above the table — agrees) |
| stance width (foot to foot) | 503 mm |
| d (hip→foot) | 147.2 mm = **75 % of stretch** — comfortable mid-workspace |

---

## 2. Joint sign conventions — the finding that gates everything

Only two of the four possible sign pairs put the foot on the ground at all,
and only one of those matches the photo (knee at the top, foot far outboard):

| FEMUR_SIGN | TIBIA_SIGN | femur elev | knee bend | foot z | knee z | verdict |
|---|---|---|---|---|---|---|
| +1 | +1 | +33.0 | −88.5 | **+146.5** | +35.1 | foot ABOVE the body — impossible |
| **+1** | **−1** | +33.0 | +88.5 | **−72.5** | **+35.1** | **valid, knee UP — the photo** |
| −1 | +1 | −33.0 | −88.5 | +72.5 | −35.1 | foot above the body — impossible |
| −1 | −1 | −33.0 | +88.5 | −146.5 | −35.1 | valid but knee DOWN — contradicts photo |

So, in plain language, for this robot:

* **coxa: increasing raw angle swings a right leg FORWARD** (unchanged).
* **femur: increasing raw angle RAISES the leg** (unchanged, `FEMUR_SIGN = +1`).
* **tibia: increasing raw angle STRAIGHTENS the knee; decreasing it CURLS the knee.**
  In formula form `phi = −1 × (raw_tibia − cal_tibia)`, i.e. **`TIBIA_SIGN = −1`**.

`ik_module.py` currently ships `TIBIA_SIGN = +1`. **That must be flipped before
any gait runs**, together with the expected tibia direction in
`WALKING_NOTES.md` §4. Consequence for reach limits: with `TIBIA_SIGN = −1` the
RF knee can bend up to `cal_tibia − 2 = 139°`, so RF's d floor becomes
**92.2 mm** (it was 105.24 mm under the wrong sign) — more margin, not less.

### 30-second confirmation on the bench (do this before walking)

Prop the robot so RF is free. From the stand pose, change **only the tibia**:

| command (legacy units) | physical | expect if `TIBIA_SIGN = −1` is right |
|---|---|---|
| tibia 35 → 48 | 52.5 → 72.0 | knee **straightens**, foot swings out and up |
| tibia 35 → 22 | 52.5 → 33.0 | knee **curls**, foot tucks under the body |

If it is the other way round, keep `TIBIA_SIGN = +1` and tell me — then the
stand table's tibia column was recorded through a different zero and we
re-derive it.

---

## 3. Inverse kinematics for the RF leg

Body frame → leg-local (RF): `u = −(y + 60)`, `v = x − 115`, `z = z`.
All angles below in true physical degrees; `F = 64.5`, `T = 130.59`, `C = 64.5`.

```
1  yaw   = atan2(v, u)                      coxa_raw  = 147.0 + yaw
2  r     = hypot(u, v)                      (foot distance from the yaw axis)
   L     = r − C                            (horizontal, hip pivot → foot)
   d     = hypot(L, z)                      (straight line, hip pivot → foot)
       guard: 92.2 ≤ d ≤ 195.09             else WorkspaceError
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

Target = the stand foot: u = 191.5, v = +20.1, z = −72.5.

```
yaw   = atan2(20.1, 191.5)        =  +6.00°      coxa_raw  = 147.0 + 6.00  = 153.00
r     = 192.55   L = 128.05   d   = hypot(128.05, 72.5) = 147.15   (75 % of stretch)
alpha = atan2(−72.5, 128.05)      = −29.53°
beta  = acos((4160.25 + 21653.7 − 17053.75) / (2·64.5·147.15)) = acos(0.4615) = +62.52°
elev  = −29.53 + 62.52            = +32.99°      femur_raw = 132.0 + 32.99 = 164.99 ≈ 165.0
gamma = acos((4160.25 + 17053.75 − 21653.7) / (2·64.5·130.59)) = acos(−0.0261) = 91.50°
phi   = 180 − 91.50               =  88.50°      tibia_raw = 141.0 − 88.50 =  52.50
```

Round trip through the code returns **153.00 / 165.00 / 52.50** exactly —
the IK reproduces your hand-calibrated stand pose to the hundredth of a degree.

Forward kinematics (used to check, and to produce the millimetre table in §1):

```
elev = +(femur_raw − 132.0)          phi = −(tibia_raw − 141.0)
tibia_abs = elev − phi
horiz = 64.5 + 64.5·cos(elev) + 130.59·cos(tibia_abs)
z     =        64.5·sin(elev) + 130.59·sin(tibia_abs)
u = horiz·cos(yaw)   v = horiz·sin(yaw)   yaw = coxa_raw − 147.0
```

---

## 4. STAND as the initial posture, and one smooth RF cycle around it

`walk.py` should first drive RF to **raw 153.0 / 165.0 / 52.5** (2 s ramp), and
treat foot (191.5, +20.1, −72.5) as the cycle centre. One 2000 ms cycle,
stride 50 mm, lift 25 mm (figure, bottom panel):

| phase | foot | joints |
|---|---|---|
| stance 0 → 0.5 | planted at z = −72.5, sweeps v +25 → −25 (body moves forward) | coxa does the work: 160.3 → 145.5 |
| swing 0.5 → 1 | smoothstep forward, z = −72.5 + 25·sin(πt) | femur 162.8 → 182.6 → 162.8, tibia 56.2 → 42.0 → 56.2 |

Resulting raw ranges for the whole cycle: **coxa 145.5–160.3, femur 162.8–182.6,
tibia 42.0–56.2** — every curve continuous with zero velocity at touch-down and
lift-off, d between 135.6 and 147.6 mm (69–76 % of stretch), and the deepest
servo position 42.0° still 40° inside the safe band. At 9600 baud a 3-servo
array frame is 14 B = 14.6 ms, so 16 segments of 125 ms pipeline with ~11 % of
each segment spent on the wire.

### One caution: the stand stance is wide, and width costs torque

Femur torque = leg load × horizontal distance hip→foot, independent of body
height. At stand that arm is L = 128.1 mm = 12.8 cm. On a 3.0 kg robot a tripod
leg carries ≈1.0 kg → **12.8 kg·cm = 64 % of stall**, before any dynamic
factor. Holding that for minutes will cook the femur servos.

Two levers, neither of which touches your stand pose (it stays the pre-walk
posture): tuck the feet in for the walking stance (u ≈ 150–160 mm → L ≈ 86–96 mm
→ 8.6–9.6 kg·cm = 43–48 % of stall), and/or verify the real mass and re-run
`walk.py --check --mass <kg>`. Recommend: stand → 1 s transition to the tucked
walking stance → gait.

---

## 5. Open points (all RF-only, all cheap to settle)

1. **Tibia jiggle test** (§2) — decides `TIBIA_SIGN`. Everything else waits on it.
2. **The sit row does not compute as a sit.** Under the confirmed convention it
   puts the body 129.8 mm up (higher than stand's 72.5 mm) with the leg at 91 %
   of stretch. A sit should be lower and more folded. Was the robot propped or
   lifted when that row was recorded? Not a blocker for walking, but it means
   the sit row cannot be used as a second calibration point until re-checked.
3. **Coxa +6° at stand** toes the RF foot 20 mm forward of the mount axis.
   Intentional trim, or horn tooth? Harmless either way — the IK absorbs it —
   but worth knowing whether "straight out" is 153 or 147 for this leg.
4. LF/RM/RR/LM/LR rows of the same table have not been checked yet. They will
   be, one leg at a time, after RF walks.
