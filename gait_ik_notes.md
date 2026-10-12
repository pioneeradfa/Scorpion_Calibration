# Scorpion gait and IK - notes (2026-10-12)

Files
- `scorpion_gait.py`     leg IK/FK for 270-degree servos, `Gait` class, servo output, CLI
- `walk_one_step.py`     one complete tripod step (one full cycle) with the same safe sequence
- `walk.py`              any gait, any number of cycles, tick timing (`--tick-ms`), dry run by default
- `test_scorpion_gait.py` unit tests (no hardware): `python3 -m unittest -v test_scorpion_gait`
- Reference: `skills_12oct.md`

## Run
```
python3 walk.py --gait ripple_123456 --cycles 2     # dry run: plan and one cycle of angles
python3 walk.py --hardware --legs RF --cycles 1        # one leg, one cycle, asks YES
python3 walk.py --hardware --gait tripod --cycles 3 --tick-ms 250
python3 walk_one_step.py                    # dry run: prints the 12-tick step
python3 walk_one_step.py --hardware --legs RF   # one leg, asks YES
python3 walk_one_step.py --hardware             # full robot, asks YES
```
```
python3 scorpion_gait.py                          # checks (stand table, IK round trip, gait)
python3 scorpion_gait.py --gait ripple_123456 --table
python3 scorpion_gait.py --hardware --legs RF --cycles 1   # real servos, asks for YES
```
`--legs RF` moves one leg only (skills section 10 K: first hardware test on RF).

Hardware mode, in this order (each step is announced before the YES prompt):
1. Move to the initial stand `stand_initial` (`--pose-ms`, default 2000 ms). Always first.
2a. Walking: move to the walking neutral (`--pose`, default `stand`), glide to the first gait
    pose, walk `--cycles`, then return to the walking neutral.
2b. `--goto NAME`: move to that posture and stop. It stays there (no return move).
The prompt accepts `YES`, `yes` or `y`. Any other answer cancels with no motion.

## Postures (stand, initial stand, sit)
Raw angles per leg, [coxa (shoulder), femur, tibia]. Defined in `POSES` in `scorpion_gait.py`.

| Posture | Source | Status |
|---|---|---|
| `stand` (default) | skills 5.1, re-recorded 2026-10-12 | recorded on the robot; walking neutral |
| `stand_initial` | skills 5.1, superseded first manual stand | IK and band checked; spread 18 mm in height |
| `sit` | skills 5.2 | IK and band checked; **not verified** (see below) |

- `--pose NAME` sets the walking neutral (`stand` by default). The gait starts from it.
- `--hardware --goto NAME --legs RF` moves to that posture and stops (asks YES).
  Walking always starts from the active pose.
- Python: `sg.sit(bus)`, `sg.stand(bus)`, `sg.move_to_pose(bus, "sit")`, `sg.set_active_pose(...)`.

**Sit is not confirmed.** The §5.4 sit foot positions match the *old scale-1.0* model
exactly (0.0 mm). So that table was computed before the 270-degree correction and does not
describe the real sit. The §5.2 raw angles are the commands sent to the servos, so they are
usable, but the real sit has to be checked by eye:
1. `--hardware --goto sit --legs RF` (one leg), then the whole robot with the body supported.
2. Confirm the body rests where you expect. Under the 1.5x model the sit feet are about
   135-150 mm below the coxa axis, which is lower than the stand (about 80 mm).
   Your sit may use a different posture, such as resting on the belly (skills 5.6).
3. Re-record the sit on the robot (like the stand) if it differs. Then update `POSES["sit"]`.

## 270-degree servo correction (most important change)
- RDS3218 travels 270 deg. The calibration and the pulse mapping use a 0..180 "raw" scale.
- Physical angle = 1.5 x raw offset from the straight-leg zero (skills 5.3.1).
- The old IK (`ik_module.py`, `ik_7da.py`, `scorpion_controller_v2.py`) treats raw as physical.
  At the recorded stand this puts the feet **45-49 mm too high** in the model
  (femur elevation 23 deg instead of 34.5 deg for LF, for example).
- `scorpion_gait.py` uses 1.5. Its FK reproduces the skills 5.3.1 table: elevation and bend
  match exactly, and reach values match to 0.3 mm.
- Pulse: `500 + raw/180*2000`, the same as before. Equivalent to `500 + phys/270*2000`.
  This assumes the RDS3218 is 500-2500 us over 0-270 deg. Check one servo first.
- Safe band: 2..268 phys deg (skills 5.3.1). The tightest joint in the gait is a tibia at
  about 15.6 phys deg, so there is about 13 deg of margin at the default lift.

## Stand height
- Neutral feet = FK of the recorded stand (skills 5.1). Each leg keeps its own height,
  mean body height about 81.7 mm.
- `scorpion_controller_v2.py` uses GROUND_CLEARANCE = 65 mm. That does not match the
  recorded stand. Decide which one is right before using that controller.

## Gait
- Gaits (skills section 6 scheme): tripod, ripple_123456, ripple_654321, ripple_135246, double_ripple, caterpillar. Steps per cycle (`GAIT_DEFAULT_STEPS`): tripod 12 (2.4 s at 200 ms, 8.2 deg per tick), ripples and caterpillar 30 (6.0 s, 9.6 deg per tick), double ripple 24 (4.8 s, 6.3 deg per tick). `--steps` overrides. Caterpillar = one leg at a time in the order LR, LM, LF, RF, RM, RR (defined here; the old robot's `catapilar()` was an empty stub). Ripple orders: 123456 = LF RF LM RM LR RR; 654321 = RR LR RM LM RF LF; 135246 = LF LM LR RF RM RR.
- Steering by ICC (skills section 7). Each leg rotates about the ICC by the same angle, so
  the body moves as one rigid unit. Forward = `set_straight(True)`, backward = `set_straight(False)`,
  turn in place = `set_turn_in_place("CCW" | "CW")`.
- Stance: foot sweeps backward on the ground. Swing: foot sweeps forward with a sin lift.
- Stance and swing are symmetric about the neutral point, so the body does not drift sideways.
- Deviations from skills section 6:
  - Swing uses the full stride angle. Section 6.4 uses half (open point Q). Using half
    causes the start-up transient of section 6.7.
  - Swing is one sin-lift phase, not three linear phases (lift, swing, lower).
    The linear version has sharp corners at the top of the lift.
  - No stride ramp for the first cycle. A ramp moves the touchdown point each cycle.
- Checks run for all gaits in all four directions: no unreachable targets, no joint outside
  the band, the pattern repeats each cycle, and the correct number of legs swings each tick.
- Largest joint change per tick: 3.4-3.6 deg for tripod, about 10 deg for ripple gaits
  (fewer ticks per swing). The tick period is `--move-ms` (default 200 ms).

## Not yet verified on the robot (do in this order)
1. One servo (RDS3218): command pulses 500, 1500, 2500 and confirm the 0 / 135 / 270 deg
   positions. This checks the 270-degree pulse mapping.
2. One leg (`--legs RF`): stand, then the first gait pose. Check the foot moves in the
   expected direction: femur up = leg lifts, tibia sign (skills 10 E).
3. Lift height 30 mm and stride 45 mm on one leg before all six.
4. Then full walking. Start with a slow tick (`--move-ms 400`) and the robot propped up.

## Open items
- `skills_12oct.md` mentions a servo model HPS-2027 (270 deg). The robot uses RDS3218 (270 deg).
  The geometry is the same. Update the doc if that is the only difference.
- `skills_12oct.md` refers to `scorpion_controller_new_robot.py`, which is not in this repo.
  `scorpion_controller_v2.py` is the controller in the repo.
- Migrate `scorpion_controller_v2.py`, `ik_module.py` and `ik_7da.py` to the 1.5 scale, or
  point them at `scorpion_gait.py`. They were not changed in this step.
