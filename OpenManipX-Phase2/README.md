# OpenManipulator-X: Project 1, Phase 2 (Real-Robot Experiments)

RBE 502 Robot Control. Due **Friday, November 13, 2026, 11:59 PM** (Canvas).

Phase 2 runs the two Phase 1 differential-kinematics controllers on the real
OpenManipulator-X and compares each run with a simulation that uses the same gains:

- **Task 1:** end-effector position control, `qdot = J⁺ Kp e`
- **Task 2:** position control with translational manipulability maximization in the
  null space, `qdot = J⁺ Kp e + (I − J⁺J) k0 ∇m_t`

The desired position is `p_d = [0.20, 0.20, 0.15]ᵀ` m, and every run starts from home, `q = 0`.

---

## 0. Status (November 7, 2026)

**Hardware experiments are complete.** Both controllers ran on the robot, every gain set
was checked in simulation first, and every report figure has a sim overlay.

**Final gains**

| Task | Kp | k0 | `qd_max` | Ts | Tfinal |
|---|---|---|---|---|---|
| Task 1 | **3** | – | 0.5 rad/s | 0.05 s | 10 s |
| Task 2 | **1.5** | **50** | 0.5 rad/s | 0.05 s | 10 s |

**Main results (robot vs simulation, same gains)**

| | Final ‖e‖ robot | Final ‖e‖ sim | Final q robot (°) | Final m_t robot | Final m_t sim |
|---|---|---|---|---|---|
| Task 1, Kp = 3 | **0.75 mm** (repeat: 1.07 mm) | ≈ 0 | [46.7, 4.0, 4.2, 0.6] | 9.67e-3 | 9.71e-3 |
| Task 2, k0 = 50 | **1.9 mm** | ≈ 0 | [46.4, 8.3, −16.9, 33.6] | **1.069e-2** | 1.071e-2 |

- At the same tip position, Task 2 ends with **10.5% higher m_t** than Task 1 and a
  **13.5% larger σ_min(J)** (0.1039 vs 0.0915 m), so its pose is further from a singular
  configuration. Robot and sim agree to within 0.3%.
- The sample time held at **50.0 ms** (worst case 50.1 ms) in every run, so timing is not
  a cause of any difference between sim and robot.

**Still to do:** write the report (§10), add the Phase 1 conclusion and the group
contributions section, and package the submission (§11).

---

## 1. Folder layout

```
OpenManipX-Phase2/
├── README.md                                   ← this file
├── code/
│   ├── robotParams.m                           link lengths (m)
│   ├── openManipulatorPosition.m               forward kinematics p(q)
│   ├── analyticJacobian.m                      3×4 position Jacobian J(q)
│   ├── analyticJacobianDerivatives.m           ∂J/∂q_i, for ∇m_t
│   ├── controlLawTask1.m                       Task 1 control law
│   ├── controlLawTask2.m                       Task 2 control law
│   ├── problem1_position_control_sim.m         Task 1 simulation
│   ├── problem2_manipulability_control_sim.m   Task 2 simulation
│   ├── problem1_position_control_robot.m       Task 1 on the robot   (required name)
│   ├── problem2_manipulability_control_robot.m Task 2 on the robot   (required name)
│   └── plotComparison.m                        sim vs robot figures and summary numbers
├── data/                                       .mat logs (sim and robot), see §7
└── report/
    └── figures/                                PDFs written by plotComparison, see §8
```

The sim and robot scripts call the **same** helper functions (FK, Jacobian, control laws).
Any difference between simulation and hardware therefore comes from the real system
(sample time, servo tracking, communication delay, model error), not from the code.

The Jacobian and its derivatives are analytic, as the spec requires. No numerical
differentiation is used anywhere in the controllers.

The sims apply the **same velocity saturation as the robot** (`qd_max`, uniform scaling),
so each comparison uses identical gains and limits. Set `qd_max = Inf` to reproduce the
unsaturated Phase 1 sims.

---

## 2. Prerequisites

1. **MATLAB.** Base MATLAB is enough. The Robotics System Toolbox is only needed for
   `animate = true` in the sims. On Ubuntu 24.04, use R2024b or newer (R2026a was used here).
2. **Dynamixel SDK.** Build the C library (`~/DynamixelSDK/c/build/linux64`, then `make`)
   and set `dynamixel_library_path` in [`init_libraries.m`](../init_libraries.m) to your
   SDK location. This path is machine-specific, so each group member sets their own.
3. **Serial port access**, once per machine (then log out and back in):
   ```bash
   sudo usermod -aG dialout $USER
   ```
   For a one-off session, `sudo chmod a+rw /dev/ttyUSB0` works too.
4. The robot scripts add [`../Communication_Code`](../Communication_Code) to the path
   **and run `init_libraries.m` themselves**, so they work from any folder, even after
   `clear all` or a MATLAB restart.

---

## 3. Step-by-step procedure

### Step 1: Check the simulations reproduce Phase 1 (offline)

In both sim scripts, temporarily set the Phase 1 values:

| Parameter | Task 1 | Task 2 |
|---|---|---|
| `Kp` | 5 | 5 |
| `k0` | – | 100 |
| `qd_max` | `Inf` | `Inf` |
| `Tfinal` | 5 | 5 |

The printed results should match:

| | Final ‖e‖ (m) | Final m_t (m³) | Final q (rad) |
|---|---|---|---|
| Task 1 | ≈ 5.3e-14 | ≈ 9.70e-3 | [0.816, 0.072, 0.064, 0.020] |
| Task 2 | ≈ 2.0e-6 | ≈ 1.07e-2 | [0.816, 0.159, −0.320, 0.609] |

**Put the settings back afterwards** (see §5) and delete the check files from `data/`.

> At the Phase 1 gains, the first control step commands about 3.7 rad/s (≈ 214°/s).
> That's why the robot runs use a 0.5 rad/s saturation.

### Step 2: Run the comparison sims for the robot gains (offline)

Run the sim with exactly the gains the robot will use. Reference values for the gain sets
used so far (10 s, `qd_max` = 0.5):

| Sim file | Final ‖e‖ (m) | Final m_t (m³) | Final q (rad) |
|---|---|---|---|
| `sim_task1_Kp1.5_sat0.5.mat` | ≈ 7.0e-8 | 9.710e-3 | [0.816, 0.072, 0.061, 0.026] |
| `sim_task1_Kp3_sat0.5.mat` | ≈ 2.5e-14 | 9.710e-3 | [0.816, 0.072, 0.061, 0.026] |
| `sim_task2_Kp1.5_k020_sat0.5.mat` | ≈ 7.5e-7 | 1.022e-2 | [0.816, 0.091, −0.090, 0.291] |
| `sim_task2_Kp1.5_k050_sat0.5.mat` | ≈ 2.0e-6 | 1.071e-2 | [0.816, 0.153, −0.302, 0.587] |

### Step 3: Test the plotting function (offline)

```matlab
plotComparison("sim_task1_Kp3_sat0.5.mat", "sim_task1_Kp3_sat0.5.mat", "test")
```
You should get 5 PDFs with white backgrounds in `report/figures/`. Delete the `test_*`
files afterwards.

### Step 4: Check the hardware connection (lab, AK 120)

Run [`example_joint_control.m`](../example_joint_control.m). The robot should go to home
(`q = 0`), turn joint 1 to −30°, then +30°, then back to 0°.

### Step 5: Check the model's joint directions (once, before velocity control)

**If one joint sign differs from the model, the controller drives the arm away from the
target.** In position mode, command one joint at a time to +15° and compare:

| Command (deg) | The model predicts | Measured on our robot |
|---|---|---|
| `[0 0 0 0]` | p ≈ [0.280, 0, 0.188] m | q = [0.0, 1.8, 1.6, 0.4]°, p = [0.284, 0, 0.171] m (gravity sag) |
| `[15 0 0 0]` | turns toward **+y** (counter-clockwise from above) | y → +0.070 m ✓ |
| `[0 15 0 0]` | upper link tilts **forward** | x +0.019, z −0.068 m ✓ |
| `[0 0 15 0]` | forearm tilts **down** | z −0.058 m ✓ |
| `[0 0 0 15]` | gripper pitches **down** | z −0.030 m ✓ |

Joints 2 and 3 settle about 1.5–1.8° past each position command: gravity sag in position
mode. This doesn't affect the controllers, which compute FK from the **measured** angles,
but it means the robot starts slightly below the ideal home pose.

### Step 6: Run Task 1 on the robot

1. Check clearance. Keep a hand ready to support the arm and someone ready to cut power.
2. Run:
   ```matlab
   clear all
   problem1_position_control_robot
   ```
   `clear all` makes sure MATLAB isn't using an old cached copy of an edited script.
3. **Support the arm at both prompts.** Torque turns off briefly at every mode change.
   - `SUPPORT THE ARM, then press Enter to home the robot (position mode)...`
   - `SUPPORT THE ARM, then press Enter to switch to velocity mode...`
     **Take your hand away as soon as the arm firms up.** The controller starts immediately.
4. The console prints progress once per second, then a summary:
   ```
   Control loop running for 10 s (Ctrl+C to abort)...
     t =  1.0 s   ||e|| = 0.1664 m
     ...
   Stopped (complete). Saved 200 samples to .../data/robot_task1_Kp3_sat0.5_complete_<timestamp>.mat
   Final ||e||: ... m   Final m_t: ... m^3   mean dt: 0.0500 s   max dt: 0.0501 s
   ```

**Each sample:** read joint angles and velocities (deg → rad) → reject implausible readings
→ check joint limits → compute `p(q)`, `e` and `qdot` → saturate → send velocities
(rad/s → deg/s) → log → wait until Ts.

**Safety features:**
- **Velocity saturation:** commands are scaled so that no joint exceeds `qd_max`.
  Scaling the whole vector keeps the direction of motion.
- **Joint-limit stop:** the run stops if a joint gets within 10° of its URDF limit (`limit`).
- **Bad-reading stop:** a failed bulk read returns 0°. A jump of more than 10° between
  samples (physically impossible at `qd_max`) stops the run (`badread`).
- **Clean stop:** zero velocities are always sent when the script exits, including on
  **Ctrl+C**. The log is saved after a normal finish (`complete`), a limit or bad-read stop,
  or a MATLAB error (`error`). **A Ctrl+C run is not saved.**
- **Torque stays on** after the run, and the arm holds itself at zero velocity.
  **Support the arm before the next run**, because homing calls `writeMode('p')`.

> **Velocity-profile fix.** Right after `writeMode('v')`, the scripts call
> `robot.writeTime(2, 0)`. `writeMode` re-applies the homing profile (667 ms acceleration)
> in the servos' time-based profile mode, where every new velocity command restarts the
> ramp. With a new command every 50 ms, the joints lagged about 1 s behind the commands,
> overshot, and ignored small commands entirely (final error 14 mm). Setting the
> acceleration time to 0 removes the profile. See §9.

### Step 7: Run Task 2 on the robot

Same procedure:
```matlab
clear all
problem2_manipulability_control_robot
```
After the tip reaches the target (about 3 s), **the elbow and wrist keep slowly changing
angle while the tip stays in place**: that's the null-space motion increasing m_t.

### Step 8: Retune the gains

For each new gain set:
1. **Set the values in the sim script first** and run it (the spec requires every gain set
   to be checked in simulation). Check max |q̇| and the joint ranges.
2. Set the **same values** in the robot script, then `clear all` and run it.
3. Keep every `.mat` file. Failed runs are evidence for the retuning discussion.

**What we did:**

| Step | Change | Why | Effect on the robot |
|---|---|---|---|
| 1 | Velocity profile disabled (`writeTime(2, 0)`) | joints lagged ~1 s; small commands ignored | Task 1 final ‖e‖ 14 mm → 1.9 mm; final pose now matches sim |
| 2 | Task 1 Kp 1.5 → 3 | steady-state error set by the velocity resolution (∝ 1/Kp) | final ‖e‖ ~2.1 → 0.75 mm, 5 mm reached at 1.96 s instead of 2.81 s, no oscillation |
| 3 | Task 2 k0 20 → 50 | null-space commands were ~1 velocity step, so >50% was lost to rounding | q̇₄ tracking 41% → 84%; final m_t within 0.25% of sim (was 1%); tip error 2.9 → 2.0 mm |

### Step 9: Make the report figures

Match each robot run with the sim file that has the same gains:

```matlab
clear plotComparison
plotComparison("sim_task1_Kp3_sat0.5.mat",        "robot_task1_Kp3_sat0.5_complete_20261107_200812.mat",       "task1_Kp3")
plotComparison("sim_task2_Kp1.5_k050_sat0.5.mat", "robot_task2_Kp1.5_k050_sat0.5_complete_20261107_201742.mat", "task2_k050")
plotComparison("sim_task1_Kp1.5_sat0.5.mat",      "robot_task1_Kp1.5_sat0.5_complete_20261107_195655.mat",     "task1")
plotComparison("sim_task1_Kp1.5_sat0.5.mat",      "robot_task1_Kp1.5_sat0.5_complete_20261107_195517.mat",     "task1_profile")
plotComparison("sim_task2_Kp1.5_k020_sat0.5.mat", "robot_task2_Kp1.5_k020_sat0.5_complete_20261107_200233.mat", "task2")
```

It warns if `Kp`, `k0` or `qd_max` differ between the two files, and prints the §5 numbers
(final ‖e‖, final q, final m_t for sim and robot, and the robot's mean/max sample time).

| Figure | Contents | Spec |
|---|---|---|
| `<tag>_joint_angles.pdf` | q_i(t), sim vs robot | §4.1 (1) |
| `<tag>_joint_velocities.pdf` | sim, robot commanded and robot measured q̇_i | §4.1 (2) |
| `<tag>_ee_position.pdf` | p_x, p_y, p_z vs desired | §4.1 (3) |
| `<tag>_error_norm.pdf` | ‖e(t)‖, log scale, scaled to the robot's error | §4.1 (4) |
| `<tag>_manipulability.pdf` | m_t(q(t)), Task 2 only | §4.2 |
| `<tag>_sample_time.pdf` | achieved vs target sample time | §5 |

The 2×2 and 1×3 figures are full page width (`figure*` in IEEEtran). The others fit one
column (`figure`). Figures always export with a white background, even when MATLAB is in
dark mode.

### Step 10: Write the report (§5 of the spec)

IEEEtran conference template in LaTeX, named `abc_project1_phase2.pdf`. Cover:

- **Final ‖e‖, robot vs sim:** Task 1 0.75 mm, Task 2 1.9 mm, vs ≈ 0 in sim. The robot
  floor comes from the servo velocity resolution (§9).
- **Final configurations and m_t of the two tasks** (table in §0), and whether they
  confirm your **Phase 1 conclusions** about singular configurations. Add your Phase 1
  conclusion here; it isn't recorded in this repo.
- **Causes of the sim/real differences** (§9): the servo velocity profile (fixed), velocity
  resolution, about 100–150 ms servo lag, gravity sag at the start, null-space motion
  leaking into the tip position. Rule out sample time (`_sample_time.pdf`).
- **Retuning:** the table in Step 8, with the before/after figures.
- State that the comparison sims include the same saturation as the robot.
- **A section on each group member's contribution** (required).

### Step 11: Package the submission

Each student submits their own zip, named with their WPI username (`abc` = the part of the
email before `@wpi.edu`). **`code/` must run on its own**, so copy the communication code
and SDK setup into it and update the two path lines in each robot script:

```bash
cd ~/OpenManipulator-X/OpenManipX-Phase2
S=../submission/abc_project1_phase2
mkdir -p $S/data
cp -r code $S/
cp -r ../Communication_Code ../init_libraries.m $S/code/
cp report/abc_project1_phase2.pdf $S/
# copy only the .mat files used in the report into $S/data/
```

Then, in **both** robot scripts inside `$S/code/`, change
`fullfile(codeDir, "..", "..", "Communication_Code")` to `fullfile(codeDir, "Communication_Code")`
and `fullfile(codeDir, "..", "..", "init_libraries.m")` to `fullfile(codeDir, "init_libraries.m")`.
Finally:

```bash
cd ../submission && zip -r abc_project1_phase2.zip abc_project1_phase2
```

Required structure (a different structure gets a penalty):
```
abc_project1_phase2.zip
├── code/   (both *_robot.m files + helpers + Communication_Code + init_libraries.m)
├── data/   (only the .mat files behind the report plots)
└── abc_project1_phase2.pdf
```

---

## 4. Data file reference

Every `.mat` file uses the same variable names, so `plotComparison` can read sim and robot
files the same way:

| Variable | Size | Meaning |
|---|---|---|
| `t` | 1×N | time (s) |
| `q_log` | 4×N | joint angles (rad), measured on the robot |
| `qdot_cmd_log` | 4×N | commanded joint velocities (rad/s), after saturation |
| `qdot_meas_log` | 4×N | measured joint velocities (rad/s); equal to the commands in sim |
| `x_log` | 3×N | end-effector position from FK (m) |
| `e_log` | 3×N | position error `p_d − p` (m) |
| `m_log` | 1×N | translational manipulability m_t (m³) |
| `dt_log` | 1×N | achieved sample time (s); constant in sim |
| `Kp`, `k0`, `qd_max` | scalar | gains (`k0` only for Task 2) |
| `x_des`, `params` | – | desired position, link lengths |
| `Ts`, `stopFlag` | – | robot only: target sample time; `complete` / `limit` / `badread` / `error` |

---

## 5. Current parameters

| Parameter | Value | Where | Note |
|---|---|---|---|
| `Kp` (Task 1) | 3 | Task 1 sim + robot | Phase 1 used 5; first robot runs used 1.5 |
| `Kp` (Task 2) | 1.5 | Task 2 sim + robot | |
| `k0` | 50 | Task 2 sim + robot | Phase 1 used 100; first robot run used 20 |
| `qd_max` | 0.5 rad/s | sim + robot | uniform-scaling saturation |
| `Ts` / `dt` | 0.05 s | sim + robot | |
| `Tfinal` | 10 s | sim + robot | Task 2's m_t is still rising slowly at 10 s |
| Joint limits | q1 ±1.5, q2 ±1.5, q3 [−1.5, 1.4], q4 [−1.7, 1.97] rad | robot | from the URDF; q1 narrowed for cable safety |
| Limit margin | 10° | robot | |
| Velocity profile | acceleration time 0 in velocity mode | robot | homing keeps `writeTime(2)` |

**Always change the sim and robot scripts together.**

---

## 6. Robot behaviour to expect

- **Task 1 (Kp = 3):** the base turns ~47° to the left (+y) while the tip moves in a
  straight line to the target; within 5 mm at about 2 s, then it holds still.
- **Task 2 (k0 = 50):** same approach, then the wrist pitches down to ~34° and the elbow
  bends up to ~−17° while the tip stays within about 3 mm of the target.
- **A ~2 cm dip in p_z in the first 0.1 s** of each run, as the velocity loop takes the
  arm's weight after the mode switch.
- **Joints stop completely near the target.** Commands below one servo velocity step
  (1.37°/s) round to zero.

---

## 7. Data files

| File | What it is | Used for |
|---|---|---|
| `sim_task1_Kp1.5_sat0.5.mat` | Task 1 sim, Kp 1.5 | comparison for the Kp 1.5 runs |
| `sim_task1_Kp3_sat0.5.mat` | Task 1 sim, Kp 3 | comparison for the final Task 1 run |
| `sim_task2_Kp1.5_k020_sat0.5.mat` | Task 2 sim, k0 20 | comparison for the k0 20 run |
| `sim_task2_Kp1.5_k050_sat0.5.mat` | Task 2 sim, k0 50 | comparison for the final Task 2 run |
| `robot_task1_Kp1.5_..._195248.mat` | Task 1, Kp 1.5, **before profile fix**, run 1 | §5 (profile problem) |
| `robot_task1_Kp1.5_..._195517.mat` | Task 1, Kp 1.5, **before profile fix**, run 2 | §5 (profile problem), `task1_profile_*` |
| `robot_task1_Kp1.5_..._195655.mat` | Task 1, Kp 1.5, after fix | §5 (retuning), `task1_*` |
| `robot_task1_Kp1.5_..._200508.mat` | Task 1, Kp 1.5, repeat | §5 (repeatability) |
| `robot_task1_Kp3_..._200812.mat` | **Task 1, Kp 3, final** | §4.1, `task1_Kp3_*` |
| `robot_task1_Kp3_..._201705.mat` | Task 1, Kp 3, repeat (final ‖e‖ 1.07 mm) | §5 (repeatability) |
| `robot_task2_Kp1.5_k020_..._200233.mat` | Task 2, k0 20 | §5 (retuning), `task2_*` |
| `robot_task2_Kp1.5_k050_..._201742.mat` | **Task 2, k0 50, final** | §4.2, `task2_k050_*` |

Remove any file you don't use in the report before packaging (§11).

---

## 8. Figure sets

| Prefix | Run | Role in the report |
|---|---|---|
| `task1_Kp3_*` | Task 1, Kp 3 | **main Task 1 result** (§4.1) |
| `task2_k050_*` | Task 2, k0 50 | **main Task 2 result** (§4.2, includes m_t) |
| `task1_*` | Task 1, Kp 1.5 after fix | retuning comparison |
| `task2_*` | Task 2, k0 20 | retuning comparison |
| `task1_profile_*` | Task 1, before profile fix | servo-profile problem |

---

## 9. Findings: sim vs robot

1. **Servo velocity profile (fixed).** `writeMode` re-applies a 667 ms acceleration profile.
   In time-based profile mode each new command restarts the ramp, so the joints lagged
   ~1 s, overshot (error stuck near 36 mm at t ≈ 3 s) and ignored small commands (final
   error 14 mm, reproduced in two runs). `writeTime(2, 0)` fixed it.
2. **Velocity resolution sets the steady-state error.** One servo step is 1.37°/s
   (0.024 rad/s); smaller commands round to zero and the joints stop. With `qdot ∝ Kp·e`,
   the remaining error is ∝ 1/Kp: ~2.1 mm at Kp 1.5 → 0.69 mm (mean of the last 5 s) at
   Kp 3. Visible as steps in q₃, q₄ and as the flat part of the error plots.
3. **The same effect slows the Task 2 reconfiguration.** At k0 20 the null-space commands
   were ~1 step, and only 41% of the commanded q̇₄ was achieved; at k0 50 (~2.3 steps), 84%.
4. **Servo lag** of about 2–3 samples (100–150 ms) between commanded and measured velocity
   remains after the fix.
5. **Gravity sag** in position mode: the robot starts at q ≈ [0, 2.1, 1.5, 0.2]° and
   p_z = 0.171 m instead of 0.1875 m, then dips ~2 cm at the mode switch.
6. **Null-space motion leaks into the tip position** (p_z varies ±2.5 mm at k0 50), because
   the joints don't follow their commands exactly. In theory it shouldn't move the tip.
7. **Timing is not a cause.** Every run held 50.0 ms (worst case 50.1 ms); the USB latency
   timer on the lab machine is 1 ms.
8. **Repeatable.** Repeat runs agree within 0.3° per joint and ~0.3 mm in final error.

---

## 10. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `Could not find file dynamixel_sdk.h` | SDK paths not loaded; check `dynamixel_library_path` in `init_libraries.m` (the robot scripts run it automatically) |
| Error line numbers don't match the file | MATLAB is running a cached copy: `clear <functionName>` or `clear all` |
| `Attempt to execute SCRIPT ... as a function` | a helper is missing its `function` line |
| `Unrecognized function or variable 'Robot'` | `Communication_Code` isn't on the path |
| Can't open `/dev/ttyUSB0` | add yourself to `dialout` (§2), or check the USB cable and power |
| Arm drops when the script starts | expected during `writeMode`: support the arm before pressing Enter |
| Arm moves away from the target | a joint sign or zero offset doesn't match the model: redo Step 5 |
| Measured q̇ lags the commands by ~1 s, small commands ignored | velocity profile still active: check that `robot.writeTime(2, 0)` follows `writeMode('v')` |
| Run stops with `limit` right away | the arm didn't reach home, or a joint is near a limit: check `q` at the start |
| Run stops with `badread` | a joint reading failed (look for `groupBulkRead getdata failed`); check cables, re-run |
| Mean sample time well above 50 ms | USB latency timer: `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` should be 1 |
| Figures have black backgrounds | old `plotComparison`; the current one forces a light theme |
| `plotComparison` warns about a gain mismatch | re-run the sim with the robot's `Kp`/`k0`/`qd_max` |
| Steady-state error of a few mm on the robot | expected (velocity resolution, §9); a higher Kp lowers it |
