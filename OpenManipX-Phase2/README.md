# OpenManipulator-X: Project 1, Phase 2 (Real-Robot Experiments)

RBE 502 Robot Control. Due **Friday, November 13, 2026, 11:59 PM** (Canvas).

Phase 2 runs the two Phase 1 differential-kinematics controllers on the real
OpenManipulator-X and compares each run with a simulation that uses the same gains:

- **Task 1:** end-effector position control, `qdot = J⁺ Kp e`
- **Task 2:** position control with translational manipulability maximization in the
  null space, `qdot = J⁺ Kp e + (I − J⁺J) k0 ∇m_t`

The desired position is `p_d = [0.20, 0.20, 0.15]ᵀ` m, and every run starts from home, `q = 0`.

---

## 1. Folder layout

```
OpenManipX-Phase2/
├── README.md                                  ← this file
├── code/
│   ├── robotParams.m                          link lengths (m)
│   ├── openManipulatorPosition.m              forward kinematics p(q)
│   ├── analyticJacobian.m                     3×4 position Jacobian J(q)
│   ├── analyticJacobianDerivatives.m          ∂J/∂q_i, for ∇m_t
│   ├── controlLawTask1.m                      Task 1 control law
│   ├── controlLawTask2.m                      Task 2 control law
│   ├── problem1_position_control_sim.m        Task 1 simulation
│   ├── problem2_manipulability_control_sim.m  Task 2 simulation
│   ├── problem1_position_control_robot.m      Task 1 on the robot   (required name)
│   ├── problem2_manipulability_control_robot.m Task 2 on the robot  (required name)
│   └── plotComparison.m                       sim vs robot figures and summary numbers
├── data/                                      .mat logs (sim and robot)
└── report/
    └── figures/                               PDFs written by plotComparison
```

The sim and robot scripts call the **same** helper functions (FK, Jacobian, control laws).
Any difference between simulation and hardware therefore comes from the real system
(sample time, servo tracking, communication delay, model error), not from the code.

The Jacobian and its derivatives are analytic, as the spec requires. No numerical
differentiation is used anywhere in the controllers.

---

## 2. Prerequisites

1. **MATLAB** with the Robotics System Toolbox. It's only needed if you turn on `animate`
   in the sims.
2. **Dynamixel SDK for MATLAB.** Follow *Environment Setup* in the repository
   [README](../README.md), then set the SDK paths in [`init_libraries.m`](../init_libraries.m).
3. **Serial port access** to the robot:
   ```bash
   sudo chmod a+rw /dev/ttyUSB0
   ```
4. The robot scripts add [`../Communication_Code`](../Communication_Code) to the path
   themselves, so they can be run from any folder.

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

Run `problem1_position_control_sim` and `problem2_manipulability_control_sim`.
The printed results should match:

| | Final ‖e‖ (m) | Final m_t (m³) | Final q (rad) |
|---|---|---|---|
| Task 1 | ≈ 5.3e-14 | ≈ 9.70e-3 | [0.816, 0.072, 0.064, 0.020] |
| Task 2 | ≈ 2.0e-6 | ≈ 1.07e-2 | [0.816, 0.159, −0.320, 0.609] |

If they match, the shared helpers are correct. **Put the settings back afterwards**
(Kp = 1.5, k0 = 20, `qd_max` = 0.5, `Tfinal` = 10) and delete the Phase 1 check files
from `data/` if you don't need them.

> At the Phase 1 gains, the first control step commands about 3.7 rad/s (≈ 214°/s).
> That's why the robot runs start with a lower `Kp` and a 0.5 rad/s saturation.

### Step 2: Generate the comparison sims for the first robot gains (offline)

With the default settings (Kp = 1.5, k0 = 20, `qd_max` = 0.5, Tfinal = 10), run both
sim scripts. This creates:

```
data/sim_task1_Kp1.5_sat0.5.mat
data/sim_task2_Kp1.5_k020_sat0.5.mat
```

Expected (final ‖e‖, final m_t; both reach 1 mm at about 3.9 s):

| | Final ‖e‖ (m) | Final m_t (m³) | Final q (rad) |
|---|---|---|---|
| Task 1 | ≈ 7.0e-8 | ≈ 9.71e-3 | [0.816, 0.072, 0.061, 0.026] |
| Task 2 | ≈ 7.5e-7 | ≈ 1.02e-2 | [0.816, 0.091, −0.090, 0.291] |

### Step 3: Test the plotting function (offline)

Pass a sim file as both arguments. This catches plotting errors before you have robot data:

```matlab
plotComparison("sim_task1_Kp1.5_sat0.5.mat", "sim_task1_Kp1.5_sat0.5.mat", "test")
```

Check the PDFs in `report/figures/`, then delete the `test_*` files.

### Step 4: Check the hardware connection (lab, AK 120)

1. Plug in the robot and power it on. Make sure the arm has clearance all around.
2. Run [`example_joint_control.m`](../example_joint_control.m). The robot should go to
   home (`q = 0`), turn joint 1 to −30°, then +30°, then back to 0°.

### Step 5: Check the model's joint directions (lab, do this once before velocity control)

The controller assumes the robot's joint signs and zero position match the kinematic
model. **If one sign is flipped, the controller drives the arm away from the target.**

In position mode, command one joint at a time to +15° and compare what the arm does
with what the model predicts:

| Command (deg) | The model predicts |
|---|---|
| `[0 0 0 0]` | home pose, p ≈ [0.280, 0, 0.188] m: vertical upper link, horizontal forearm and gripper |
| `[15 0 0 0]` | the arm turns toward **+y** (counter-clockwise seen from above) |
| `[0 15 0 0]` | the upper link tilts **forward**, so the gripper moves forward and down |
| `[0 0 15 0]` | the forearm tilts **down** |
| `[0 0 0 15]` | the gripper pitches **down** |

```matlab
addpath("Communication_Code");
robot = Robot(); robot.writeMode('p'); robot.writeTime(2); robot.writeMotorState(true);
robot.writeJoints([0 15 0 0]); pause(2.5);
r = robot.getJointsReadings(); q = deg2rad(r(1,:)')
p = openManipulatorPosition(q, robotParams())   % compare with where the gripper actually is
robot.writeJoints([0 0 0 0]); pause(2.5);
```

If a joint moves the opposite way, stop. Don't run the controllers until the sign
convention is fixed.

### Step 6: Run Task 1 on the robot

1. Check clearance. Keep one hand ready to support the arm and one person ready to cut power.
2. In MATLAB:
   ```matlab
   problem1_position_control_robot
   ```
3. The script asks twice for Enter, because torque turns off briefly at every mode change.
   **Hold the arm both times.**
   - `SUPPORT THE ARM, then press Enter to home the robot (position mode)...`
     The robot then moves to `q = 0`.
   - `SUPPORT THE ARM, then press Enter to switch to velocity mode...`
4. After the second Enter, the controller runs for 10 s at Ts = 0.05 s.
5. When it finishes, it prints how the run ended, the final ‖e‖, and the mean and maximum
   sample time. It saves a file like:
   ```
   data/robot_task1_Kp1.5_sat0.5_complete_20261111_093512.mat
   ```

**What the script does each sample:** it reads the measured joint angles and velocities
(deg → rad), checks the joint limits, computes `p(q)`, `e` and `qdot` from the control law,
saturates, sends velocities (rad/s → deg/s), logs, and waits until Ts has passed.

**Safety features:**
- **Velocity saturation:** commands are scaled so that no joint exceeds `qd_max`.
  Scaling the whole vector keeps the direction of motion.
- **Joint-limit stop:** the run stops if any joint gets within 10° of its URDF limit.
  The file is then tagged `limit`.
- **Clean stop:** zero velocities are always sent when the script exits, including on
  **Ctrl+C**. The log is saved after a normal finish (tagged `complete`), a joint-limit stop
  (`limit`), an implausible joint reading (`badread`), or a MATLAB error (`error`).
  **A Ctrl+C run is not saved.**
- **Torque stays on** after the run. The arm holds itself at zero velocity.
  **Support the arm before the next run**, because homing calls `writeMode('p')`.

### Step 7: Run Task 2 on the robot

Same procedure as Step 6:
```matlab
problem2_manipulability_control_robot
```
It saves `data/robot_task2_Kp1.5_k020_sat0.5_<ended>_<timestamp>.mat` and also prints the
final m_t.

### Step 8: Retune the gains (optional, but discuss it in the report)

For each new gain set:

1. **Set the new values in the sim script first** (`Kp`, `k0`, `qd_max`) and run it.
   The spec requires every gain set to be checked in simulation before it goes on the robot.
   Check the joint velocities and joint ranges.
2. Set the **same values** in the robot script and run it.
3. Keep every `.mat` file, including failed runs. They're evidence for the retuning discussion.

**Suggested order:**
- **Task 1:** raise `Kp` step by step (1.5 → 3 → 5) and watch for oscillation or a bigger
  gap between commanded and measured q̇. For a discrete-time controller, stability needs
  roughly `Kp · Ts < 2` (Siciliano §3.7.1), and the **achieved** Ts matters, not the target.
- **Task 2:** with `Kp` fixed, raise `k0` (20 → 50 → 100). A large `k0` can make the
  null-space motion saturate the joints.

### Step 9: Make the report figures

Match each robot run with the sim file that has the same gains:

```matlab
plotComparison("sim_task1_Kp1.5_sat0.5.mat", ...
               "robot_task1_Kp1.5_sat0.5_complete_20261111_093512.mat", "task1")
plotComparison("sim_task2_Kp1.5_k020_sat0.5.mat", ...
               "robot_task2_Kp1.5_k020_sat0.5_complete_20261111_094210.mat", "task2")
```

It warns you if `Kp`, `k0` or `qd_max` differ between the two files.

| Figure | Contents | Spec |
|---|---|---|
| `<tag>_joint_angles.pdf` | q_i(t), sim vs robot | §4.1 (1) |
| `<tag>_joint_velocities.pdf` | sim, robot commanded and robot measured q̇_i | §4.1 (2) |
| `<tag>_ee_position.pdf` | p_x, p_y, p_z vs desired | §4.1 (3) |
| `<tag>_error_norm.pdf` | ‖e(t)‖, log scale | §4.1 (4) |
| `task2_manipulability.pdf` | m_t(q(t)) | §4.2 |
| `<tag>_sample_time.pdf` | achieved vs target sample time | §5 |

The 2×2 and 1×3 figures are full page width (`figure*` in IEEEtran). The others fit
one column (`figure`).

The function also prints the numbers §5 asks for: final ‖e‖, final q and final m_t for
sim and robot, plus the robot's mean and maximum sample time.

### Step 10: Write the report (§5 of the spec)

The report must use LaTeX with the IEEEtran conference template. Cover:

- the final ‖e‖ on the robot vs in simulation, for each controller
- the final configurations and final m_t of Task 1 vs Task 2 on the robot, and whether
  they confirm your Phase 1 conclusions about singular configurations
- the causes of the sim/real differences:
  - achieved sample time (see `_sample_time.pdf`)
  - how well the servos follow commanded velocities (commanded vs measured q̇)
  - communication delay
  - kinematic model error, such as link lengths and joint offsets
- any gain retuning: what you changed, why, and the effect
- **a section on each group member's contribution**

### Step 11: Package the submission

Each student submits their own zip, named with their WPI username (`abc` = the part of
the email before `@wpi.edu`):

```bash
cd OpenManipX-Phase2
mkdir -p ../submission/abc_project1_phase2
cp -r code data ../submission/abc_project1_phase2/
cp report/abc_project1_phase2.pdf ../submission/abc_project1_phase2/
cd ../submission && zip -r abc_project1_phase2.zip abc_project1_phase2
```

Required structure (a different structure gets a penalty):
```
abc_project1_phase2.zip
├── code/   (both *_robot.m files + helpers)
├── data/   (the .mat files behind the report plots)
└── abc_project1_phase2.pdf
```

Before zipping, delete test files and runs that aren't in the report from `data/`. The
robot scripts expect `Communication_Code/` two levels up. To make `code/` run on its own,
copy `Communication_Code/` into it and change the `addpath` line.

---

## 4. Data file reference

Every `.mat` file uses the same variable names, so `plotComparison` can read sim and
robot files the same way:

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

## 5. Default parameters

| Parameter | Value | Where | Note |
|---|---|---|---|
| `Kp` | 1.5 | sim + robot | Phase 1 used 5 |
| `k0` | 20 | Task 2 sim + robot | Phase 1 used 100 |
| `qd_max` | 0.5 rad/s | sim + robot | uniform-scaling saturation |
| `Ts` / `dt` | 0.05 s | sim + robot | |
| `Tfinal` | 10 s | sim + robot | Task 2's m_t is still rising at 5 s |
| Joint limits | q1 ±1.5, q2 ±1.5, q3 [−1.5, 1.4], q4 [−1.7, 1.97] rad | robot | from the URDF; q1 narrowed for cable safety |
| Limit margin | 10° | robot | |

**Always change the sim and robot scripts together.**

---

## 6. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `Attempt to execute SCRIPT ... as a function` | a helper is missing its `function` line |
| `Unrecognized function or variable 'Robot'` | `Communication_Code` isn't on the path, or the SDK isn't set up (`init_libraries.m`) |
| Can't open `/dev/ttyUSB0` | `sudo chmod a+rw /dev/ttyUSB0`, or check the USB cable and power |
| Arm drops when the script starts | expected during `writeMode`: support the arm before pressing Enter |
| Arm moves away from the target | a joint sign or zero offset doesn't match the model: redo Step 5 |
| Run stops right away with `limit` | the arm didn't reach home, or a joint is near a limit: check `q` at the start |
| Mean sample time well above 50 ms | USB latency; close other programs, and report the achieved Ts |
| Measured q̇ lags the commands | servo velocity tracking or profile acceleration; discuss it in §5, and compare plots before and after any change |
| `plotComparison` warns about a gain mismatch | re-run the sim with the robot's `Kp`/`k0`/`qd_max` |
| Steady-state error on the robot but not in sim | expected (gravity sag, deadband, link-length mismatch); discuss it in §5 |
