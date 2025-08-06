# Unitree‑G1 Arm Executor

> **Control & scripting framework for the 2‑arm, 5‑DoF‑per‑arm Unitree G1 platform.**
>
> *Play, log and compose motion sequences from Python — identical code path for real hardware and simulator (dry‑run).*
> *Control loop: 50 Hz · Linear trajectory planner · Safety watchdog*

---

## 1. Quick start

```bash

# Run in DRY‑RUN mode (no hardware needed)
$ python application.py examples/wave.json lo --dry-run --log info

# On real robot (see **Safety** first!)
$ python application.py examples/wave.json <network-interface>--log info 
```

### Safety

* **Clear the workspace** before enabling torque.
* Joint limits are enforced in software (`ControlConfig.joint_limits`) but *never trust software alone*.
* A watchdog stops motion when incoming state is older than `enable_watchdog_timeout` (default 0.5 s).

---

## 2. Repository layout

| Path                  | Purpose                                                                       |
| --------------------- | ----------------------------------------------------------------------------- |
| `application.py`      | Thin runtime wrapper that wires every component, handles CLI & logging.       |
| `cli.py`              | Text UI: lists sequences, parses user commands, delegates to `ArmController`. |
| `arm_controller.py`   | **Heart of the system** – 50 Hz loop: state → trajectory → LowCmd.            |
| `arm_io.py`           | IO backend: abstracts Unitree SDK DDS channels & dry‑run stubs.               |
| `command_builder.py`  | Converts desired joint map `{idx: q}` → `LowCmd` with KP/KD.                  |
| `sequence_loader.py`  | Loads JSON files → `ArmSequence` (list of `ArmPose`).                         |
| `sequence_manager.py` | Keeps all sequences, feeds next pose, appends auto “return‑home”.             |
| `trajectory.py`       | Jerk‑free **linear** planner & segment sampler.                               |
| `g1_5joints.py`       | Maps readable joint names → SDK indices (5 per arm).                          |
| `ultility.py`         | Logger helper and misc tiny utils.                                            |

> **One file · one responsibility.**  Replace any piece (e.g. planner, IO) without touching the rest.

---

## 3. Control‑loop life cycle

```mermaid
graph TD
    A[SequenceManager.next_pose()] -->|target vector| B[TrajectoryPlanner.plan()]
    B --> C[ArmController _control_step()]
    C -->|q_des| D[CommandBuilder.make_cmd()]
    D --> E[IArmIO.write_cmd()]
    E -->|LowCmd| F[Unitree G1 arms]
    C -->|t >= duration| G[Hold logic]\n(re‑enter C)
```

1. **Pose fetch** – when no active segment, controller asks the manager for the next `ArmPose`.
2. **Planning** – linear interpolation; duration derived from max joint delta & velocity cap.
3. **Sampling** – every tick calls `seg.sample(t)` for smooth set‑points.
4. **Packaging** – `CommandBuilder` fills `LowCmd` (`q`, `kp`, `kd`) & optional torque enable.
5. **Execution** – `ArmIO` sends DDS or prints to console in dry‑run.
6. **Hold / advance** – optional per‑pose pause; when all poses done, controller freezes & triggers CLI menu.

---

## 4. Configuration (`config.py`)

| Parameter                 | Default   | Description                              |
| ------------------------- | --------- | ---------------------------------------- |
| `control_dt`              | 0.02 s    | Control cycle period (50 Hz).            |
| `max_velocity`            | 0.5 rad/s | Global velocity cap; scaled per pose.    |
| `kp`, `kd`                | 60, 1.5   | PD gains applied uniformly.              |
| `enable_watchdog_timeout` | 0.5 s     | No fresh state → hold posture.           |
| `soft_stop_duration`      | 0.5 s     | Ramp gains to zero on shutdown.          |
| `joint_limits`            | `{}`      | Optional per‑joint `(min, max)` radians. |

Adjust values once – all modules read from this dataclass.

---

## 5. Writing a sequence

```jsonc
{
  "name": "greeting_wave",
  "loop": false,
  "poses": [
    { "L_ARM": [0.0, -0.5, 0.0, -1.1, 0.0], "R_ARM": [0,0,0,0,0], "speed_scale": 1.0, "hold": 0.2 },
    { "L_ARM": [0.0, -0.3, 0.0, -0.8, 0.0], "R_ARM": [0,0,0,0,0], "speed_scale": 0.8, "hold": 0.2 }
  ]
}
```

* Keys correspond to 5 physical joints per arm.
* `speed_scale` multiplies global `max_velocity`.
* `hold` adds a pause after reaching that pose.
* At runtime the **first detected posture** is automatically appended as a final *return‑home* pose.

---

## 6. Extending / hacking

| Want to…                                    | Touch these files                              |
| ------------------------------------------- | ---------------------------------------------- |
| Change interpolation (e.g. cubic)           | `trajectory.py`                                |
| Add inverse kinematics / task‑space planner | new module + swap call in `ArmController`      |
| Support different robot or more DoF         | update `g1_5joints.py`, joint mapping & limits |
| Log to CSV / ROSbag                         | hook into `ArmController._control_step()`      |
| GUI instead of CLI                          | replace `cli.py`                               |

PRs welcome – keep one class per file.

---

## 7. Troubleshooting

* **Menu keeps spamming** → ensure `ArmController._completion_sent` latches the callback once.
* **`enableArmSdk` attribute error** → using dry‑run without real SDK (safe to ignore). Pass `--dry-run` to suppress warning.
* **Robot stiff at startup** → press Estop, verify joint limits and `kp/kd`.

---

## 8. License



---

Made by Exxeta:Hightech with ❤️ - beat  by Exxeta Prometheus Team. Pull requests, bug reports and feature ideas are highly appreciated!
