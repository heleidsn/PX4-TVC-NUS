# PX4-TVC-NUS

A fork of [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) v1.16 for **TVC (Thrust Vectoring Control)** aircraft SITL simulation in Gazebo, used together with the ROS 2 controllers in the `TVC_ws` workspace.

## Purpose

- Provide PX4 flight control and Gazebo simulation support for an inverted coaxial servo TVC drone
- Launch SITL with airframe ID `6003` and load the `tvc` Gazebo model for simulation
- Serve as a submodule of `TVC_ws` for integration with external LQR/PID control nodes

## Launch

Build first (from the `PX4-TVC-NUS` directory):

```bash
make px4_sitl
```

Start the TVC SITL simulation:

```bash
PX4_SYS_AUTOSTART=6003 PX4_SIM_MODEL=tvc PX4_GZ_WORLD=default ./build/px4_sitl_default/bin/px4
```

| Environment Variable | Description |
|---|---|
| `PX4_SYS_AUTOSTART=6003` | Select airframe `6003_tvc` |
| `PX4_SIM_MODEL=tvc` | Load the TVC Gazebo model |
| `PX4_GZ_WORLD=default` | Use the default Gazebo world |

## Key Parameter Changes

These parameters are adjusted for TVC operation with external ROS 2 control. Apply them in QGroundControl or via `param set` after startup.

### Control Allocator & Flight Behavior

| Parameter | Default | TVC Value | Description |
|---|---|---|---|
| `CA_SP0_ANG1` | 90 deg | 140 deg | Servo tilt angle for thrust vectoring |
| `CA_SP0_COUNT` | 2 | 3 | Number of servos on the tilt mechanism |
| `MC_AIRMODE` | Roll/Pitch | Disabled | Disable airmode for cleaner thrust control |
| `COM_DISARM_PRFLT` | 0.0 s | 10.0 s | Delay auto-disarm after preflight checks |
| `FD_ESCS_EN` | Disabled | Enabled | Enable ESC failure detection |
| `FD_FAIL_R` | 180 deg | 60 deg | Roll failure detection threshold |

### Simulation & Airframe

| Parameter | Default | TVC Value | Description |
|---|---|---|---|
| `SYS_AUTOSTART` | 6002 | 6003 | Switch to the TVC airframe |
| `SIM_GZ_EC_FUNC1` | Counter-clockwise Rotor | Disabled | Disable direct Gazebo CCW motor mapping |
| `SIM_GZ_EC_FUNC2` | Clockwise Rotor | Disabled | Disable direct Gazebo CW motor mapping |
| `SIM_GZ_SV_FUNC1` | Servo 1 | Disabled | Disable direct Gazebo servo 1 mapping |
| `SIM_GZ_SV_FUNC2` | Servo 2 | Disabled | Disable direct Gazebo servo 2 mapping |

Disabling the Gazebo motor/servo function mappings allows actuation to be handled externally by the ROS 2 controller instead of PX4's built-in mixer output.

## Changes from Upstream PX4

1. **Added airframe configuration `6003_tvc`**
   - Path: `ROMFS/px4fmu_common/init.d-posix/airframes/6003_tvc`
   - Configured for Gazebo Harmonic (`gz`) simulation with the `tvc` model loaded by default
   - Sets `MAV_TYPE=3` (Rocket) and `CA_AIRFRAME=12` (custom bicopter with servo tilt)
   - Enables GPS/magnetometer simulation, disables barometer simulation; disables auto-disarm on landing (`COM_DISARM_LAND=0`)

2. **Added TVC Gazebo simulation model**
   - Path: `Tools/simulation/gz/models/tvc/`
   - Includes SDF model files, model config, and propeller meshes
   - Models an inverted coaxial TVC aircraft with servo gimbal, dual motors, and IMU/GPS/magnetometer sensors

3. **Vendored `Tools/simulation/gz` as regular files**
   - Upstream uses this directory as a git submodule; it is now vendored directly to allow in-repo maintenance of the custom TVC model

4. **Registered airframe in the build system**
   - Added `6003_tvc` to `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
