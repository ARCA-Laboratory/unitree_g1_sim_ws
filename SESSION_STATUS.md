# Session Status - Unitree G1 Simulation Workspace

## Current State: READY TO PUSH

### Git Status
- **2 commits ready to push:**
  1. `093b824` - Initial commit: Unitree G1 humanoid simulation with ROS 2 and MuJoCo
  2. `66ae189` - Fix display.launch.py condition and update README dependencies

- **Remote:** `origin` -> `https://github.com/ARCA-Laboratory/unitree_g1_sim_ws`
- **Branch:** `master`

### To Push
```bash
git push -u origin master
```
(Requires authentication - run manually in terminal)

---

## What Was Built

### Packages Created
1. **g1_description** - URDF/Xacro robot description, RViz config
2. **g1_sim_bridge** - MuJoCo-ROS2 bridge with 29-DOF position control
3. **g1_sim_bringup** - Launch files for simulation, demos, teleop

### Key Features
- MuJoCo physics simulation at 500Hz
- ROS 2 topics: `/joint_states`, `/imu/data`, `/tf`, `/joint_commands`
- Keyboard teleop for arm/waist control
- Demo modes: standing, wave, squat, arms_up
- Fixed base mode (robot pinned for testing)

### Files Modified During Session
- `src/g1_sim_bridge/g1_sim_bridge/mujoco_ros2_bridge.py` - Changed to position actuators, added fixed_base
- `src/g1_sim_bringup/launch/g1_sim.launch.py` - Fixed PythonExpression, use_sim_time=false
- `src/g1_description/rviz/g1_display.rviz` - Fixed frame to 'world', enabled TF
- `src/g1_description/launch/display.launch.py` - Fixed IfCondition bug
- `README.md` - Fixed executable names, added missing dependencies

---

## Running the Simulation

### Quick Start
```bash
# Source environment
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Run simulation
ros2 launch g1_sim_bringup g1_sim.launch.py
```

### Keyboard Teleop (separate terminal)
```bash
ros2 run g1_sim_bridge keyboard_teleop
```

### Demo Modes
```bash
./scripts/run_sim.sh wave
./scripts/run_sim.sh squat
```

---

## Excluded from Git (in .gitignore)
- `src/unitree_mujoco/` - Official Unitree repo (clone separately)
- `src/unitree_ros/` - Official Unitree repo (clone separately)
- `src/unitree_sdk2/` - Official Unitree repo (clone separately)
- `build/`, `install/`, `log/` - Build artifacts
- `.claude/`, `__pycache__/` - Tool/cache files

---

## Known Issues Fixed
1. CycloneDDS required: `sudo apt install ros-humble-rmw-cyclonedds-cpp`
2. Robot was falling - fixed with position actuators + fixed_base mode
3. RViz frame errors - changed fixed frame to 'world'
4. TF not publishing - set use_sim_time to false
5. display.launch.py had wrong condition syntax - fixed with IfCondition

---

## Next Steps (if continuing)
1. Push to GitHub: `git push -u origin master`
2. Test on fresh machine following README
3. Optional: Add walking controller, more demos, floating base mode
