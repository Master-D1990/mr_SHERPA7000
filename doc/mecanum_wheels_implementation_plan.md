# Mecanum Wheel Implementation TODO List for Sherpa Robot

## Overview
This document outlines the necessary modifications to convert the Sherpa robot from standard wheels to mecanum wheels with a diameter of 13cm. The changes aim to be minimal while enabling full mecanum wheel capabilities including holonomic movement.

## Prerequisites
- [ ] Install required ROS packages:
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
  ```

## 1. URDF Modifications (`/home/david/catkin_ws/src/sherpa/sherpa_description/resources/sherpa.urdf`)

- [ ] Change wheel joints from fixed to continuous:
  - [ ] LF_WHEEL
  - [ ] RF_WHEEL 
  - [ ] LH_WHEEL
  - [ ] RH_WHEEL

- [ ] Update wheel geometry to reflect 13cm diameter mecanum wheels:
  - [ ] Change radius from 0.14 to 0.065 (13cm diameter)
  - [ ] Adjust wheel length if necessary (mecanum wheels are typically wider)

- [ ] Update wheel gazebo properties for mecanum behavior:
  - [ ] Set correct friction coefficients (mu1, mu2)
  - [ ] Configure fdir1 parameters for roller directions:
    - LF & RR wheels: fdir1="1 1 0" (45° roller orientation)
    - RF & LR wheels: fdir1="1 -1 0" (-45° roller orientation)

- [ ] Add transmission elements for each wheel to enable actuation

## 2. Controller Configuration (`/home/david/catkin_ws/src/sherpa/sherpa_control/config/control.yaml`)

- [ ] Update sherpa_velocity_controller:
  - [ ] Change wheel names to match URDF if needed (currently uses JOINT suffix)
  - [ ] Update wheel radius parameter to 0.065 (13cm diameter)
  - [ ] Consider switching to mecanum-specific controller if available (recommended for best functionality)
  - [ ] If staying with diff_drive_controller, configure it for mecanum-like behavior:
    - [ ] Set wheel_separation to appropriate value
    - [ ] Add linear.y velocity limits for lateral movement

## 3. Package Dependencies (`/home/david/catkin_ws/src/sherpa/sherpa_control/package.xml`)

- [ ] Ensure all necessary dependencies are included:
  - [ ] diff_drive_controller (or mecanum_drive_controller if available)
  - [ ] joint_state_controller
  - [ ] controller_manager
  - [ ] hardware_interface
  - [ ] Ensure velocity controllers are available

## 4. Launch File Check (`/home/david/catkin_ws/src/sherpa/sherpa_bringup/launch/sherpa_simulation.launch`)

- [ ] Verify the launch process loads the correct controllers
- [ ] No modifications likely needed if controller configuration files remain in the same location

## Implementation Notes

1. **Wheel Orientation**:
   - Diagonal wheels must have rollers in the same direction:
     - LF (front left) and RR (rear right): 45° rollers
     - RF (front right) and LR (rear left): -45° rollers

2. **Testing Strategy**:
   - [ ] Test basic movements first (forward/backward)
   - [ ] Test lateral movement (strafing left/right)
   - [ ] Test diagonal movement
   - [ ] Test rotation in place
   - [ ] Gradually increase speed to test stability

3. **Potential Issues**:
   - [ ] The standard diff_drive_controller may not fully support mecanum-specific movements
   - [ ] May need custom controller or configuration for full holonomic capabilities
   - [ ] Gazebo simulation may need fine-tuning of physics parameters

## Fallback Plan

If full mecanum wheel functionality cannot be achieved with the existing controller framework:

1. Consider implementing a custom mecanum controller plugin
2. Use modified diff_drive_controller with appropriate parameters for limited mecanum functionality
3. Implement velocity transformation in a separate node that converts desired holonomic motion to wheel velocities

## Final Verification

- [ ] Ensure robot can move in all directions (forward, backward, lateral, diagonal)
- [ ] Verify stability at different speeds
- [ ] Check for any simulation physics issues
