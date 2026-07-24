# Third-party projects

This project uses and modifies files from the following open-source projects.

## Unitree Go2 ROS2 simulation

- Repository: `unitree-go2-ros2`
- Purpose: Unitree Go2 robot description, CHAMP controller and Gazebo integration
- Project modifications:
  - Added an RGB-D camera Xacro
  - Included the camera in the Go2 robot description
  - Fixed ROS2 `robot_description` string parameter handling
  - Added the `gui` condition to the Gazebo launch file
  - Adjusted gait parameters for cave terrain testing

## Gazebo Cave World

- Repository: `gazebo_cave_world`
- Purpose: Cave simulation environment
- Project modifications:
  - Repositioned survivor models to reachable regions
  - Saved the modified environment as `cave_world_rescue.world`

Refer to each upstream repository for its original license and copyright.
