# Introduction Task Elbflorance

This ROS package controls the movement of a turtle in the turtlesim window using `/turtle_teleop_key` Node. The turtle moves in response to arrow key commands, and the background color changes based on the side of the canvas it hits using the `turtlesim_color_change` Node.

## Dependencies
Before running this ROS package, make sure you have the following dependencies installed:

- ROS2 (Robot Operating System): Iron Irwini
- turtlesim package

You can install the turtlesim package using the following command:
```bash
sudo apt-get install ros-iron-turtlesim
```

## Important: Don't Forget to Source ROS

Before running ROS packages, ensure that you have sourced the ROS environment in your terminal. If you forget to do this, you may encounter issues with package execution.

To source ROS, use the following command in your terminal:

```bash
source /opt/ros/iron/setup.bash
```

# Running the ROS Package
Follow these steps to run the ROS Package to change the background color of the turtlesim depending on which wall is hit by the turtle.

1. Clone the Git repository to your local machine:
```bash
git clone https://github.com/rainshere/Intro_Task_Elbflorance
```
2. Navigate to the root of the cloned repository:
```bash
cd Intro_Task_Elbflorance/
```
3. Build the ROS package:
```bash
colcon build
```
4. Source the ROS environment:
```bash
source install/local_setup.bash
```
Make sure to run this command in every new terminal where you plan to use the ROS package.
5. Launch the ROS Package nodes using the launch.py file:
```bash
ros2 launch turtlesim_color_change color_launch.py
```
This launch command will initiate the following nodes:

**turtlesim**: Simulates a turtle in a graphical environment.
**turtle_teleop_key**: Provides teleoperation control for the turtle using keyboard arrow keys.
**change_background**: Monitors the turtle's position and changes the background color based on canvas borders.

To control the turtle's movement within the turtlesim window, make sure to select the `turtle_teleop_key` terminal. Use the arrow keys to navigate the turtle, and observe the background color changes as the turtle hits different sides of the canvas.
