[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# Robotic arm - Pick & Place project

![Kuka KR210 Photo](./misc_images/KukuKR210.JPG)

The KUKA KR 210 industrial robot arm is a high payload solution for serious industrial applications. With a high payload of 210 kg and a massive reach of 2700 mm, the KR 210 KR C2 robot is ideal for a foundry setting. In fact, a foundry wrist with IP 67 protection is available with the KR 210 KR C2 instead of the standard IP 65 wrist. (photo and text  www.[robots.com](https://www.robots.com/robots/kuka-kr-210))


### A.  Create a Catkin Workspace

1. Download the VM provided by Udacity

2. Create a top level catkin workspace directory and a sub-directory named `src`
```
mkdir -p ~/catkin/src
```
3. Navigate to the `src`
```
cd ~/catkin_ws/src
```
4. Initialize the catkin Workspace
```
catkin_init_workspace
```
Notice that a symbolic link (`CMakeLists.txt`) has been created to
`/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake`
![](./misc_images/02b.jpeg)

5. Return to the top level directory
```
cd ~\catkin_ws
```
6.  Build the Workspace
```
catkin_make
```
For more information on the catkin build, go to [ROS wiki](http://wiki.ros.org/catkin/conceptual_overview)

------

### B.  Clone the [Project Repository](https://github.com/udacity/RoboND-Kinematics-Project) into the src directory

1. Clone this repository to your home directory.

   ```sh
   $ git clone https://github.com/udacity/RoboND-Kinematics-Project
   ```

2. Now from the terminal window
```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```

3.  Build the project
   ```
   $ cd ~/catkin_ws
   $ catkin_make
   ```

4. Add following to your .bashrc file
   ```
   export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

   source ~/catkin_ws/devel/setup.bash
   ```

5. For demo mode make sure the demo flag is set to "true" in inverse_kinematics.launch file under /RoboND-Kinematics-Project/kuka_arm/launch

    In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the spawn_location argument in target_description.launch file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

    You can launch the project by
   ```sh
   $ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
   $ ./safe_spawner.sh
   ```

6.  To run your own Inverse Kinematics code change the demo flag described above to "false" and run your code (once the project has successfully loaded) by:
```
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```

Note:
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:
```
  - Robot

  - Shelf

  - Blue cylindrical target in one of the shelves

  - Dropbox right next to the robot
```
If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully.

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location.

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

------

### C.  Development Notes

1. For demo mode, make sure the demo flag is set to true in `inverse_kinematics.launch` file under `~/catkin_ws/src/kuka_arm/launch/`

2. You can also control the spawn location of the target object in the shelf by modifying the spaw_location argument in `target_description.launch` file under `~/catkin_ws/src/kuka_arm/launch`.
0-9 valid values for spawn_location with 0 being random mode.

3. To run forward kinematics test:

   ```sh
   $ roslaunch kuka_arm forward_kinematics.launch
   ```

4. To run simulator:

   ```sh
   $ rosrun kuka_arm safe_spawner.sh
   ```

5. To run IK Server:

   ```sh
   $ rosrun kuka_arm IK_server.py
   ```

   ##
