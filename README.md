tum_simulator on ROS Kinetic and Gazebo 7
=============

These packages are used to simulate the Parrot AR.Drone 2.0 in a ROS Kinetic environment using the Gazebo simulator. Altogether, there are 4 packages. Their functions are descript as below:

1. cvg_sim_gazebo: contains object models, sensor models, quadrocopter models, flying environment information and individual launch files for each object and a pure environment without any other objects.

2. cvg_sim_gazebo_plugins: contains gazebo plugins for the quadrocopter model. quadrotor_simple_controller is used to control the robot motion and deliver navigation information, such as: /ardrone/navdata. Others are plugins for sensors in the quadcopter, such as: IMU sensor, sonar sensor, GPS sensor.

3. message_to_tf: is a package used to create a ros node, which transfers the ros topic /ground_truth/state to a /tf topic.

4. cvg_sim_msgs: contains message forms for the simulator.

Some packages are based on the tu-darmstadt-ros-pkg by Stefan Kohlbrecher, TU Darmstadt.

This package depends on ardrone_autonomy package and gazebo7 so install these first.

How to install the simulator:

1. Install gazebo7 (comes with ROS Kinetic) and the ardrone_autonomy package (two options listed below)
    
    a) Installing ardrone_autonomy as a binary *(recommended)*
    ```
    sudo apt-get install ros-kinetic-ardrone-autonomy
    ```
    b) Installing ardrone_autonomy from source in your workspace
    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel
    $ cd ~/catkin_ws
    $ rosdep install --from-paths src -i
    $ catkin_make
    ```
    Of course, this can be found on the ardrone_autonomy documentation website: https://ardrone-autonomy.readthedocs.io/en/latest/installation.html

2. Navigate to your workspace's src folder
    ```
    cd ~/catkin_ws/src/
    ```

3. Clone this repository

    ```
    git clone https://github.com/jkleiber/ardrone_simulator_gazebo7
    ```
4. Build the simulator (and any other files in your workspace)

    ```
    cd ..
    catkin_make
    ```
4. Source the environment

    ```
    source devel/setup.bash
    ```
    
How to run a simulation:

1. Run a simulation by executing a launch file in cvg_sim_gazebo package:

    ```
    roslaunch cvg_sim_gazebo ardrone_testworld.launch
    ```
    
    Alternatively, include the above launch file in one of your own custom launch files, like so:
    ```
    <include file="$(find cvg_sim_gazebo)/launch/ardrone_testworld.launch"/>
    ```

How to run a simulation using ar_track_alvar tags:

1. Move the contents of  ~/ardrone_simulator/src/cvg_sim_gazebo/meshes/ar_track_alvar_tags/ to  ~/.gazebo/models

2. Run simulation

    ```
    roslaunch cvg_sim_gazebo ar_tag.launch
    ```
    
## Debugging
If the drone flies upward forever, try installing these hector quadrotor packages:
    ```
    sudo apt-get install ros-kinetic-hector-gazebo ros-kinetic-hector-sensors-gazebo ros-kinetic-hector-xacro-tools
    ```
