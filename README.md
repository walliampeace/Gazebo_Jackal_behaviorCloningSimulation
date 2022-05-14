# Gazebo_Jackal_behaviorCloningSimulation
A simulator for behavior cloning research with Jackal robots with realsense depth camera
## Environment
Ros: noetic
Gazebo multi-robot simulator, version 11.9.0
## First thing first
Follow the Jackal tutorial [Simulating Jackal](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/simulation.html)
## How to run the Jackal simulation
The following commands will run the simulation with my custom controlling plugin. 
After setting up ROS and Gazebo, run the simulated Jackal in the customize world simulator.world by running:
```bash
roslaunch jackal_gazebo simulator.launch
```
you can drag the Jackal robot wherever you want and the picture is shown below:
![gazebo_simulation2](https://user-images.githubusercontent.com/59054117/168437742-996d71c1-ebee-401c-990b-a741a504eab4.png)

Then run simulator_viz.launch file for visualizing the Jackal model in the Rviz tool.
```bash
roslaunch jackal_viz simulator_viz.launch
```
After that you can first control the Jackal manually by using the interative red arrows and blue circles to move the Jackal. Drag the red arrows in Rviz to move in the linear x and the blue circle to move in the angular z. This is the same as the Jackal tutorial mentioned.

![robot_simulator_viz](https://user-images.githubusercontent.com/59054117/168438747-2105071a-a0b0-43ed-8b63-9444ebc65d2d.png)

Then start the Jackal odom_navigation node and gmapping node for controlling the Jackal automatically.
```bash
roslaunch jackal_navigation odom_navigation_demo.launch
roslaunch jackal_navigation gmapping.launch
```
Finally, you can run the python script to navigate the Jackal to the target position.

```python
python move_simulator.py
```
The picture below shows the simulation demo
![gazebo_with_viz](https://user-images.githubusercontent.com/59054117/168438832-04e82b30-228a-4759-b0bd-f6d2acb06e79.png)
