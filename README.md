# Carla_simulation_waypoint_follower

The script vehicle_control_strategy.py is used to test control and planning algorithms for autonomous vehicles. You can run script and choose a random waypoint
as your goal location or you can specify a location (lines 46-52). To indicate you want to choose a random waypoint as your goal location, do the following:

python vehicle_control_strategy.py 1

The system argument will be used as a comparison in an if statement to determine whether the script should use a random waypoint or a specified location. It is
important to mention that the PID values for the longitudinal and lateral controller are not tuned. You can change these parameters and see how your vehicle will
response.

Below is an image of the script working.

![alt text](https://github.com/Johan579/Carla_simulation_waypoint_follower/tree/master/Carla_Images)
