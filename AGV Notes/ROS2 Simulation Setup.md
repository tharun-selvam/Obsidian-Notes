- [Simulation Repo](https://github.com/f1tenth/f1tenth_gym_ros#topics-published-by-the-simulation)

## Steps to start the containers 

1. Start Docker.app on your Mac
2. `cd f1tenth_gym_ros` 
3. Run the following command
```
$ docker-compose up
```
4. Open a new terminal window and `cd f1tenth_gym_ros`. This opens an interactive container containing the sim
```
$ docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```
5. Run the following command in this terminal to start the sim
```
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
6. Open this link http://localhost:8080/vnc.html in your browser.

## Controlling the car
### Keyboard_teleop
1. The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. To enable keyboard teleop, set `kb_teleop` to `True` in `sim.yaml`. After launching the simulation, in another terminal, run:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Adding new packages to the container
1. Edit the file `docker-compose.yml` by adding the below line at the right place
```
volumes:
 - .:/sim_ws/src/f1tenth_gym_ros (already present)
 - <path_to_your_package_on_host>:/sim_ws/src/safety_node (new line)
```
2. We can just edit the code on the <path_to_your_package_on_host> and it gets reflected in the container.
3. Basically this code sets a folder in our system to the path `/sim_ws/src/safety_node` 
## About the simulation

### Topics published by the simulation
In **single** agent:

- `/scan`: The ego agent's laser scan
- `/ego_racecar/odom`: The ego agent's odometry
- `/map`: The map of the environment
- A `tf` tree is also maintained.

In **two** agents:
In addition to the topics available in the single agent scenario, these topics are also available:

- `/opp_scan`: The opponent agent's laser scan
- `/ego_racecar/opp_odom`: The opponent agent's odometry for the ego agent's planner
- `/opp_racecar/odom`: The opponent agents' odometry
- `/opp_racecar/opp_odom`: The ego agent's odometry for the opponent agent's planner

### Topics subscribed by the simulation
In **single** agent:

- `/drive`: The ego agent's drive command via `AckermannDriveStamped` messages
- `/initalpose`: This is the topic for resetting the ego's pose via RViz's 2D Pose Estimate tool. Do **NOT** publish directly to this topic unless you know what you're doing.
TODO: kb teleop topics

In **two** agents:
In addition to all topics in the single agent scenario, these topics are also available:

- `/opp_drive`: The opponent agent's drive command via `AckermannDriveStamped` messages. Note that you'll need to publish to **both** the ego's drive topic and the opponent's drive topic for the cars to move when using 2 agents.

- `/goal_pose`: This is the topic for resetting the opponent agent's pose via RViz's 2D Goal Pose tool. Do **NOT** publish directly to this topic unless you know what you're doing.