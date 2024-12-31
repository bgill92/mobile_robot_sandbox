# mobile_robot_sandbox

This repo is supposed to be a sandbox where I can experiment with different control/perception algorithms that can be used with a mobile base.
I am using the [andino](https://github.com/Ekumen-OS/andino) robot as the mobile platform.

## Docker instructions

Instructions for setting up the Docker container can be found/adapted from [here](https://github.com/bgill92/physics_sandbox?tab=readme-ov-file#dockerfile-information).

## Launching the Gazebo Simulation

To run the Gazebo simulation:

2) In a terminal, run:
```bash
export UID=$(id -u) export GID=$(id -g); docker compose -f compose.dev.yml run development
```

Make sure that you have build the Docker Container via the following command:
```bash
export UID=$(id -u); export GID=$(id -g); docker compose -f compose.dev.yml build
```

3) When in the Docker container, build the packages via:
```bash
colcon build --symlink-install
```

4) Source the workspace:
```bash
source install/setup.bash
```

5) Run:
```bash
ros2 launch mobile_robot_algorithms robot_in_maze.launch.py
```

This should launch the andino robot inside of a maze.