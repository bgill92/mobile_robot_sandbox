# mobile_robot_sandbox

This repo is supposed to be a sandbox where I can experiment with different control/perception algorithms that can be used with a mobile base.
I am using the [andino](https://github.com/Ekumen-OS/andino) robot as the mobile platform.

## Prerequisites

This project uses [pixi](https://pixi.sh) for managing dependencies and the ROS2 environment. Pixi is a modern, fast package manager that simplifies environment management.

### Installing pixi

Install pixi by running:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

After installation, restart your shell or run:

```bash
source ~/.bashrc  # or ~/.zshrc if using zsh
```

For more installation options, visit [pixi.sh](https://pixi.sh).

## Setup

1. Make sure that the `andino` submodule is pulled:
```bash
git submodule update --init --recursive
```

2. Install all dependencies with pixi:
```bash
pixi install
```

This will create a `.pixi` directory with all ROS2 Humble packages, Gazebo Classic, and development tools.

3. (First-time setup) Exclude hardware-specific packages for simulation:
```bash
touch src/andino/andino_base/COLCON_IGNORE
touch src/andino/andino_firmware/COLCON_IGNORE
touch src/andino/andino_hardware/COLCON_IGNORE
```

These packages require hardware-specific dependencies (like libserial) that aren't needed for Gazebo simulation.

## Building the Workspace

Build the ROS2 workspace:

```bash
pixi run build
```

This command runs `colcon build --symlink-install` with the proper ROS2 environment activated.

## Launching the Gazebo Simulation

To run the robot in maze simulation:

```bash
pixi run sim
```

This will launch the andino robot inside a maze using Gazebo Classic.

## Additional Commands

Run any ROS2 command in the pixi environment:

```bash
pixi run ros2 topic list
pixi run ros2 node list
```

Clean build artifacts:

```bash
pixi run clean
```

Enter the pixi shell for interactive development:

```bash
pixi shell
```

Once in the shell, you can run any ROS2 commands directly without the `pixi run` prefix.

## Migrating from Docker

This project previously used Docker for environment management. The pixi-based workflow offers:
- Faster startup times (no container overhead)
- Simpler commands (no docker compose complexity)
- Native performance (no virtualization layer)
- Better integration with host tools (editors, debuggers, etc.)

All the same ROS2 packages and tools are available through pixi.