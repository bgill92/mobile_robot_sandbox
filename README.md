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

**⚠️ Known Limitation:** Gazebo Classic has a threading issue in conda/pixi environments that causes crashes. See [Hybrid Workflow](#hybrid-workflow-pixi--docker) below for the recommended approach.

If you want to try running Gazebo with pixi (unstable):

```bash
pixi run sim
```

Note: This will likely crash with a `pthread_mutex_lock` error due to library threading conflicts between Gazebo Classic and conda packages. This is a known robostack limitation.

## Hybrid Workflow (Pixi + Docker)

**Recommended approach:** Use pixi for development and Docker for Gazebo simulation.

### Development with Pixi (Fast, Native)

Use pixi for all development work:

```bash
# Build the workspace
pixi run build

# Run ROS2 commands
pixi run ros2 topic list
pixi run ros2 node list

# Interactive development
pixi run source-zsh  # or 'source' for bash
# Now you have full ROS2 environment with native performance
```

**Benefits:**
- ✓ Fast builds (no Docker overhead)
- ✓ Native performance
- ✓ Direct IDE integration
- ✓ Easy access to debugging tools

### Gazebo Simulation with Docker (Stable)

When you need to run Gazebo simulations, use Docker:

```bash
# Build the Docker image (first time only)
export UID=$(id -u)
export GID=$(id -g)
docker compose -f compose.dev.yml build

# Run Gazebo simulation
docker compose -f compose.dev.yml run development
```

Inside the Docker container:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch mobile_robot_algorithms robot_in_maze.launch.py
```

**Why Docker for Gazebo?**
- Gazebo Classic has known threading issues in conda environments
- Docker uses system-installed Gazebo which works reliably
- Only needed when running full simulations, not for general development

### Workflow Summary

| Task | Use | Command |
|------|-----|---------|
| Building code | Pixi | `pixi run build` |
| Running tests | Pixi | `pixi run bash -c 'source install/setup.bash && ...'` |
| ROS2 development | Pixi | `pixi run source-zsh` |
| Gazebo simulation | Docker | `docker compose -f compose.dev.yml run development` |

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

Once in the shell, source the workspace to use ROS2 packages:

**For bash users:**
```bash
source install/setup.bash
```

**For zsh users:**
```zsh
source install/setup.zsh
```

Alternatively, use the pixi tasks that handle sourcing automatically:
```bash
pixi run source      # Opens bash with workspace sourced
pixi run source-zsh  # Opens zsh with workspace sourced
```

After sourcing, you can run any ROS2 commands directly without the `pixi run` prefix.

## Migrating from Docker

This project previously used Docker for environment management. The pixi-based workflow offers:
- Faster startup times (no container overhead)
- Simpler commands (no docker compose complexity)
- Native performance (no virtualization layer)
- Better integration with host tools (editors, debuggers, etc.)

All the same ROS2 packages and tools are available through pixi.