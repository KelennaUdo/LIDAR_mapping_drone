# Command Cheat Sheets

These notes are quick references for the tools used by the project. They are
not scripts: run commands individually and read the comment above each command
before using it.

## Start Here

| Cheat sheet | Use it when |
| --- | --- |
| [Docker](DOCKER_CHEATSHEET.md) | Inspecting images and containers, checking storage, or stopping PX4 |
| [ROS 2](ROS2_CHEATSHEET.md) | Building a workspace or inspecting nodes, topics, TF, and bags |
| [Git](GIT_CHEATSHEET.md) | Reviewing, committing, pushing, or changing branches |
| [PX4 SITL](PX4_SITL_CHEATSHEET.md) | Starting and stopping the project's PX4 X500 simulation |
| [Linux Storage](LINUX_STORAGE_CHEATSHEET.md) | Mounting the external PX4 workspace or checking disk usage |

The normal external-drive workflow uses one project helper:

```bash
./scripts/px4_workspace.sh connect
./scripts/px4_workspace.sh status
./scripts/px4_workspace.sh disconnect
```

## Project Mental Model

```text
Ubuntu 26.04 host
├── Git repository
│   └── /home/kelenna-udo/LIDAR_mapping_drone
├── Docker
│   └── px4-sitl:v1.17.0 (Ubuntu 24.04 dependency image)
├── NVIDIA Container Toolkit
│   └── Gives Gazebo access to the RTX 4050
└── External-drive ext4 image
    └── /mnt/px4-workspace/PX4-Autopilot
        ├── PX4 source
        └── PX4 build output
```

The X3 custom-controller sandbox is preserved on
`feature/telemetry-sensors`. The PX4 X500 workflow is on `feature/px4-sitl`.

## Prompt Symbols

Examples do not include a shell prompt. Enter only the command, not `$`, `>`,
or the text printed by an earlier command.

Lines beginning with `#` are explanatory comments:

```bash
# This line explains the command below. Bash does not execute it.
git status
```

## Safety Rule

Inspection commands such as `git status`, `docker ps`, `df`, and
`ros2 topic list` are normally read-only. Commands containing `rm`, `prune`,
`reset --hard`, `clean -fd`, `--force`, or filesystem formatting options can
destroy work and are intentionally not part of the normal workflows here.
