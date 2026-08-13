# Project Rules

1. Do not make any changes to this project's code without first explaining the proposed changes to the user and receiving an explicit message containing exactly "Approved".
2. When explaining concepts, use the idea of mental models to explain how things fit into the larger scope. Connect each explanation to a wider, expanding mental model of how this project's system works.
3. When adding launch support for a package or system workflow, include both a ROS launch file and a companion `.sh` script.


## Learning-First Agent Workflow

1. Explain the purpose of every major tool and component before using it.
2. Define unfamiliar terms in plain language.
3. Show the exact command before running it and explain what it should do.
4. Work in small checkpoints instead of completing the entire setup at once. Stop after each checkpoint and wait for the user's approval before continuing.
5. Never perform system installations, destructive commands, large refactors, or major architecture changes without explicit approval.
6. After every step, explain what changed, why it changed, how to verify it, what successful output should look like, and how to undo it if necessary.
7. Do not hide errors or silently try many fixes. Explain the diagnostic reasoning before attempting a fix.
8. Do not generate large scripts or configuration files without walking the user through their main sections.
9. Treat the goal as helping the user learn PX4, not merely making the drone fly.

## Code Readability and Organization Standard

When writing or refactoring project code, prioritize readability, learning, and clear visual organization.

Use the following file as the primary reference for the preferred code organization, commenting style, section headers, spacing, and overall readability:

`/home/kelenna-udo/LIDAR_mapping_drone/src/px4_offboard_control/src/offboard_control.cpp`

New code should generally match the style demonstrated in that file, especially:

* clear visual section headers;
* class state and long-lived objects presented before the behavior that uses them;
* logical grouping of related variables, objects, and functions;
* short one-line comments above important variables, objects, and functions explaining their purpose and relationship to the surrounding system;
* descriptive variable and function names;
* enough whitespace to make major sections easy to scan;
* comments that explain intent and relationships rather than simply restating syntax.

Do not copy the reference file's exact structure when it does not fit the code being written. Adapt the same readability principles to the type of program, language, and architecture involved.

Avoid both extremes:

* code that is so sparsely commented that its purpose is difficult to understand;
* code that is overwhelmed by comments explaining every obvious line.

The goal is for generated code to have the same clean, structured, learning-friendly feel as the reference file.

When reorganizing existing code for readability, do not change its behavior unless the behavioral change has been separately explained and approved.
