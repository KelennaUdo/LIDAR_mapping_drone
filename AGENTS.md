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
