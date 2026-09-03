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

## Git Safety and User Support

Assume the user is still developing confidence with Git. Treat protecting the
working tree and reviewing commit contents as part of the agent's job, rather
than expecting the user to notice subtle staging or ignore-rule problems.

1. Before any commit or push, inspect `git status --short` and the complete
   staged file list with `git diff --cached --name-status`.
2. Prefer staging explicit paths when unrelated, generated, personal, or
   untracked files are present. Do not recommend `git add .` without first
   explaining exactly what it would stage.
3. Verify important ignore rules with `git check-ignore -v <path>`. Clearly
   warn the user when a rule is malformed or does not match; merely leaving the
   file out of one commit is not sufficient protection.
4. Remember that `.gitignore` patterns are repository-relative, not absolute
   filesystem paths, and that adding an ignore rule does not untrack a file
   Git already tracks.
5. Never stage or commit a local-only file unless the user explicitly confirms
   it belongs in the repository. Use `.git/info/exclude` when an ignore rule
   should remain private to this checkout.
6. Before pushing, summarize any new, deleted, renamed, or unexpectedly staged
   files. If potentially sensitive content appears, stop and warn the user
   before it enters published history.
7. When removing an accidentally tracked local file, explain that
   `git rm --cached` removes it from Git while preserving the local copy, and
   that a normal follow-up commit does not erase the file from older history.
