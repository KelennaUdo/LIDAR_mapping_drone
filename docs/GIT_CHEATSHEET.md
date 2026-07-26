# Git Command Cheat Sheet

## Mental Model

```text
Working tree
    ↓ git add
Staging area
    ↓ git commit
Local branch history
    ↓ git push
Remote branch on GitHub
```

`git commit` saves a snapshot locally. It does not upload anything.
`git push` uploads local commits to a remote repository such as GitHub.

## Inspect Before Changing Anything

```bash
# Show the current branch and changed, staged, or untracked files.
git status

# Show the same information in compact form.
git status --short --branch

# Show unstaged line-by-line changes.
git diff

# Show changes already added to the staging area.
git diff --staged

# Check for whitespace errors in current changes.
git diff --check

# Show recent commits as a compact graph.
git log --oneline --graph --decorate -15

# Show one commit and its patch.
git show COMMIT_ID

# Show only the files and size summary for one commit.
git show --stat COMMIT_ID
```

## Branches

```bash
# List local branches and mark the current one with an asterisk.
git branch

# Include remote-tracking branches.
git branch --all

# Create a new branch and switch to it.
git switch -c feature/example-name

# Switch to an existing branch.
git switch feature/px4-sitl

# Show commits that differ between two branches.
git log --oneline feature/telemetry-sensors..feature/px4-sitl

# Show a summary of file differences between two branches.
git diff --stat feature/telemetry-sensors...feature/px4-sitl
```

Commit or stash unfinished work before switching when the same files differ
between branches.

## Stage and Commit

```bash
# Stage one specific file.
git add path/to/file

# Stage several reviewed files explicitly.
git add README.md PX4_SETUP.md

# Review exactly what will be committed.
git diff --staged

# Create a local commit with a concise description.
git commit -m "Describe the completed change"

# Confirm that the working tree is clean after committing.
git status
```

Prefer explicit file paths over `git add .` when unrelated work may be present.

## Remotes and GitHub

```bash
# Show configured fetch and push URLs.
git remote -v

# Download remote branch information without modifying local files.
git fetch origin

# Push the current branch and establish its upstream relationship.
git push -u origin feature/px4-sitl

# Later pushes can use the established upstream.
git push

# Show whether the local branch is ahead of or behind its upstream.
git status --short --branch
```

An HTTPS remote may request a GitHub personal access token. GitHub account
passwords are not accepted as Git credentials.

## Pulling Remote Work

```bash
# Download and integrate the current branch's upstream changes.
git pull
```

Before pulling, use `git status` and commit or stash local work. `git fetch`
is a safer inspection step when you are not ready to integrate changes.

## Stash Temporary Work

```bash
# Temporarily store tracked and untracked changes with a description.
git stash push --include-untracked -m "Temporary work"

# List saved stashes.
git stash list

# Reapply the newest stash and remove it from the stash list.
git stash pop
```

Always run `git status` after `stash pop`; overlapping changes can produce
merge conflicts.

## Undo Staging Without Deleting Edits

```bash
# Remove a file from the staging area but keep its working-tree edits.
git restore --staged path/to/file
```

The following command discards uncommitted edits in the selected file:

```bash
# WARNING: this permanently discards current unstaged edits in the file.
git restore path/to/file
```

Review `git diff` first and do not use it when changes should be preserved.

## Commands Requiring Special Care

Do not use these as routine fixes:

```text
git reset --hard
git clean -fd
git push --force
```

They can delete local work or rewrite shared history. Diagnose the repository
state first and use a targeted recovery method.

## Project Branches

```text
feature/telemetry-sensors  X3 custom-controller and telemetry sandbox
feature/px4-sitl           PX4 SITL and supported Gazebo X500
```

Use `git status` before switching between them.
