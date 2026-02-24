# Git Workflow for Catalyst Manipulation (ROS 2 Jazzy)

## Repository Setup

### Clone the repository

```bash
git clone https://github.com/<org>/catalyst-manipulation.git
cd catalyst-manipulation
```

### Import external dependencies

External packages (xarm_ros2, realsense-ros, apriltag, apriltag_ros) are not stored in this repo. Import them using `vcs`:

```bash
cd ros2_ws/src
vcs import < deps.repos
```

This clones the correct branches/versions into `ros2_ws/src/external/`.

To update external dependencies later:

```bash
cd ros2_ws/src
vcs pull external
```

## Branches

- `main` — stable, production-ready code
- `jazzy_dev` — active development branch for ROS 2 Jazzy

## Day-to-Day Workflow

### Check current status

```bash
git status
git branch
```

### Pull latest changes

```bash
git pull origin jazzy_dev
```

### Stage and commit changes

Stage specific files (preferred):

```bash
git add ros2_ws/src/catalyst_bringup/launch/demo.launch.py
git add ros2_ws/src/catalyst_gripper/catalyst_gripper/gripper_node.py
git commit -m "Fix gripper launch in real mode"
```

Stage all changed files:

```bash
git add -A
git commit -m "Add scene manager collision objects"
```

### Push changes

```bash
git push origin jazzy_dev
```

## Working with Branches

### Create a feature branch

```bash
git checkout jazzy_dev
git pull origin jazzy_dev
git checkout -b feature/add-new-sensor
```

### Switch between branches

```bash
git checkout jazzy_dev
git checkout feature/add-new-sensor
```

### Merge a feature branch into jazzy_dev

```bash
git checkout jazzy_dev
git pull origin jazzy_dev
git merge feature/add-new-sensor
git push origin jazzy_dev
```

### Delete a feature branch after merging

```bash
git branch -d feature/add-new-sensor
git push origin --delete feature/add-new-sensor
```

## Merging jazzy_dev into main

When jazzy_dev is stable and tested:

```bash
git checkout main
git pull origin main
git merge jazzy_dev
git push origin main
```

## Resolving Merge Conflicts

If `git merge` or `git pull` reports conflicts:

1. Check which files have conflicts:

```bash
git status
```

2. Open conflicting files and look for conflict markers:

```
<<<<<<< HEAD
your changes
=======
incoming changes
>>>>>>> branch-name
```

3. Edit the file to keep the correct version, removing the markers.

4. Stage and commit the resolved files:

```bash
git add <resolved-file>
git commit -m "Resolve merge conflict in <file>"
```

## Viewing History

```bash
# Recent commits (one line each)
git log --oneline -10

# Detailed log with file changes
git log --stat -5

# See what changed in a specific commit
git show <commit-hash>

# See diff of uncommitted changes
git diff
```

## Undoing Changes

### Discard uncommitted changes to a file

```bash
git checkout -- <file>
```

### Unstage a file (keep changes, remove from staging)

```bash
git reset HEAD <file>
```

### Undo the last commit (keep changes)

```bash
git reset --soft HEAD~1
```

## Pushing to Multiple Remotes

If you have both a personal repo and an organization repo, you can push to both.

### Add the org repo as a second remote

```bash
git remote add org https://github.com/<org-name>/<repo-name>.git
```

### Push to a new branch on the org remote

```bash
git push org jazzy_dev:jazzy_dev
```

The branch is created automatically on the org repo if it doesn't exist.

### Verify remotes

```bash
git remote -v
```

You'll see two remotes:
- `origin` — your personal repo
- `org` — the organization repo

### Usage going forward

```bash
git push origin jazzy_dev    # push to personal repo
git push org jazzy_dev       # push to org repo
```

### Pull from org remote

```bash
git pull org main            # pull main from org repo
git pull org jazzy_dev       # pull jazzy_dev from org repo
```

## .gitignore

The following are excluded from version control:

- `ros2_ws/build/` — colcon build artifacts
- `ros2_ws/install/` — colcon install artifacts
- `ros2_ws/log/` — colcon log files
- `ros2_ws/src/external/` — cloned via `deps.repos`
- `**/__pycache__/` — Python bytecode
- `**/.vscode/` — editor settings
