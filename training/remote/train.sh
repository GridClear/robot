#!/usr/bin/env bash
# Train the robot dog locomotion policy in Isaac Lab ON the GB10.
# Run ON the GB10:  ssh asus@gx10-f3fb 'bash ~/robotdog/training/remote/train.sh'
# (after setup_gb10.sh + sync.sh).
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaac}"
DEST="${ROBOTDOG_REMOTE_DIR:-$HOME/robotdog}"
TASK="${TASK:-Isaac-Velocity-Flat-RobotDog-v0}"
ITERS="${ITERS:-1500}"

# TERM=xterm avoids "ansi+tabs: unknown terminal type" tripping set -e under nohup.
export TERM="${TERM:-xterm}"
source "$ISAAC_ROOT/env.sh"
# make the custom task package importable so its gym.register runs
export PYTHONPATH="$DEST/training:${PYTHONPATH:-}"

cd "$ISAACLAB_PATH"
# Use the robot-dog launcher (train_robotdog.py imports isaaclab_task so the
# custom gym ids register); the stock train.py does NOT import the task package.
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train_robotdog.py \
  --task "$TASK" --headless --max_iterations "$ITERS"

echo ">>> training done. Export the checkpoint to ONNX:"
echo "    ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play_robotdog.py \\"
echo "        --task $TASK --num_envs 8 --checkpoint <run>/model_*.pt"
echo ">>> Then pull logs/rsl_rl/robot_dog_flat/<run>/exported/policy.onnx back to policy/."
