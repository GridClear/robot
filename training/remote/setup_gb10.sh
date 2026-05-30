#!/usr/bin/env bash
# Install Isaac Sim + Isaac Lab from SOURCE on the DGX Spark / GB10 (aarch64).
# This is the SUPPORTED path on GB10 — the pip wheels / NGC container do not work
# on aarch64+CUDA13 (compute capability 12.1). Refs: Arm DGX Spark Isaac learning
# path + build.nvidia.com/spark/isaac.
#
# Run ON the GB10:  ssh asus@gx10-f3fb 'bash -s' < training/remote/setup_gb10.sh
# Needs ~50 GB disk and 10-15 min. Idempotent-ish (skips clones that exist).
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaac}"
mkdir -p "$ISAAC_ROOT"; cd "$ISAAC_ROOT"

echo ">>> [1/5] prerequisites (gcc-11, git-lfs)"
sudo apt-get update -y
sudo apt-get install -y gcc-11 g++-11 git git-lfs build-essential cmake
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
git lfs install

echo ">>> [2/5] clone + build Isaac Sim (aarch64 release)"
if [ ! -d IsaacSim ]; then
  git clone --depth=1 --recursive https://github.com/isaac-sim/IsaacSim
fi
cd IsaacSim
git lfs pull || true
./build.sh
export ISAACSIM_PATH="$PWD/_build/linux-aarch64/release"
cd "$ISAAC_ROOT"

echo ">>> [3/5] clone Isaac Lab + link Isaac Sim"
if [ ! -d IsaacLab ]; then
  git clone --recursive https://github.com/isaac-sim/IsaacLab
fi
cd IsaacLab
ln -sfn "$ISAACSIM_PATH" "$PWD/_isaac_sim"

echo ">>> [4/5] install Isaac Lab (creates its conda/venv + rsl_rl)"
./isaaclab.sh --install rsl_rl

echo ">>> [5/5] persist env for train.sh"
cat > "$ISAAC_ROOT/env.sh" <<EOF
export ISAACSIM_PATH="$ISAACSIM_PATH"
export ISAACLAB_PATH="$ISAAC_ROOT/IsaacLab"
# aarch64 workaround: preload libgomp so PhysX/torch threading initializes
export LD_PRELOAD="\${LD_PRELOAD:-}:/lib/aarch64-linux-gnu/libgomp.so.1"
EOF

echo ">>> DONE. Validate with the stock task before the custom one:"
echo "    source $ISAAC_ROOT/env.sh"
echo "    cd \$ISAACLAB_PATH && ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \\"
echo "        --task Isaac-Velocity-Flat-Anymal-D-v0 --headless --max_iterations 50"
