# Isaac Lab on the GB10 (DGX Spark, aarch64)

Target training machine: `asus@gx10-f3fb` — NVIDIA **GB10** (Grace-Blackwell),
aarch64, Ubuntu 24.04, driver **580.142**, 121 GB RAM, 20 cores, 620 GB free.

## Verdict: supported, via SOURCE build (not pip, not the NGC container)

On GB10 (compute capability 12.1 / CUDA 13) the pip wheels and the
`nvcr.io/nvidia/isaac-lab` container do **not** work — you must build Isaac Sim
from source. This is NVIDIA/Arm's documented path for DGX Spark (Isaac Sim 5.1).

Refs:
- Arm Learning Path — Isaac install on DGX Spark
- Arm Learning Path — Train a locomotion policy on DGX Spark
- build.nvidia.com/spark/isaac
- Isaac Sim 5.1.0 requirements; IsaacLab GitHub

## Constraints / caveats (aarch64)

- **Build from source**, gcc-11, CUDA 13, driver ≥580, always `--headless`.
- Use the `libgomp` LD_PRELOAD workaround (baked into `remote/env.sh`).
- Unsupported on aarch64 (not needed for walking): OpenXR/teleop, cuRobo/SkillGen,
  SKRL-with-JAX. Use **RSL-RL PPO** (the validated library on Spark).
- Live streaming is unsupported on aarch64 — train headless, inspect with `play.py`.

## Steps (scripts in `training/remote/`)

```bash
# 1) ON the GB10 — install (needs the `asus` account's sudo password for apt):
ssh asus@gx10-f3fb 'bash -s' < training/remote/setup_gb10.sh
# 2) FROM the laptop — push model + task package:
bash training/remote/sync.sh
# 3) ON the GB10 — validate stock task FIRST, then train ours:
ssh asus@gx10-f3fb 'bash ~/robotdog/training/remote/train.sh'
# 4) export + pull back policy.onnx into policy/
```

## ⚠️ Current blocker

`setup_gb10.sh` needs `sudo apt-get install gcc-11 git-lfs` on the GB10. The
remote `asus` account's sudo password was **not** available in this session (the
`Abhitej@123` password is for the local laptop, and was rejected on `asus@`).
`git-lfs` is missing and gcc is 13 (Isaac wants 11). Provide the `asus` sudo
password (or pre-install git-lfs + gcc-11) and `setup_gb10.sh` will complete.

Everything else is ready: the task package (`training/isaaclab_task/`) is synced to
`~/robotdog` on the GB10 and compiles there; `sync.sh` is verified working.

## Day-1 risk retirement

Before training the custom robot, confirm the RL stack works on the stock task:
```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-D-v0 --headless --max_iterations 50
```
If that produces a checkpoint, then `--task Isaac-Velocity-Flat-RobotDog-v0`.

## Don't want to wait on the GB10 build?

`training/mujoco_fallback/train_ppo.py` trains the **same** policy (identical
obs/action layout) on CPU anywhere — the artifacts are interchangeable. This repo's
walking demo and policy export were validated through that path.
