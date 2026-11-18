import json
import os
from pathlib import Path

T3_SPEED_MPS = None
T3_SELF_TRAINED = False

tasks_dir = Path(__file__).resolve().parent / "tasks"
tasks = []
for name in ("T2", "T3", "T4", "T5"):
    launcher = tasks_dir / name
    if launcher.is_file() and os.access(launcher, os.X_OK):
        tasks.append(name)

if "T3" in tasks and T3_SPEED_MPS is None:
    raise SystemExit("Set T3_SPEED_MPS before adding the T3 launcher")

print(json.dumps({
    "protocol_version": "1.0",
    "supported_tasks": tasks,
    "self_trained_rl": "T3" in tasks and T3_SELF_TRAINED,
    "preferred_speed_mps": T3_SPEED_MPS if "T3" in tasks else None,
}))
