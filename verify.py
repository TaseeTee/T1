import json
import subprocess
import sys
from time import monotonic


def docker(*args, timeout=30):
    return subprocess.run(
        ["docker", *args], check=True, capture_output=True, text=True,
        timeout=timeout,
    ).stdout


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify(image):
    platform = docker(
        "image", "inspect", "--format", "{{.Os}}/{{.Architecture}}", image,
    ).strip()
    require(platform == "linux/amd64", f"Expected linux/amd64, got {platform}")

    start = monotonic()
    output = docker(
        "run", "--pull=never", "--rm", "--network", "none",
        "--entrypoint", "/entrypoint.sh", image, "ping", timeout=10,
    )
    elapsed = monotonic() - start
    payload = json.loads(output)
    require(isinstance(payload, dict), "Expected one JSON object")
    require(payload.get("protocol_version") == "1.0", "Invalid protocol version")
    tasks = payload.get("supported_tasks")
    require(isinstance(tasks, list), "Expected a supported_tasks array")
    require(all(task in ("T2", "T3", "T4", "T5") for task in tasks),
            "Unexpected task identifier")
    require(len(set(tasks)) == len(tasks), "Duplicate task identifier")

    trained = payload.get("self_trained_rl")
    require(type(trained) is bool, "Expected a self_trained_rl boolean")
    require("preferred_speed_mps" in payload, "Missing preferred_speed_mps")
    speed = payload["preferred_speed_mps"]
    if "T3" in tasks:
        require(type(speed) in (int, float) and 0 < speed < float("inf"),
                "Expected a finite positive speed for T3")
    else:
        require(speed is None and not trained, "Invalid metadata without T3")

    uid, os_version, rmw = docker(
        "run", "--pull=never", "--rm", "--network", "none",
        "--entrypoint", "/bin/bash", image, "-ec",
        'id -u; . /etc/os-release; printf "%s\\n" "$ID:$VERSION_ID" "$RMW_IMPLEMENTATION"; '
        'test -f /opt/ros/humble/share/rmw_cyclonedds_cpp/package.xml',
    ).splitlines()
    require(uid != "0", "Container runs as root")
    require(os_version == "ubuntu:22.04", f"Unexpected OS {os_version}")
    require(rmw == "rmw_cyclonedds_cpp", f"Unexpected RMW {rmw}")
    print(f"5-HT2A COLLECTIVE | offline ping OK in {elapsed:.3f}s | {platform}")
    print(f"Tasks: {', '.join(tasks) or 'none'} | UID: {uid} | DDS: {rmw}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python3 verify.py IMAGE")
    try:
        verify(sys.argv[1])
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.stderr.strip() or str(error)) from error
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        raise SystemExit(str(error)) from error
