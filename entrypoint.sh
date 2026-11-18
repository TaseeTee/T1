#!/bin/bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$project_dir" == / ]]; then
    project_dir=/opt/participant
fi

case "${1:-}" in
    ping)
        exec python3 "$project_dir/manifest.py"
        ;;
    T2|T3|T4|T5)
        launcher="$project_dir/tasks/$1"
        if [[ ! -f "$launcher" || ! -x "$launcher" ]]; then
            printf 'Task %s is not available\n' "$1" >&2
            exit 64
        fi
        shift
        source /opt/ros/humble/setup.bash
        exec "$launcher" "$@"
        ;;
    *)
        printf 'Usage: %s {ping|T2|T3|T4|T5}\n' "$0" >&2
        exit 64
        ;;
esac
