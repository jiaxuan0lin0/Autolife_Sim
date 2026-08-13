#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Source this file instead of executing it:" >&2
    echo "  source scripts/activate_autolife_sim.sh" >&2
    exit 1
fi

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
environment_name="${AUTOLIFE_SIM_ENV_NAME:-autolife_sim}"

if [[ "$(type -t conda 2>/dev/null || true)" != "function" ]]; then
    if command -v conda >/dev/null 2>&1; then
        eval "$(conda shell.bash hook)"
    elif [[ -n "${CONDA_EXE:-}" && -x "$CONDA_EXE" ]]; then
        eval "$("$CONDA_EXE" shell.bash hook)"
    else
        echo "Conda was not found. Install Miniconda or Miniforge first." >&2
        return 1
    fi
fi

conda activate "$environment_name" || return 1

export AUTOLIFE_SIM_ROOT="$repository_root"
export AUTOLIFE_MOLMOSPACE_CACHE="${AUTOLIFE_MOLMOSPACE_CACHE:-$repository_root/.cache/molmospaces}"
if [[ -d "$repository_root/.deps/BEHAVIOR-1K" ]]; then
    export AUTOLIFE_BEHAVIOR_ROOT="${AUTOLIFE_BEHAVIOR_ROOT:-$repository_root/.deps/BEHAVIOR-1K}"
fi

# Isaac Sim uses Python 3.11; system ROS 2 Jazzy uses Python 3.12. Keep the
# system Python path out of Kit and use Isaac Sim's bundled ROS bridge.
unset PYTHONPATH
export ROS_DISTRO="jazzy"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"

bridge_lib="$(python - <<'PY'
import sys
from pathlib import Path

print(Path(sys.prefix) / "lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib")
PY
)"
if [[ ! -d "$bridge_lib" ]]; then
    echo "Isaac Sim bundled Jazzy bridge was not found: $bridge_lib" >&2
    return 1
fi

case ":${LD_LIBRARY_PATH:-}:" in
    *":$bridge_lib:"*) ;;
    *) export LD_LIBRARY_PATH="$bridge_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac

echo "Activated autolife_sim (Python 3.11, Isaac Sim, OmniGibson, MolmoSpaces)."
echo "Repository root: $AUTOLIFE_SIM_ROOT"
