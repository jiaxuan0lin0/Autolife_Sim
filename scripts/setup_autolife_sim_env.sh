#!/usr/bin/env bash
set -euo pipefail

environment_name="${AUTOLIFE_SIM_ENV_NAME:-autolife_sim}"
expected_behavior_commit="8579326f8a9719fe7a261f69ab0f27d545ac38a9"
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dependency_root="${AUTOLIFE_DEPENDENCY_ROOT:-$repository_root/.deps}"
behavior_root="${AUTOLIFE_BEHAVIOR_ROOT:-$dependency_root/BEHAVIOR-1K}"
requirements_file="$repository_root/requirements/autolife_sim.txt"
accept_licenses=false

for argument in "$@"; do
    case "$argument" in
        --accept-licenses)
            accept_licenses=true
            ;;
        -h|--help)
            echo "Usage: scripts/setup_autolife_sim_env.sh [--accept-licenses]"
            echo ""
            echo "--accept-licenses  Pass non-interactive acceptance flags to the"
            echo "                   NVIDIA and BEHAVIOR upstream installer."
            exit 0
            ;;
        *)
            echo "Unknown argument: $argument" >&2
            exit 2
            ;;
    esac
done

initialize_conda() {
    if [[ "$(type -t conda 2>/dev/null || true)" == "function" ]]; then
        return
    fi
    if command -v conda >/dev/null 2>&1; then
        eval "$(conda shell.bash hook)"
        return
    fi
    if [[ -n "${CONDA_EXE:-}" && -x "$CONDA_EXE" ]]; then
        eval "$("$CONDA_EXE" shell.bash hook)"
        return
    fi
    echo "Conda was not found. Install Miniconda or Miniforge and retry." >&2
    exit 1
}

environment_exists() {
    conda env list | awk 'NF >= 2 && $1 !~ /^#/ {print $1}' | grep -Fqx "$1"
}

prepare_behavior_checkout() {
    mkdir -p "$dependency_root"
    if [[ ! -d "$behavior_root/.git" ]]; then
        echo "Cloning the pinned BEHAVIOR-1K source into .deps/BEHAVIOR-1K ..."
        git init "$behavior_root"
        git -C "$behavior_root" remote add origin https://github.com/StanfordVL/BEHAVIOR-1K.git
    fi

    local actual_commit=""
    actual_commit="$(git -C "$behavior_root" rev-parse HEAD 2>/dev/null || true)"
    if [[ "$actual_commit" == "$expected_behavior_commit" ]]; then
        return
    fi
    if [[ -n "$(git -C "$behavior_root" status --porcelain 2>/dev/null)" ]]; then
        echo "Cannot switch a modified BEHAVIOR-1K checkout: $behavior_root" >&2
        exit 1
    fi
    git -C "$behavior_root" fetch --depth 1 origin "$expected_behavior_commit"
    git -C "$behavior_root" checkout --detach "$expected_behavior_commit"
}

initialize_conda

if ! environment_exists "$environment_name"; then
    source_environment="${AUTOLIFE_SIM_CLONE_FROM:-}"
    if [[ -n "$source_environment" ]] && ! environment_exists "$source_environment"; then
        echo "Clone source environment does not exist: $source_environment" >&2
        exit 1
    fi

    if [[ -n "$source_environment" ]]; then
        echo "Creating $environment_name from the existing $source_environment environment ..."
        conda create --name "$environment_name" --clone "$source_environment" --yes
    else
        prepare_behavior_checkout
        echo "Installing the pinned BEHAVIOR / OmniGibson stack ..."
        upstream_arguments=(
            --new-env "$environment_name"
            --omnigibson
            --bddl
            --dataset
        )
        if [[ "$accept_licenses" == true ]]; then
            upstream_arguments+=(
                --accept-conda-tos
                --accept-nvidia-eula
                --accept-dataset-tos
            )
        fi
        (
            cd "$behavior_root"
            ./setup.sh "${upstream_arguments[@]}"
        )
    fi
fi

# The BEHAVIOR environment supplies Isaac Sim, OmniGibson, BDDL, and their
# binary dependencies. This small overlay adds the pinned MolmoSpaces manager
# without allowing pip to replace Isaac Sim's Click version.
conda run --name "$environment_name" \
    python -m pip install --no-deps --requirement "$requirements_file"

conda run --no-capture-output --name "$environment_name" python -c '
import importlib.metadata as metadata

expected = {
    "isaacsim": "5.1.0.0",
    "omnigibson": "3.8.0",
    "bddl": "3.7.0",
    "molmospaces_resources": "0.0.1b4",
    "click": "8.1.7",
    "typer": "0.20.0",
}
actual = {name: metadata.version(name) for name in expected}
if actual != expected:
    raise SystemExit(f"Version mismatch: expected={expected}, actual={actual}")
print("autolife_sim environment is ready:", actual)
'

echo "Repository: $repository_root"
echo "Activate:   source scripts/activate_autolife_sim.sh"
