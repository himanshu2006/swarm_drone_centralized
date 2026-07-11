# 1. Dynamically locate where this script is installed inside the share folder
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 2. Step backward 5 levels to safely reach your absolute workspace root (~/swarm_ws)
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

# 3. Export the absolute paths directly to Gazebo
export GZ_SIM_RESOURCE_PATH="${WS_ROOT}/src/swarm_model:${GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="${WS_ROOT}/src/swarm_model:${IGN_GAZEBO_RESOURCE_PATH}"