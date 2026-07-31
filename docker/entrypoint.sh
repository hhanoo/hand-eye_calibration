#!/bin/bash
set -e

# ===== Workspace =====
WORKSPACE="/ros2_ws"

# ===== Functions =====
restore_ownership() {
    # Restore host file ownership
    if [ -n "$HOST_UID" ] && [ -n "$HOST_GID" ] && [ -d "$WORKSPACE" ]; then
        chown -R "$HOST_UID:$HOST_GID" "$WORKSPACE" 2>/dev/null || true
    fi
}

# ===== Trap =====
trap restore_ownership EXIT

# ===== RMW / DDS configuration (CycloneDDS for large sensor messages) =====
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f /etc/cyclonedds.xml ]; then
    export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
fi

# ===== Default Command =====
if [ "$#" -eq 0 ]; then
    set -- /bin/bash
fi

# ===== Execute Command =====
"$@"
