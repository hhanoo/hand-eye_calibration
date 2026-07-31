#!/bin/bash

# Directories & Variables
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load common variables (auto-copy from example if not exists)
if [ ! -f "$SCRIPT_DIR/config.sh" ]; then
    cp "$SCRIPT_DIR/config.sh.example" "$SCRIPT_DIR/config.sh"
fi
source "$SCRIPT_DIR/config.sh"

# Check if the image exists
if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo "Error: Image $IMAGE_NAME not found."
    echo "Pull the image first: docker pull $IMAGE_NAME"
    echo "Or build locally: ./build.sh"
    exit 1
fi

# [1/3] Host kernel tuning for ROS2 / CycloneDDS large sensor messages
echo "==> [1/3] Tuning host kernel (rmem/wmem, ipfrag) for ROS2 DDS..."
sudo sysctl -qw net.core.rmem_max=67108864
sudo sysctl -qw net.core.rmem_default=67108864
sudo sysctl -qw net.core.wmem_max=67108864
sudo sysctl -qw net.core.wmem_default=67108864
sudo sysctl -qw net.ipv4.ipfrag_time=3
sudo sysctl -qw net.ipv4.ipfrag_high_thresh=134217728

# [2/3] Enable X11 access for Docker
echo "==> [2/3] Enabling X11 access for Docker (xhost +local:docker)..."
xhost +local:docker > /dev/null 2>&1

# [3/3] Run the Docker container
echo "==> [3/3] Checking container '$CONTAINER_NAME'..."

# Reuse existing container if present; otherwise create a new one
if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then

    # Start the container if it is stopped
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" = "false" ]; then
        echo "--> Container exists but is stopped. Starting '$CONTAINER_NAME'..."
        docker start "$CONTAINER_NAME" > /dev/null
    fi

    # Attach a new shell to the running container
    echo "--> Attaching to running container '$CONTAINER_NAME'..."
    echo "---------- container output ----------"
    echo
    docker exec -it \
        -e HOST_UID="$(id -u)" \
        -e HOST_GID="$(id -g)" \
        "$CONTAINER_NAME" \
        /entrypoint.sh /bin/bash

else
    # No existing container: create and run a new one
    echo "--> Container '$CONTAINER_NAME' not found. Creating a new one..."
    echo "---------- container output ----------"
    echo
    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        --privileged \
        --network host \
        --ipc=host \
        --gpus all \
        \
        -e DISPLAY="$DISPLAY" \
        -e QT_X11_NO_MITSHM=1 \
        -e HOST_UID="$(id -u)" \
        -e HOST_GID="$(id -g)" \
        -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        -e XAUTHORITY=/root/.Xauthority \
        \
        -v "$ROS2_WS_ROOT:/ros2_ws" \
        -v /dev:/dev \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /etc/localtime:/etc/localtime:ro \
        -v "$XAUTHORITY_PATH":/root/.Xauthority:rw \
        \
        "$IMAGE_NAME"
fi
