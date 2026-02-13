#!/bin/bash
set -e  # Exit immediately if a command fails

echo ">>> [HOST] Starting Docker Build process..."

# Navigate to docker directory
if [ -d "dynosam_pkg/docker" ]; then
    cd dynosam_pkg/docker
elif [ -d "docker" ]; then
    cd docker
else
    echo "Error: Could not find docker directory. Run this from your workspace root."
    exit 1
fi

# Build Docker Image
echo ">>> [HOST] Building Docker Image (AMD64)..."
./build_docker_amd64.sh //

# Create Container
echo ">>> [HOST] Creating Container..."
# Note: Ensure you have your data mounted in the create_container script or add -v here if needed
./create_container_amd64.sh //

echo ">>> [HOST] Container 'dyno_sam' is up."
echo ">>> [HOST] Waiting 5 seconds for services to stabilize..."
sleep 5

echo ">>> [CONTAINER] sending setup commands to 'dyno_sam'..."

# We use a heredoc (<<EOF) to run complex commands inside the container via docker exec
docker exec -i dyno_sam bash << 'EOF'
set -e
source /opt/ros/kilted/setup.bash

# 1. Build the Main DynoSAM Core
echo ">>> [DOCKER] Building DynoSAM Core..."
export MAKEFLAGS="-j10"
colcon build

# 2. Create the Custom Publisher Package
echo ">>> [DOCKER] Creating rgbd_publisher_pkg..."
cd src
if [ ! -d "rgbd_publisher_pkg" ]; then
    ros2 pkg create rgbd_publisher_pkg \
      --build-type ament_python \
      --dependencies rclpy sensor_msgs cv_bridge
else
    echo "Package already exists, skipping creation."
fi

cd .. # Back to dev_ws

# 3. Write setup.py
echo ">>> [DOCKER] Writing setup.py..."
cat > src/rgbd_publisher_pkg/setup.py << 'PY_SETUP'
from setuptools import setup

package_name = 'rgbd_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='RGBD publisher for DynoSAM',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgbd_publisher = rgbd_publisher_pkg.rgbd_publisher:main',
        ],
    },
)
PY_SETUP

# 4. Write the Node Code (rgbd_publisher.py)

unset OPENMPI_VERSION
unset OMPI_MCA_coll_hcoll_enable
unset OPAL_PREFIX
export PATH=$(echo $PATH | tr ':' '\n' | grep -v '/opt/hpcx' | grep -v '/usr/local/mpi/bin' | paste -sd:)

sudo apt update
sudo apt install libopenmpi-dev openmpi-bin

which mpicc
mpicc --version

rm -rf build/ install/ log/

export MPI_C_COMPILER=/usr/bin/mpicc
export MPI_CXX_COMPILER=/usr/bin/mpicxx
colcon build

echo ">>> [DOCKER] Writing rgbd_publisher.py..."
# Ensure directory exists just in case
mkdir -p src/rgbd_publisher_pkg/rgbd_publisher_pkg/

cat > src/rgbd_publisher_pkg/rgbd_publisher_pkg/rgbd_publisher.py << 'PY_CODE'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import glob
import yaml
import numpy as np
import os

class RGBDPublisher(Node):

    def __init__(self):
        super().__init__('rgbd_publisher')

        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_rect_color', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)

        self.bridge = CvBridge()
        self.data_dir = '/root/data' 
        
        # Robust path checking
        if not os.path.exists(self.data_dir):
            self.get_logger().error(f"Data directory {self.data_dir} does not exist!")

        self.rgb_files = sorted(glob.glob(os.path.join(self.data_dir, 'rgb', '*.[pj][np][ge]*')))
        self.depth_files = sorted(glob.glob(os.path.join(self.data_dir, 'depth', '*.[pj][np][ge]*')))
        
        if not self.rgb_files:
            self.get_logger().error("No images found. Check your Docker Mounts!")
        
        self.cam_info = self.load_camera_info(os.path.join(self.data_dir, 'camera_info.yaml'))

        self.idx = 0
        self.timer = self.create_timer(0.033, self.publish_frame)
        self.get_logger().info(f"RGBD Publisher Started! Found {len(self.rgb_files)} frames.")

    def load_camera_info(self, path):
        msg = CameraInfo()
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            msg.width = data.get('image_width', 640)
            msg.height = data.get('image_height', 480)
            msg.k = [float(x) for x in data.get('camera_matrix', {}).get('data', [])]
            msg.d = [float(x) for x in data.get('distortion_coefficients', {}).get('data', [])]
            msg.r = [float(x) for x in data.get('rectification_matrix', {}).get('data', [])]
            msg.p = [float(x) for x in data.get('projection_matrix', {}).get('data', [])]
            msg.distortion_model = data.get('distortion_model', 'plumb_bob')
        except Exception as e:
            self.get_logger().warn(f"Could not load camera info: {e}. Using empty default.")
        return msg

    def publish_frame(self):
        if not self.rgb_files or self.idx >= len(self.rgb_files):
            self.idx = 0
            self.get_logger().info("Looping dataset...")
            return

        rgb = cv2.imread(self.rgb_files[self.idx])
        # Read with ANYDEPTH to preserve 16-bit, and UNCHANGED to avoid forced BGR
        depth = cv2.imread(self.depth_files[self.idx], cv2.IMREAD_UNCHANGED | cv2.IMREAD_ANYDEPTH)

        if rgb is None or depth is None:
            self.get_logger().warn(f"Failed to read frame {self.idx}")
            self.idx += 1
            return

        # FIX: Ensure depth is 1-channel (16UC1)
        if len(depth.shape) == 3:
            # If the image was saved as RGB, convert to grayscale (single channel)
            depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)

        # DynoSAM expects 16-bit unsigned
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        
        try:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')

            now = self.get_clock().now().to_msg()
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = "camera_optical_frame"
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "camera_optical_frame"
            self.cam_info.header.stamp = now
            self.cam_info.header.frame_id = "camera_optical_frame"

            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)
            self.info_pub.publish(self.cam_info)
        except Exception as e:
            self.get_logger().error(f"Bridge Error: {e}")

        self.idx += 1

def main():
    rclpy.init()
    node = RGBDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PY_CODE


# 5. Rebuild Workspace with new package
echo ">>> [DOCKER] Rebuilding workspace..."
colcon build --packages-select rgbd_publisher_pkg

rm -rf /home/user/dev_ws/build/dynosam_nn/ament_cmake_python/dynosam_nn_py/dynosam_nn_py
colcon build --packages-select dynosam_nn --symlink-install --allow-overriding dynosam_nn

source /home/user/dev_ws/install/setup.bash
python3 -c "import dynosam_nn_py._core; print('ok')"

echo ">>> [DOCKER] SETUP COMPLETE."
EOF

# ==========================================
# PHASE 3: EXECUTION INSTRUCTIONS
# ==========================================
echo ""
echo "========================================================"
echo "          SETUP SUCCESSFUL - READY TO RUN"
echo "========================================================"
echo "To run the system, open TWO terminals."
echo ""
echo "TERMINAL 1 (Publisher):"
echo "  docker exec -it dyno_sam bash"
echo "  source install/setup.bash"
echo "  ros2 run rgbd_publisher_pkg rgbd_publisher"
echo ""
echo "TERMINAL 2 (DynoSAM Algorithm):"
echo "  docker exec -it dyno_sam bash"
echo "  source install/setup.bash"
echo "  ros2 launch dynosam_ros dyno_sam_launch.py \\"
echo "    params_path:=/home/user/dev_ws/src/core/dynosam/params/ \\"
echo "    dataset_path:=/root/data/ \\"
echo "    v:=1 \\"
echo "    data_provider_type:=3 \\"
echo "    output_path:=/root/results"
echo "========================================================"