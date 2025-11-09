## 📄 README: ROS 2 (Humble) + Conda (Python 3.10) + YOLOv5

This document outlines the exact, stable environment configuration and steps required to run the `pedestrian_nav_ai` ROS 2 package using YOLOv5 within a dedicated Conda environment on Ubuntu 22.04 (ROS Humble).

### I. 🐍 Conda Environment Setup

Due to conflicts between ROS 2 libraries (like `cv_bridge`) and newer deep learning dependencies (like PyTorch and NumPy 2.x), a Python 3.10 Conda environment is mandatory.

1.  **Create the Environment:**

    ```bash
    conda create -n ros_yolo python=3.10
    conda activate ros_yolo
    ```

2.  **Install Core Deep Learning Packages:**
    *Use the official PyTorch installation method (e.g., for CUDA 11.8 or CPU-only) and install compatible versions of the deep learning libraries using `pip`.*

    ```bash
    # 1. Install PyTorch (Example for CPU)
    conda install pytorch torchvision torchaudio cpuonly -c pytorch

    # 2. Install Ultralytics and dependencies
    pip install ultralytics  # For the modern YOLO() class

    # 3. Ensure ROS-compatible NumPy and OpenCV versions are installed
    pip install numpy==1.24.4  # Crucial for ROS 2 Humble/cv_bridge compatibility
    pip install opencv-python==4.11.0.86 # Crucial for stability with ultralytics
    pip install PyYAML==6.0.1 urllib3 # Resolve dependency warnings
    ```

3.  **Install ROS Python Dependencies:**
    *Install the core ROS Python libraries into the Conda environment.*

    ```bash
    pip install rclpy cv-bridge
    ```

4.  **Install `ros-numpy` (Source):**
    *If `ros-numpy` is unavailable via `apt`, install it from source into the workspace while Conda is active:*

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/eric-wieser/ros_numpy.git
    cd ~/ros2_ws
    colcon build --packages-select ros_numpy
    ```

### II. 🔗 Critical System & Library Fixes

These steps resolve deep linker and library conflicts between Conda and the system's ROS installation.

1.  **Fix C++ Shared Library Conflict (`libstdc++.so.6`):**
    *This prevents the `undefined symbol: GLIBCXX_3.4.30 not found` error.*

    ```bash
    conda deactivate
    cd ~/anaconda3/envs/ros_yolo/lib
    mv libstdc++.so.6 libstdc++.so.6.backup
    conda activate ros_yolo
    ```

2.  **Fix GTK Display Backend (If `cv2.imshow` fails):**
    *Install system libraries required for OpenCV to display windows on Linux:*

    ```bash
    sudo apt update
    sudo apt install libgtk2.0-dev pkg-config
    ```

### III. 🔨 Build and Run

1.  **Build the Workspace:**
    *Ensure your Conda environment is active and ROS is sourced before building.*

    ```bash
    conda activate ros_yolo
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

2.  **Patch the Executable Script (Essential Fix):**
    *To resolve the persistent **`ModuleNotFoundError: No module named 'ultralytics'`**, the Conda path must be manually inserted into the ROS executable script.*

      * **Locate the executable:** `/home/nk/Music/DvaitAI/ros2_ws/install/pedestrian_nav_ai/lib/pedestrian_nav_ai/pedestrian_node`

      * **Edit the script** and insert the following two lines immediately after the initial shebang/imports:

        ```python
        # In the executable file:
        import sys
        # Insert the Conda site-packages path as the highest priority search path
        sys.path.insert(1, '/home/nk/anaconda3/envs/ros_yolo/lib/python3.10/site-packages')
        ```

3.  **Execute the Node:**

    ```bash
    ros2 run pedestrian_nav_ai pedestrian_node
    ```