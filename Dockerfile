# Vendor libiimtcl.so is supplied for x86-64 Linux only.
FROM --platform=linux/amd64 osrf/ros:humble-desktop
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/iimt
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions python3-rosdep \
    python3-venv python3-pip python3-pytest python3-yaml \
    ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rmw-cyclonedds-cpp util-linux tini \
    && rm -rf /var/lib/apt/lists/*
COPY requirements-notebook.txt ./
RUN python3 -m venv --system-site-packages /opt/iimt/venv \
    && /opt/iimt/venv/bin/pip install --no-cache-dir -r requirements-notebook.txt \
    && /opt/iimt/venv/bin/python -m ipykernel install --prefix=/opt/iimt/venv \
       --name iimt-ros2 --display-name 'IIMT ROS 2 Humble (Python 3.10)'
COPY ros2_ws/src ./ros2_ws/src
RUN source /opt/ros/humble/setup.bash \
    && (test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init) \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble -y \
    && cd ros2_ws \
    && colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    && rm -rf /var/lib/apt/lists/*
COPY notebooks ./notebooks
COPY data ./data
COPY docker ./docker
COPY scripts ./scripts
COPY README.md ./
COPY docs ./docs
RUN chmod +x docker/*.sh scripts/*.sh \
    && mkdir -p notebooks data results config /run/iimt
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/opt/iimt/docker/entrypoint.sh"]
CMD ["bash"]
