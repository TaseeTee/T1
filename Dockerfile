FROM ros:humble-ros-base-jammy AS interfaces
SHELL ["/bin/bash", "-c"]
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential cmake python3-dev python3-colcon-common-extensions \
        ros-humble-rosidl-default-generators ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*
COPY vendor/unitree_go/ /tmp/interfaces/src/unitree_go/
WORKDIR /tmp/interfaces
RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --install-base /opt/unitree_interfaces \
        --cmake-args -DBUILD_TESTING=OFF

FROM ros:humble-ros-base-jammy
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp ros-humble-rosgraph-msgs \
        ros-humble-tf2-ros-py ros-humble-nav-msgs ros-humble-sensor-msgs \
        python3-pip libgl1 libglfw3 \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir 'mujoco==3.3.7' 'numpy==1.26.4' 'scipy==1.11.4' \
    && useradd --uid 10001 --create-home participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /opt/participant
COPY --from=interfaces /opt/unitree_interfaces /opt/unitree_interfaces
COPY vendor/go1-model/ ./vendor/go1-model/
COPY manifest.py ./manifest.py
COPY tasks/ ./tasks/
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh /opt/participant/tasks/T2 /opt/participant/tasks/T3 /opt/participant/tasks/T4 \
    && bash -ec 'source /opt/ros/humble/setup.bash; source /opt/unitree_interfaces/local_setup.bash; python3 -c "import mujoco; from unitree_go.msg import LowCmd, LowState; assert len(LowCmd().motor_cmd) == len(LowState().motor_state) == 20"'

USER participant
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
