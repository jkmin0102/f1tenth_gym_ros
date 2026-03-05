#FROM ros:foxy

#SHELL ["/bin/bash", "-c"]

## dependencies
#RUN apt-get update --fix-missing && \
#    apt-get install -y git \
#                       nano \
#                       vim \
#                       python3-pip \
#                       libeigen3-dev \
#                       tmux \
#                       ros-foxy-rviz2
#RUN apt-get -y dist-upgrade
#RUN pip3 install transforms3d

# f1tenth gym
#RUN git clone https://github.com/f1tenth/f1tenth_gym
#RUN cd f1tenth_gym && \
#    pip3 install -e .

# ros2 gym bridge
#RUN mkdir -p sim_ws/src/f1tenth_gym_ros
#COPY . /sim_ws/src/f1tenth_gym_ros
#RUN source /opt/ros/foxy/setup.bash && \
#    cd sim_ws/ && \
#    apt-get update --fix-missing && \
#    rosdep install -i --from-path src --rosdistro foxy -y && \
#    colcon build

#WORKDIR '/sim_ws'
#ENTRYPOINT ["/bin/bash"]


FROM ros:foxy-desktop

SHELL ["/bin/bash", "-lc"]

# 기본 개발/빌드 도구 + OpenGL 진단 도구(가벼움)
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ROS 환경 자동 source (컨테이너 들어가면 바로 ros2 사용 가능)
RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

WORKDIR /sim_ws
CMD ["/bin/bash"]
