from ros:galactic

RUN apt-get update && apt-get install -y python3-requests python3-psutil

WORKDIR /root/
RUN mkdir -p /root/src/
COPY home_assistant_bridge /root/src/home_assistant_bridge
COPY home_assistant_msgs /root/src/home_assistant_msgs
RUN bash -c "source /opt/ros/galactic/setup.bash && colcon build"
COPY launch /root/src/launch
ENTRYPOINT bash -c "cd /root/ && source /opt/ros/galactic/setup.bash && source install/setup.bash && ros2 launch src/launch/default.py"
