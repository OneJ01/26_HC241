FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir mavsdk

WORKDIR /root/cam_ws

CMD ["/bin/bash"]
