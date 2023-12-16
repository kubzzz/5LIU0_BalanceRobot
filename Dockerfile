FROM osrf/ros:humble-desktop-full

# install applications
RUN apt-get update && \
    apt-get install -y build-essential && \
    apt-get install -y iputils-ping && \
    apt-get install -y nano && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    apt-get install -y bash-completion && \
    apt-get install -y python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# create non-root user
ARG USERNAME=martijn
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# set-up sudo
RUN apt-get update && \
    apt-get install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    rm -rf /var/lib/apt/lists/*

RUN cd ~/volume

# for arguments
COPY entrypoint.sh /entrypoint.sh
# for new bashrc commands when a new terminal is opened
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]

# to RUN this dockercontainer arround a specific volume:
# RUN $ docker run -it --user martijn -e DISPLAY=host.docker.internal:0.0 --network=host --ipc=host 
# -v C:\Users\marti\Documents\Embedded_Systems\Year1\5LIU0_LinearSystemsSignalsControl\volume:/home/martijn/volume ros_image