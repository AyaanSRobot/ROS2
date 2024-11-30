# Base image is a clean ROS2 jazzy installation on Ubuntu 22
FROM ros:jazzy

# use bash istedet for sh
SHELL ["/bin/bash", "-c"]

# Install dependencies. Our container provides isolation, so we can install these
# packages "system-wide" which is just container-wide.
RUN apt update
RUN apt install -y python3-curtsies python3-flask
# Note: ROS documentation is centered around system-wide installs. If you use a virtual
# environment, you have to make sure to use the same version as the system install. You
# could get the version like so: $ docker run --rm -it ros:jazzy python3 --version

# -- Build --  
# When I've worked on e.g. the webserver for this exercise, it was annoying to wait for
# the other packages to get built every time. By making a multi-stage build
# (https://docs.docker.com/build/building/multi-stage/), docker cashes the builds of the
# other packages.
#
# Make a new folder "/our-workspace" in the container and change directory to it.
WORKDIR /our-workspace
# Copy the first three packages from "ros-workspace" (host) into the new folder in the
# container:
COPY ros-workspace/src/santa_sim_interface ./src/santa_sim_interface
COPY ros-workspace/src/santa_sim ./src/santa_sim
COPY ros-workspace/src/keyboard_pub_pkg ./src/keyboard_pub_pkg
# build the first three packages:
RUN source "/opt/ros/jazzy/setup.bash" && colcon build

# When we update the code of the webserver, "docker build" can start at this stage,
# becaue the above is cached. Try changing "Santa Sim" in
# santa_sim_webserver/santa_sim_webserver/index.html to something else and run docker
# build again. It should only take a few seconds.
COPY ros-workspace/src/santa_sim_webserver ./src/santa_sim_webserver
RUN source "/opt/ros/jazzy/setup.bash" && colcon build

# You can do the same when you develop your keyboard controller.  
# Outcomment these lines:  
COPY ros-workspace/src/santa_keyboard_controller ./src/santa_keyboard_controller  
RUN source "/opt/ros/jazzy/setup.bash" && colcon build  

# Entrypoint: "entrypoint.sh" makes sure to source ROS2 and our build, so we don't have
# to do it manually when starting the container.
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
