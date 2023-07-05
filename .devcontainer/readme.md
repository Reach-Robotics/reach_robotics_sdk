# Getting started with ROS/ROS 2 on ubuntu 
The instructions below breifly describe how to get started with ROS/ROS 2 on ubuntu using a VS code devcontainer. 

## Allowing container to spawn GUI applicaitons (Ubuntu)
1. Open sshd_config `vim /etc/ssh/sshd_config` and verify `X11Forwarding yes`
2. run `xhost +Local:*` to authorise xhost over local connection
3. run `xhost`and check for `LOCAL:`

## Launching ROS/ROS 2 VS code container 
1. For ROS development environment set the dockerFile variable in devcontainer.json to `noetic-dockerfile`
2. For ROS 2 development environment set the dockerFile variable in devcontainer.json to `foxy-dockerfile`