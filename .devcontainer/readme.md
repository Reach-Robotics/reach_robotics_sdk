# Getting started with ROS 
The instructions below briefly describe how to get started with ROS 2 using a VS code devcontainer. To get started you'll need to,
1. Install VS Code (https://code.visualstudio.com/)
2. Install Docker (https://www.docker.com/)
3. (Windows) Install WSL2 

## Allowing container to spawn GUI applicaitons

The instructions below describe how to export GUI application from a docker container. To enable full functionality of ROS applications, further setup may be required.

### Ubuntu
1. open sshd_config `vim /etc/ssh/sshd_config` and verify `X11Forwarding yes`
2. run `xhost +Local:*` to authorise xhost over local connection
3. run `xhost`and check for `LOCAL:`
4. modify devcontainer.josn to correctly set the DISPLAY variable to `DISPLAY=${env:DISPLAY}`
4. mount the `/tmp/.x11-unix directory`

### Windows
1. install xserver on the host windows machine (e.g., VcXsrv or Xming)
2. start the xserver on the windowa machine
2. modify devcontainer.json to correctly set the DISPLAY environment variable to `host.docker.internal:0.0`

## Enabling external UDP connections 
1. Expose the UDP port (typically 6789) in the devcontainer by setting the 

## Enabling external serial connections 
On linux machines, USB/hardware ports are simply mounted to the container as a volume by mapping the host `/dev/` to the container `/dev/`. On windows machines, this is not possible. You may follow the instructions here (https://docs.docker.com/desktop/features/usbip/) to enable USB connection via a network conneciton. 

## Launching ROS 2 VS code container 
To launch ROS 2 development environment set the dockerFile variable in devcontainer.json to `foxy-dockerfile`. Support for other ROS distributions may be added by modifying the DockerFile as required. 