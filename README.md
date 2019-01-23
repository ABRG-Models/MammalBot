# MammalBot
For models and work forming the MammalBot architecture

## Docker

### Downloading Docker images and running Miro Gazebo
Go to `MammalBot/docker/Miro_Gazebo_Simulator`:

- To install and download all images run: `./launch.sh install`
- To launch the Miro Gazebo Simulator, run: `./launch.sh`

This will run gazebo in a docker container and open a browser tab with the simulation visualisation

### Pycharm Settings for Python Development
1. Go to `File -> Settings -> Build,Execution,Deployment -> Docker`
2. In `Docker -> Tools` make sure `Docker Machine executable` is set to `docker` and you should see a version number pop up on the right after a while 18.xx.x or higher
3. In `Docker`, hit the `+` sign, give your docker server a name (referred to as _DockerServerName_ from here onwards), choose `Unix Socket` and wait for the `Connection successful` message to show below the options

You have now connected pycharm to the docker service on your computer. Now we need to point pycharm towards the docker image it needs to use

4. Go to `File -> Settings -> Project:<ProjectName> -> ProjectInterpreter` and click the gear icon on the top right (in line with `Project Interpreter` drop down menu) and click `Add...`
5. This brings up the interpreter selector menu. From the options on the left, choose `Docker`
6. In the `Server:` dropdown, find your _DockerServerName_ name from step 3
7. In the `Image name:` dropdown, choose `dcamilleri13/miro_ros:latest`
8. In the 'Python interpreter path' put in the python command you need. `python` if using the Miro Gazebo Simulator, `python3` if you are using something else
9. Hit `OK` to exit and wait until interpreter is found
10. Once found, choose your Remote Python Interpreter from the `Project Interpreter` drop down list which should look like this : `Remote Python 2.7.12 Docker (dcamilleri13/miro_ros:latest)`
11. Wait until all packages have been discovered and hit `OK` to exit. Once exited, in the lower right of the pycharm window you should see `X processes remaining`. Pycharm is, at this point, going through all the libraries inside of the docker image and making a list of available libraries to allow for code completion. Wait for this to finish
12. Once import has finished, you now need to set the configuration in which to launch your python code. This is done by going to `Run -> Edit Configurations`
13. In this menu, give a name to your configuration next to `Name:`
14. Choose the script you want to execute inside the `miro_ros` image and set its path in the script path
15. Make sure the `Python Interpreter` is set to `Remote Python 2.7.12 Docker (dcamilleri13/miro_ros:latest)`
16. Finally scroll  a bit downwards to find `Docker container settings:` and click the open folder icon to edit its parameters
17. In the resulting popup, change:
    - `Network Mode` from `bridge` to `host`
    - Tick `Publish all ports`
18. Add the following Name - Value entries to the `Environment variables` widget:
    - PYTHONPATH - `/usr/local/src/robot/mdk/share:/opt/ros/kinetic/lib/python2.7/dist-packages`
    - PATH - `/opt/ros/kinetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin`
    - LD_LIBRARY_PATH - `/usr/local/src/robot/catkin_ws/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu`
    - ROS_IP - `127.0.0.1`
    - ROS_MASTER_URI - `http://$ROS_IP:11311`
    - ROS_PACKAGE_PATH - `/usr/local/src/robot/mdk/share:/usr/local/src/robot/catkin_ws/src:/opt/ros/kinetic/share`
19. Click `OK` to exit popup and save settings and a further `OK` to exit `Edit configurations`.
20. Your configuration is now available in the top right side of the Pycharm window. Hit the play button to launch
21. Enjoy programming on your host while running the code with the miro_ros python interpreter
