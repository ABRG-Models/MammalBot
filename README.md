# MammalBot
Computational models and other work forming the MammalBot cognitive architecture.

The MammalBot cognitive architecture is composed of several modular components, each representing a particular brain region and/or function. We use [ROS](https://www.ros.org) to interface with physical and virtual robots, [Gazebo](http://gazebosim.org) to simulate virtual robots, and [SpineML](http://spineml.github.io) for most model descriptions. Thus, there are some prerequisites for using or developing MammalBot:

## Software requirements
MammalBot is being developed with the [MiRo](https://www.miro-e.com) robot as a principal testbed, so you will currently need access to either a virtual or physical MiRo to develop or use MammalBot. Assuming you don't have access to a physical robot, this means you will need:
* Gazebo 7 for simulating the virtual MiRo robot, which requires
* ROS Kinetic, the latest ROS version compatible with Gazebo 7, which requires
* [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) (or an Ubuntu 16-based fork such as [Linux Mint 18.3](https://linuxmint.com/release.php?id=31)), the latest Ubuntu release compatible with ROS Kinetic
* [SpineCreator](https://github.com/SpineML/SpineCreator) and the associated programs [SpineML PreFlight](https://github.com/SpineML/SpineML_PreFlight), [SpineML to BRAHMS](https://github.com/SpineML/SpineML_2_BRAHMS), and the simulation engine [BRAHMS](https://github.com/BRAHMS-SystemML/brahms) to run MammalBot models.

There are two paths to getting set up:

### Native
If you have an Ubuntu 16.04 install or are able to create one, you may install the required software packages yourself.
1. Follow [these instructions](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Developer_Profiles_Simulator) to install ROS, the MiRo MDK, and Gazebo
2. Follow [these instructions](https://spineml.github.io/spinecreator/sourcelin/) to install SpineCreator and associated packages
	* You must set the `-DCOMPILE_PYTHON_BINDING=ON` flag when compiling BRAHMS to enable Python components

### Docker *(recommended)*
As installing and configuring multiple interreliant software packages can be a time-consuming task in itself, the MammalBot repo includes a Docker Compose file that will download a prebuilt Docker environment with all the required software preconfigured.
1. [Install Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
2. Clone the MammalBot repository to your machine
3. Run `docker-compose run spineml bash` from the repo directory; after the images are downloaded, a Gazebo window should appear with a virtual MiRo, and you will presented with a terminal prompt similar to `root@14595b1a7a95:/#`

This should work on any version of Linux but won't work on Macs, as [xhyve](https://github.com/machyve/xhyve) (on which Docker for Mac is built) [doesn't support GPU passthrough](https://github.com/machyve/xhyve/issues/108) required for Gazebo's 3D rendering

## Testing SpineML / Python / ROS integration
### Native install
1. Copy the contents of the `examples/bindings` folder from the MammalBot download to your Namespace root folder - by default this is `/usr/local/SystemML/Namespace`
2. Open SpineCreator and load the `bindings_examples.proj` model. You should see a very simple model with two single neurons and nothing else
3. To run the model:
	* Open a terminal and run `roscore` to start the ROS core 
	* Open a second terminal window and navigate to the `examples/projects/bindings` directory
	* Execute the model with `~/SpineML_2_BRAHMS/convert_script_s2b -m "$PWD" -e 0`
	* If the model has run successfully, the bottom of the output log will look something like:
    ```
    W3    0.928236       FINISHED
    W5    0.930353       FINISHED
    ```
    * To observe the model's output to an example ROS node, run `rostopic echo /example/python` in a third terminal window
    * Execute the model again to see data output to this ROS node
    
### Docker install
(Pending)
`docker-compose run spineml bash`

## Further reading
* BRAHMS:
	* [General documentation](http://brahms.sourceforge.net/docs/)
	* [BRAHMS Manager](http://brahms.sourceforge.net/docs/BRAHMS%20Manager.html)
	* [Python process development tutorial](http://brahms.sourceforge.net/docs/Quick%20Start%20(1262).html)
	* [Python component bindings reference](http://brahms.sourceforge.net/docs/Python%20(1262).html)
* SpineML / SpineCreator:
	* [General documentations](http://spineml.github.io)
	* [SpineCreator tutorial](http://spineml.github.io/spinecreator/tutorial/)

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
