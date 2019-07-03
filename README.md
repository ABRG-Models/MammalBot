# MammalBot
Computational models and related work for the MammalBot cognitive architecture.

*This open source software code was developed in part or in whole in the Human Brain Project, funded from the European Union’s Horizon 2020 Framework Programme for Research and Innovation under the Specific Grant Agreement No. 785907 (Human Brain Project SGA2).*

![alt text](./images/integration.png "MammalBot Infrastructure")


The MammalBot cognitive architecture is composed of several modular components, each representing a particular brain region and/or function. We use [ROS](https://www.ros.org) to interface with physical and virtual robots, [Gazebo](http://gazebosim.org) to simulate virtual robots, and [SpineML](http://spineml.github.io) for most model descriptions. Thus, there are some prerequisites for using or developing MammalBot:

## Software requirements
MammalBot is being developed with the [MiRo](https://www.miro-e.com) robot as a principal testbed, so you will currently need access to either a virtual or physical MiRo to develop or use MammalBot. Assuming you don't have access to a physical robot, this means you will need:
* Gazebo 7 for simulating a virtual MiRo robot, which requires
* ROS Kinetic, the latest ROS version compatible with Gazebo 7, which requires
* [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) (or an Ubuntu 16-based fork such as [Linux Mint 18.3](https://linuxmint.com/release.php?id=31)), the latest Ubuntu release compatible with ROS Kinetic
* You will also need [SpineCreator](https://github.com/SpineML/SpineCreator) and the associated programs [SpineML PreFlight](https://github.com/SpineML/SpineML_PreFlight), [SpineML to BRAHMS](https://github.com/SpineML/SpineML_2_BRAHMS), and the simulation engine [BRAHMS](https://github.com/BRAHMS-SystemML/brahms) to view, edit, and run MammalBot models.

There are two paths to getting set up:

### Docker *(recommended)*
As installing and configuring multiple interreliant software packages can be a time-consuming project in itself, the MammalBot repo includes a Docker Compose file that will launch a prebuilt Docker environment with all the required software preconfigured. This should work on any version of Linux but won't work on Macs, as [xhyve](https://github.com/machyve/xhyve) (on which Docker for Mac is built) [doesn't support GPU passthrough](https://github.com/machyve/xhyve/issues/108) required for Gazebo's 3D rendering.
#### To start
1. [Install Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
2. Clone the MammalBot repository to your machine
3. Run `docker-compose up -d` from the repo directory to download and start all required images. A Gazebo window should appear with a virtual MiRo robot in a walled arena.
	* Gazebo will sometimes quit prematurely due to a [known bug](https://github.com/vvv-school/assignment_computed-torque/issues/3#issuecomment-364370433). If the Gazebo window doesn't appear, run `docker-compose up -d` again
4. Run `docker exec -it mammalbot_spineml ./ros_entrypoint.sh bash` to open a Bash prompt inside the SpineML container. This should look similar to `root@14595b1a7a95:/#`. From here, you can run specific MammalBot experiments (see below).
#### To stop
1. Simply type `exit` within the SpineML container to return to the shell of your host machine.
2. Run `docker-compose stop` to halt all MammalBot containers.
	* You can restart MammalBot later with `docker-compose start`

### Native
If you have an Ubuntu 16.04 install or are able to create one, you may install the required software packages yourself.
1. Follow [these instructions](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Developer_Profiles_Simulator) to install ROS, the MiRo MDK, and Gazebo
2. Follow [these instructions](https://spineml.github.io/spinecreator/sourcelin/) to install SpineCreator and associated packages
	* You must set the `-DCOMPILE_PYTHON_BINDING=ON` flag when compiling BRAHMS to enable Python components

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
