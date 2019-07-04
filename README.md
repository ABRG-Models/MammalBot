# MammalBot
Computational models and related work for the MammalBot cognitive architecture.

*This open source software code was developed in part or in whole in the Human Brain Project, funded from the European Union’s Horizon 2020 Framework Programme for Research and Innovation under the Specific Grant Agreement No. 785907 (Human Brain Project SGA2).*

![alt text](./images/integration.png "MammalBot Infrastructure")

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
