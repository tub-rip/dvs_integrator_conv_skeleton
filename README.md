# dvs_integrator_conv

Event-by-event processing in ROS C++ for the purpose of image reconstruction (using a leaky integrator). Based on the paper by Scheerlinck et al., RA-L 2019 (see bottom of page).

The package reads events from a topic and outputs the reconstructed images convolved with a spatial kernel. The method uses per-pixel temporal filters (leaky integrators). It has the functionality to dynamically select a spatial convolution kernel.


## Input / Output
**Input**:
- Events (topic)

**Output**:
- State (for illustration purposes): time surface
- Reconstructed intensity convolved with a spatial kernel. It is almost the brightness state (just propagated--decayed-- to the current time).

**Parameters** (dynamic reconfigure):
- Decaying parameter ("cutoff frequency" alpha) of the leaky integrator.
- Selector of the convolution mask (e.g., identity, Sobel-x, Sobel-y, Blur, Laplacian, etc.)


## Dependencies

Requires [catkin_simple](https://github.com/catkin/catkin_simple) and [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) driver


## Compile

**Preliminaries**:
First, build `catkin simple` and the `davis_ros_driver`. The most important commands are:

	catkin build catkin_simple
	catkin build davis_ros_driver

Then, incorporate the packages to the path, so that ROS finds them:
	
	source ~/ros/catkin_ws_evis/devel/setup.bash
	
**Compile this package**:
	
	catkin build dvs_integrator_conv --force-cmake
	
The flag `--force-cmake` is optional.	
After building, at least the first time, remember to run:

	source ~/ros/catkin_ws_evis/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/dvs_integrator_conv/
	
An alternative command to start from scratch (cleaning all catkin pakages) is (to be used with *caution*): `catkin clean`


## Run example
Download a ROS bag dataset, for example [slider_depth](http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag) to play it in a terminal, that is, to use it as source of data (as if an event camera was connected to the computer).

Every time that you open a terminal, you may need to run:

	source ~/ros/catkin_ws_evis/devel/setup.bash

to have access to the ROS commands.

Run in a terminal (you may need to "source" first):

	roscore
	
Open the rqt perspective view with four visualizers. This file is in the launch folder. Run the following command in a terminal, from the dvs_integrator_conv package folder:

	rqt --perspective-file launch/integrator_view.perspective

Run the visualizer node in another terminal:

	roslaunch dvs_displayer display_monocular.launch

Run the image reconstruction node in another terminal:
	
	roslaunch dvs_integrator_conv integrator.launch

In another terminal play the bag with the event data:

	rosbag play -l -r 0.1 --pause path_to_file/slider_depth.bag

Press the space bar to  play / pause the reproduction of the bag. Event messages are accumulated in the suscriber of dvs_integrator, to avoid losing events for the reconstruction.
	
In another terminal open the dynamic reconfigure and play around with the parameters in the window named `dvs_displayer_one`
	
	rosrun rqt_reconfigure rqt_reconfigure

Using the same roscore during execution of multiple runs of the dvs_integrator allows the dynamic parameters to persist even if the nodes are killed. The next time they are started, they configuration parameters will be set using the ones from the previous session.

End the program execution with `Ctrl + C` keyboard shortcut. 


## TO DO:
- Allow for different contrast sensitivity depending on the polarity (using a parameter in dynamic reconfigure)
- Display events using a fixed number of them (currently we are displaying the events as arranged in ROS messages)
- Display events using a fixed time interval
- Allow interaction (using dynamic reconfifure), to change the number of events or the size of the time slice. Use a parameter to control the rate of output images published (number of events or Delta-t).


## Reference
- Scheerlinck et al., *[Asynchronous Spatial Image Convolutions for Event Cameras](https://www.cedricscheerlinck.com/event-convolutions)*, IEEE Robotics and Automation Letters, 4(2):816-822, April 2019.