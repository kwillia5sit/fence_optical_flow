# fence_optical_flow
These scripts are for tracking the feature transformation of an underwater fence viewed through sonar oculus M1200d. 

<p> <strong> Data </strong> <br>
The fence data type is image. It is found by subscribing to '/sonar_oculus/M1200d/image' in the recieve_message() function. <br>
The fence is the dotted pattern at the bottom of the image arc. <br>
Cropped_range_optical_flow crops the image arc to view just that section (the rest is a bright wall that causes too much noise in the feature detection). </p>

	Note: If switching to see the old marine data, subscribe to '/sonar_oculus/image' instead.
	Also run oculus_viewer_2.py so it can convert the ping data to image data and be used by the optical flow script
	
<h2> Inputs and Outputs for each function: </h2>
<strong> oculus_viewer_2.py </strong> <br>
		input: ping data <br>
		output: image data <br>
<strong> sonar_optical_flow.py </strong> <br>
		input: data, set parameters <br>
		output: lines and dots on image tracking the optical flow movement <br>
<strong> cropped_range_optical_flow.py </strong> <br>
		input: data, crop shape/size, set parameters <br>
		output: lines and dots on a cropped image tracking the optical flow movement <br>
Other outputs in the terminal (prints) are for trouble shooting or visualization of steps. Not calculations/flow outputs. </p>

<h2> Dependencies </h2>
<strong> python </strong> <br>
<ul>
<li> cv_bridge</li>
<li> cv2 </li>
<li> numpy </li>
<li> rosbag </li>
<li> sensor_msgs </li>
<li> random </li>
<li> pyyaml </li>
</ul>

<strong> ROS </strong> <br>
<ul>
<li> ROS-noetic </li>
<li> catkin_ws </li>
Note: Use catkin build instead of catkin make for the workspace
</ul>

<h2> How to run </h2>
<ol>
	<li> Run Roscore </li>
	<li> python3 Run oculus_viewer_2.py (If running old marine data) </li>
	<li> python3 Run optical flow script </li>
	<li> "rosbag play -s 100 [bag file name]" Run bag file </li>
</ol>
<ul>
	<li> sample_data.bag is old marine data </li>
	<li> 3_meter_orbit.bag is fence in tank data </li>
	<li> Recommended to use the cropped version on fence data and non-cropped on marine data </li>
</ul>

	Note: If using the old marine data, subscribe to '/sonar_oculus/image' <br>
	If using fence data, subscrbe to '/sonar_oculus/M1200d/image'. <br>
	The subscriber part is found in the receive_messages function in the optical flow scripts (cropped and general)
	
<h2> Common Troubleshooting </h2>
<strong> One frame shows, but the image doesn't play </strong> <br>
	If you get one frame to show up but then the image doesn't move, there is a cv2.waitKey(0) in the code by mistake. Change this to cv2.waitKey(1) for the frames to move in real time. <br>

<h2> Folder with bag data </h2>
The sample_data.bag is the old marine data. Credit to jake3991 for data.<br>
The 3_meter_orbit.bag is the new fence in a tank data. Credit to the Jake and his lab<br>
Google Drive link:<br>
https://drive.google.com/drive/folders/1Sgbykf3KxymAlvE6IEBY8_Bkyeqsaj60?usp=sharing

