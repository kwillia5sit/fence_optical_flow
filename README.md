# fence_optical_flow
These scripts are for tracking the feature transformation of an underwater fence viewed through sonar oculus M1200d. 

<p> <strong> Data </strong> <br>
The fence data type is image. It is found by subscribing to '/sonar_oculus/M1200d/image' in the recieve_message() function. <br>
The fence is the dotted pattern at the bottom of the image arc. <br>
Cropped_range_optical_flow crops the image arc to view just that section (the rest is a bright wall that causes too much noise in the feature detection). </p>

	Note: If switching to see the old marine data, subscribe to '/sonar_oculus/image' instead.
	Also run oculus_viewer_2.py so it can convert the ping data to image data and be used by the optical flow script
	
<h2> Scripts and Their Purposes </h2>
<strong> oculus_viewer_2.py </strong> <br>
		input: ping data <br>
		output: image data <br>
The old marine data needs its type converted to run in the optical flow scripts. This script must be run in order to do the live conversion. <br>  
<br>
<strong> sonar_optical_flow.py </strong> <br>
		input: data, set parameters <br>
		output: lines and dots on image tracking the optical flow movement <br>
This code calculates the optical flow of points in the sonar image.<br>
<br>
<strong> cropped_range_optical_flow.py </strong> <br>
		input: data, crop shape/size, set parameters <br>
		output: lines and dots on a cropped image tracking the optical flow movement <br>
This script crops and resizes the sonar image. It then calculates optical flow in the same fashion. Points are verified as good by testing that they do not move too far from the previous location within one frame. The good points are kept in a dictionary so that they match with one specific color. This feature currently does not work well in the uncropped sonar_optical_flow script.<br>
<br>
<strong>unstable_no_crop_consistent_colors</strong>
		input: data, set parameters <br>
		output: lines and dots on image tracking the optical flow movement <br>
		The purpose of this script is to apply the dictionary method used in cropped_range_optical_flow to the long-range images so that the dots and lines will be more consistent in which points are which colors. It currently runs but there are fewer points tracked than the original sonar_optical_flow.py and it is uncertain how reliable the points are.<br>
<br>
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


<h2> What's left to be done<'h2>
<ul>
	<li> YAML file with all the parameters for each code in one place </li>
	<li> accurate transform data </li>
	<li> stable colors for the long-range optical flow (if that is needed)</li>
<ul>
