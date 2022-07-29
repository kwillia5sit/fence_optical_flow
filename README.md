# fence_optical_flow
Scripts for tracking the feature transformation of an underwater fence viewed through sonar

<p> Data <br>
The fence data type is image. It is found by subscribing to '/sonar_oculus/M1200d/image' in the recieve_message() function. <br>
The fence is the dotted pattern at the bottom of the image arc. <br>
Cropped_range_optical_flow crops the image arc to view just that section (the rest is a bright wall that causes too much noise in the feature detection). </p>
	
	Note: If switching to see the old field data, subscribe to '/sonar_oculus/image' instead.
	Also run oculus_viewer_2.py so it can convert the ping data to image data and be used by the script
	
<p> Inputs and Outputs for each function: <br>
**oculus_viewer_2.py** <br>
		input: ping data <br>
		output: image data <br>
**sonar_optical_flow.py** <br>
		input: data, set parameters <br>
		output: lines and dots on image tracking the optical flow movement <br>
**cropped_range_optical_flow.py** <br>
		input: data, crop shape/size, set parameters <br>
		output: lines and dots on a cropped image tracking the optical flow movement <br>
Other outputs in the terminal (prints) are for trouble shooting or visualization of steps. Not calculations/flow outputs. </p>




