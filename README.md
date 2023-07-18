# CNN Based Soft Exoskeleton for Neurorehabilitation
## ROS
<br />
Run in different terminals:
<pre>
roscore
</pre>
<pre>  
rosrun rosserial_python serial_node.py /dev/ttyAMC0
</pre>
<pre>  
cd exo_ws
</pre> 
<pre> 
rosrun servo_control publisher.py
</pre> 
<br/>
<br/>
launch cpp exo control test:
<pre> 
roslaunch exo_control exo_control.launch
</pre> 
<br/>
launch Neural Net https://github.com/davidmartinez13/YOLACT-mini-Instance-segmentation 
<pre> 
python3 detect_measure_ros_pub.py
</pre> 
