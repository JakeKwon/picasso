# pr2_ws
workspace for grasping with pr2 in gazebo

###Dependencies
Make sure ros is installed. Then install the pr2 and gazebo packages. 
```
sudo apt-get install ros-indigo-gazebo-*
sudo apt-get install ros-indigo-pr2*
```

###Installation
```
git clone git@github.com:CURG/pr2_ws.git
cd pr2_ws/src
source /opt/ros/indigo/setup.bash
catkin_init_workspace
cd ..
catkin_make
```

###Running
```
source devel/setup.bash
source src/pr2_launch/launch/set_env.sh
roslaunch pr2_launch pr2_gazebo.launch
```

Now you should have the pr2 up inside gazebo.


###Common errors:
Gazebo pops up but window is all black, and nothing is visble.
```
Exception [Master.cc:50] Unable to start server[bind: Address already in use]. There is probably another Gazebo process running.
```
Run:
```
jvarley@skye:~/ros/pr2_ws$ ps aux | grep gz
jvarley   7558  0.0  0.0   4444   704 ?        Ss   Nov07   0:00 /bin/sh /opt/ros/indigo/lib/gazebo_ros/gzserver -u -e ode worlds/empty.world __name:=gazebo __log:=/home/jvarley/.ros/log/1212b656-a523-11e6-973d-f8b156c96bca/gazebo-2.log
jvarley   7606  165  6.3 6094968 519848 ?      Sl   Nov07 4396:47 gzserver -u -e ode worlds/empty.world __name:=gazebo __log:=/home/jvarley/.ros/log/1212b656-a523-11e6-973d-f8b156c96bca/gazebo-2.log -s /opt/ros/indigo/lib/libgazebo_ros_paths_plugin.so -s /opt/ros/indigo/lib/libgazebo_ros_api_plugin.so
jvarley  21194  0.0  0.0  11748   940 pts/27   S+   11:05   0:00 grep --color=auto gz
jvarley@skye:~/ros/pr2_ws$ kill -9 7606
jvarley@skye:~/ros/pr2_ws$ ps aux | grep gz
jvarley  21202  0.0  0.0  11748   940 pts/27   S+   11:05   0:00 grep --color=auto gz

```


###Starting 
