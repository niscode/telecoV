$ git clone git@github.com:niscode/scripts.git
	➡   ホームディレクトリに配置後、$ ./ros-noetic-setup.sh を起動
	    これで $ roscore できるようになる

$ sudo apt -y install ros-noetic-rosserial
$ sudo apt -y install ros-noetic-slam-gmapping 
$ sudo apt -y install ros-noetic-navigation

$ cd catkin_ws/src
➡  $ git clone https://github.com/niscode/telecoV.git
   $ git clone https://github.com/Slamtec/rplidar_ros.git
   $ git clone https://github.com/iralabdisco/ira_laser_tools.git

$ cd catkin_ws
$ catkin_make

$ roslaunch telecoV dual_gmapping.launch
$ roslaunch telecoV dual_naivation.launch
