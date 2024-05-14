catkin_make
source devel/setup.bash
rosrun bag2pcd bag2pcd $1 $2 $3
$1: bag file
$2: output path
$3: intput lidar points type