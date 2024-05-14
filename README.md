# slam_tools
This repository maintained slam tools, now there has :
1. rotation_transform, this project can trans Quaterion, Rotation Matrix, Euler Angels to each other very conveniently.
2. play_pcd_in_map, this project shows moving trace on a point cloud map, which use slam pose and corresponding point clouds.
3. func_bag2pcd, this project parse rosbag pointcloud topic msgs to pcd files, there is several pcd types, like XYZIRT in 444128 bit, and others, its also can be defined by users;
4. func_pcd2bag, this project combine pointcloud, imu&gps msgs to a rosbag, plz check data timestamp before use this func, its a reverse function of func_bag2pcd;
5. colorful_print, this project shows an ANSI method to print many types of print, and can be defined by users easily, especially like INFO, WARNING and ERROR;