# PCL-Visualization
Visualization of multiple point cloud meshes by using PCL (Point Cloud Library)

# Description
This code is loading all the point cloud meshes which are captured with 15 degree differences from center of a room. Applies segmentation to each point cloud to extract the planar surfaces (i.e. floor, ceiling, walls).	Then, it finds the boundaries of each cloud objects and creates a fake bounding box for all of them. It is used to visualize the point clouds' boundaries.

# Dependencies
* PCL 1.6.0
* Visual Studio 2010

# Code Usage
* Add PCL_HOME to environment variables (i.e. D:\Libraries\PCL 1.6.0)
* Modify BASE_PATH to point the point cloud datas (.pcd)
* Modify point cloud counts (START_CNT) and (FINISH_CNT)

Detailed explanation of each step can be found inside code comments.

# Result

## Scene RGB Photos
![alt tag](https://github.com/dBeker/Point-Cloud-Visualization/blob/master/Images/1.jpg)
![alt tag](https://github.com/dBeker/Point-Cloud-Visualization/blob/master/Images/2.jpg)
![alt tag](https://github.com/dBeker/Point-Cloud-Visualization/blob/master/Images/3.jpg)

## Scene Point Cloud 
![alt tag](https://github.com/dBeker/Point-Cloud-Visualization/blob/master/Images/pc.jpg)

## Scene Boundary Box (Output of this software)
![alt tag](https://github.com/dBeker/Point-Cloud-Visualization/blob/master/Images/output.jpg)
