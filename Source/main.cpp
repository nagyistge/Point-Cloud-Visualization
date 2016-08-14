/***************************************************************************
*                                                                         *
*   Developed By Deniz Beker											  *
*	Used in MSc Thesis Presentation.									  *
*	2014, Yeditepe University, Istanbul,								  *		
*																		  *
*	This code is loading all the point cloud meshes which are captured	  *
*	with 15 degree differences from center of a room. Applies			  *
*	segmentation to each point cloud to extract the planar surfaces (i.e. *
*	floor, ceiling, walls).	Then, it finds the boundaries of each         *
*	cloud objects and creates a fake bounding box for all of them		  *
*																		  *
*   https://github.com/dBeker                                             *
*                                                                         *
***************************************************************************/

// General Headers
#include <iostream>

// Boost - Threading purpose
#include <boost/thread/thread.hpp>

// PCL Includes
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>

#define BASE_PATH "D:\\Freelance\\Self\\Github\\Point-Cloud-Visualization\\Data\\"
#define START_CNT 1
#define FINISH_CNT 12
#define EXTENSION ".pcd"

using namespace pcl;
using namespace std;

vector<PointCloud<PointXYZRGB>::Ptr> visualizeVec;

double angle = 15;
boost::mutex mutex;
bool newCommand = false;

// Calculates central point of a vector by using arithmetic mean
// Notice that, this mean calculation is vulnerable to the noise on point cloud data
Eigen::Vector3f cloudCentralPoint(PointCloud<PointXYZRGB>::Ptr cloud)
{
	float xyz[3] = { 0.0f, 0.0f, 0.0f };
	float count = 0.0f;

	for (size_t i(0); i < cloud->size(); i++)
	{
		xyz[0] = xyz[0] + (float)cloud->at(i).x;
		xyz[1] = xyz[1] + (float)cloud->at(i).y;
		xyz[2] = xyz[2] + (float)cloud->at(i).z;
		count = count + 1.0f;
	}

	xyz[0] = xyz[0] / count;
	xyz[1] = xyz[1] / count;
	xyz[2] = xyz[2] / count;
	
	return Eigen::Vector3f(xyz[0], xyz[1], xyz[2]);
}

// Rotate point cloud based on 
void rotateCloud(PointCloud<PointXYZRGB>::Ptr* cloud, Eigen::Vector3f midPoint, double angle_x, double angle_y, double angle_z)
{
	

	Eigen::Matrix4f transformationMatrix;

	float angle;

	// Rotate Based on X-Axis
	if (angle_x != 0)
	{
		angle = M_PI * angle_x / 180;
		transformationMatrix <<
			1, 0, 0, 0,
			0, cos(angle), -sin(angle), 0,
			0, sin(angle), cos(angle), 0,
			0, 0, 0, 1;
		transformPointCloud(**cloud, **cloud, transformationMatrix);
	}

	// Rotate Based on Y-Axis
	if (angle_y != 0)
	{
		angle = M_PI * angle_y / 360;
		transformationMatrix <<
			cos(angle), 0, sin(angle), 0,
			0, 1, 0, 0,
			-sin(angle), 0, cos(angle), 0,
			0, 0, 0, 1;
		transformPointCloud(**cloud, **cloud, transformationMatrix);
	}

	// Rotate Based on Z-Axis
	if (angle_z != 0)
	{
		angle = M_PI * angle_z / 360;
		transformationMatrix <<
			cos(angle), -sin(angle), 0, 0,
			sin(angle), cos(angle), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		transformPointCloud(**cloud, **cloud, transformationMatrix);
	}
}

// Loader and processor thread. 
void process()
{
	// Iterate through all point clouds
#pragma omp parallel for
	for (size_t i = START_CNT; i <= FINISH_CNT; i++)
	{
		// Load pcd file to cloud object
		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		std::string pointCloudPath = BASE_PATH + std::to_string((_Longlong)i) + EXTENSION;

		if (io::loadPCDFile<PointXYZRGB>(pointCloudPath, *cloud) == -1)
		{
			std::cout << "Error Load : " << pointCloudPath << std::endl;
			continue;
		}
		std::cout << "Loaded : " << pointCloudPath << std::endl;


		// Point cloud is now loaded.
		// Define the parameters

		ModelCoefficients::Ptr coefficients(new ModelCoefficients);	// Segmentation Coefficients
		PointIndices::Ptr inliers(new PointIndices);
		SACSegmentation<PointXYZRGB> seg;	// Segmentation Object
		
		// Calculate Central Point of Cloud
		Eigen::Vector3f midPoint = cloudCentralPoint(cloud);
		// Rotate Cloud based on given angle
		rotateCloud(&cloud, midPoint, 0, angle, 0);

		// Set segmentation parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(SACMODEL_PLANE);	// Extract only planes
		seg.setMethodType(SAC_RANSAC);		// Use RANSAC to improve the plane segmentation quality
		seg.setDistanceThreshold(0.01);		// Maximum acceptable distance between points
		seg.setInputCloud(cloud);			// Set input point cloud
		seg.segment(*inliers, *coefficients);	// Apply Segmentation
			

		PointCloud<PointXYZRGB>::Ptr cloud_segmented(new PointCloud<PointXYZRGB>);
		// Get all the inlier points and discard the outliers
#pragma omp parallel for
		for (size_t i = 0; i < inliers->indices.size(); ++i)
		{
			PointXYZRGB point;
			point.x = cloud->points[inliers->indices[i]].x;
			point.y = cloud->points[inliers->indices[i]].y;
			point.z = cloud->points[inliers->indices[i]].z;
			point.r = cloud->points[inliers->indices[i]].r;
			point.g = cloud->points[inliers->indices[i]].g;
			point.b = cloud->points[inliers->indices[i]].b;

			cloud_segmented->points.push_back(point);
		}

		// Calculate maximum and minimum of segmented point cloud
		PointXYZRGB first_min, first_max;
		getMinMax3D(*cloud_segmented, first_min, first_max);

		// Create bounding point cloud
		PointCloud<PointXYZRGB>::Ptr cloud_boundary(new PointCloud<PointXYZRGB>);

		// Populate boundary cloud with points
		for (int i = first_min.x; i < first_max.x;i++)
		{
			for (int j = first_min.y; j < first_max.y;j++)
			{
				for (int k = first_min.z; k < first_max.z;k++)
				{
					PointXYZRGB point;
					point.x = i;
					point.y = j;
					point.z = k;
					cloud_boundary->push_back(point);
					k = k + 24;
				}
				j = j + 24;
			}
			i = i + 24;
		}
		// Insert point cloud to visualization buffer
		mutex.lock();
		visualizeVec.push_back(cloud_boundary);
		mutex.unlock();

	}

	// When all the loading is completed, set new command flag to load clouds in viewer
	mutex.lock();
	newCommand = true;
	mutex.unlock();

	// Wait for input from user
	char value;
	std::cin >> value;

	// Based on input, set angle
	// TODO: Protection against other values
	if (value == 'd')
	{
		angle += 15;
	}
	else if (value == 'a')
	{
		angle += -15;
	}

	// Call self again to process with new angle value
	// Note that it is not the most efficient method. In production, this part has to be improved.
	// Ideally, input pcd files have to be kept in cache and rotation has to be applied only 
	// TODO : Improve performance
	process();
}

// Visualization Thread Function
void visualize()
{
	// Create Viewer Object
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));

	// Set Viewer Object's Parameters
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(100.0, 0);
	viewer->initCameraParameters();

	// Loop 
	while (!viewer->wasStopped())
	{
		// If new point cloud is inserted to the list, clear all existing shapes and add all of them again.
		// This is not an incremental insertion.
		// TODO: Improve the performance issue in here
		if (newCommand)
		{
			mutex.lock();

			viewer->removeAllPointClouds();
			viewer->removeAllShapes();

			// Iterate all visualization objects
			for (size_t i = 0; i < visualizeVec.size(); i++)
			{
				// Create fake color
				visualization::PointCloudColorHandlerCustom<PointXYZRGB> rgb(visualizeVec.at(i), std::rand() % 255, std::rand() % 255, std::rand() % 255);

				// Give all of the point clouds a unique id and add them to viewer
				viewer->addPointCloud<PointXYZRGB>(visualizeVec.at(i), rgb, "cloud" + std::to_string((_Longlong)i));

				// Set rendering properties
				viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud" + std::to_string((_Longlong)i));
			}

			// Clear new command flag
			newCommand = false;

			// Clear visualization buffer
			visualizeVec.clear();

			// Unlock mutex
			mutex.unlock();
		}
		else
		{
			// Visualization object
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(10000));
		}
	}
}

int main(int argc, char** argv)
{

	boost::thread* th1 = new boost::thread(visualize);
	boost::thread* th2 = new boost::thread(process);

	th1->join();
	th2->join();

	return 0;
}