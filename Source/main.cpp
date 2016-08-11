#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

// OpenCV
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using namespace pcl;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vec;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> visualizeVec;
//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
double angle = 0;
boost::mutex mutex;
bool newCommand = false;

cv::Vec3f cloudMidPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	float xyz[3] = {0.0f, 0.0f, 0.0f}; 
	float count = 0.0f;

	for(size_t i(0); i < cloud->size(); i++)
	{
		xyz[0] = xyz[0] + (float)cloud->at(i).x;
		xyz[1] = xyz[1] + (float)cloud->at(i).y;
		xyz[2] = xyz[2] + (float)cloud->at(i).z;
		count = count + 1.0f;
	}

	xyz[0] = xyz[0] / count;
	xyz[1] = xyz[1] / count;
	xyz[2] = xyz[2] / count;

	return cv::Vec3f(xyz[0],xyz[1],xyz[2]);
}

void turnCloudViewPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr* cloud, cv::Vec3f midPoint, double angle_x, double angle_y, double angle_z)
{

	Eigen::Matrix4f transformationMatrix;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotatedCloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
	rotatedCloud = *cloud;

	float angle;
	// Rotate 
	if(angle_x != 0)
	{
		angle = M_PI * angle_x/180;
		transformationMatrix <<
			1, 0, 0, 0,
			0, cos(angle), -sin(angle), 0,
			0, sin(angle), cos(angle), 0,		
			0, 0, 0, 1;
		pcl::transformPointCloud(*rotatedCloud,*rotatedCloud,transformationMatrix); 
	}

	if(angle_y != 0)
	{
		angle = M_PI * angle_y/360;
		transformationMatrix <<
		cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1;
		pcl::transformPointCloud(*rotatedCloud,*rotatedCloud,transformationMatrix); 
	}

	if(angle_z!= 0)
	{
		angle = M_PI * angle_z/360;
		transformationMatrix <<
		cos(angle), -sin(angle), 0, 0,
		sin(angle), cos(angle), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
		pcl::transformPointCloud(*rotatedCloud,*rotatedCloud,transformationMatrix); 
	}
	// Push Back to Cloud Vector
	*cloud = rotatedCloud;
}

void process()
{
	for(size_t i(0); i<4; i++)
	{

		for(size_t j(1); j<4; j++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("F:\\x64\\Test\\Registered_" + std::to_string((_Longlong)i)+"_"+std::to_string((_Longlong)j)+".pcd", *cloud) == -1)
			{
				//std::cout << "Error Load : " << "F:\\x64\\Test1\\Registered_" + std::to_string((_Longlong)i)+"_"+std::to_string((_Longlong)j)+".pcd" <<std::endl;
				continue;
			}
			//std::cout << "Loaded : " << "F:\\x64\\Test1\\Registered_" + std::to_string((_Longlong)i)+"_"+std::to_string((_Longlong)j)+".pcd" <<std::endl;
			cv::Vec3f midPoint = cloudMidPoint(cloud);
			turnCloudViewPoint(&cloud,midPoint,0,angle,0);
			vec.push_back(cloud);
		}		 
	}
	for(size_t m(0); m<vec.size(); m++)
	{
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		seg.setInputCloud (vec.at(m));
		seg.segment (*inliers, *coefficients);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_r (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>) ;

#pragma omp parallel for
		for (size_t i = 0; i < inliers->indices.size (); ++i)
		{
			pcl::PointXYZRGB point;
			point.x = vec.at(m)->points[inliers->indices[i]].x;
			point.y = vec.at(m)->points[inliers->indices[i]].y;
			point.z = vec.at(m)->points[inliers->indices[i]].z;
			cloud_r->points.push_back (point);
		}

		

		float boundaries[6];
		pcl::PointXYZRGB first_min, first_max;
		pcl::PointCloud<pcl::PointXYZRGB> m_ptrCloud(*cloud_r);
		pcl::getMinMax3D(m_ptrCloud, first_min, first_max); 
		boundaries[0] = first_min.x;
		boundaries[1] = first_min.y;
		boundaries[2] = first_min.z;
		boundaries[3] = first_max.x;
		boundaries[4] = first_max.y;
		boundaries[5] = first_max.z;
		//std::cout << first_min << " " << first_max << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(int i=first_min.x; i<first_max.x;i++)
		{
			for(int j=first_min.y; j<first_max.y;j++)
			{
				for(int k=first_min.z; k<first_max.z;k++)
				{
					pcl::PointXYZRGB point;
					point.x=i;
					point.y=j;
					point.z=k;
					cloud->push_back(point);
					k=k+24;
				}
				j=j+24;
			}
			i=i+24;
		}
		mutex.lock();
		visualizeVec.push_back(cloud);
		mutex.unlock();
		
	}
	mutex.lock();
	newCommand=true;
	mutex.unlock();
	
	char value;
	std::cin >> value;
	std::cout << value;
	if(value == 'd')
	{
		angle += 15;
	}
	else if(value == 'a')
	{
		angle += -15;
	}
	vec.clear();
	visualizeVec.clear();
	process();

}

void turn()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (100.0, 0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		mutex.lock();
		if(newCommand)
		{
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();

			for(int i(0); i<visualizeVec.size(); i++)
			{
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(visualizeVec.at(i), std::rand()%255, std::rand()%255,std::rand()%255);
				viewer->addPointCloud<pcl::PointXYZRGB> (visualizeVec.at(i), rgb, "cloud"+std::to_string((_Longlong)i));
				//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(vec.at(i), 0, 50+i*10, 50+i*10);				
				//viewer->addPointCloud<pcl::PointXYZRGB> (vec.at(i), rgb, "cloud"+std::to_string((_Longlong)i));
				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud"+std::to_string((_Longlong)i));
			}

			newCommand=false;
			mutex.unlock();
		}
		else
		{
			mutex.unlock();
			viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds (10000));
		}
	}
}

int main (int argc, char** argv)
{

	boost::thread* th1 = new boost::thread(turn);
	boost::thread* th2 = new boost::thread(process);

	th1->join();
	th2->join();

	system("pause");
	return (0);
}