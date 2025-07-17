#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>


class PointCloudAssembler{
	private:
		ros::Timer fiveSecTimer;
		ros::NodeHandle node;
		ros::Subscriber pointCloudInSub;
		ros::Publisher pointCloudOutPub;
		ros::Publisher otherCloudPub;
		ros::Publisher filteredCloudPub;

		// pcl::PCLPointCloud2 currentPointCloud;
		pcl::PointCloud<pcl::PointXYZ> currentPointCloud;
		sensor_msgs::PointCloud2 currentPCMes;

		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener;

		bool isFirstTime = true;
		bool isAllowedToUpdate = true;
    
	public: 
		PointCloudAssembler(): tfListener(tfBuffer){
			pointCloudOutPub = node.advertise<sensor_msgs::PointCloud2>("/arm/assembled_point_cloud", 1);
			otherCloudPub = node.advertise<sensor_msgs::PointCloud2>("/arm/other_cloud", 1);
			filteredCloudPub = node.advertise<sensor_msgs::PointCloud2>("/arm/filtered_cloud", 1);
			pointCloudInSub = node.subscribe("/cloud_input", 1, &PointCloudAssembler::onCloudInput, this);

			fiveSecTimer = node.createTimer(ros::Duration(2,0), &PointCloudAssembler::fiveSecTimerCallback, this);
		}

		void fiveSecTimerCallback(const ros::TimerEvent& e){
			isAllowedToUpdate = true;
		}

   		void onCloudInput(const sensor_msgs::PointCloud2ConstPtr& input){
			// pcl::PCLPointCloud2 cloudIn;
			// pcl_conversions::toPCL(*input, cloudIn);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclInPtr(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*input, *pclInPtr);

			pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
			voxelFilter.setInputCloud(pclInPtr);
			voxelFilter.setLeafSize(0.2, 0.2, 0.2);
			voxelFilter.filter(*filteredCloud);

			std::string targetFrame = "base_link";
			sensor_msgs::PointCloud2 output, filteredOutput;

			try{
				// ROS_INFO("I made it here pt1");
				// std::cout << "cout works" << std::endl;
				geometry_msgs::TransformStamped tfStamped = tfBuffer.lookupTransform(targetFrame, input->header.frame_id, ros::Time(0));
				// ROS_INFO("I made it here pt2");

				pcl::PointCloud<pcl::PointXYZ> baseLinkCloud;
				pcl_ros::transformPointCloud(*filteredCloud, baseLinkCloud, tfStamped.transform);
				if(isFirstTime){
					currentPointCloud = baseLinkCloud;
			  		isFirstTime = false;
					ROS_INFO("First time!");
				}
				else currentPointCloud += baseLinkCloud;
				pcl::toROSMsg(currentPointCloud, output);
				output.header.frame_id = targetFrame;
				pointCloudOutPub.publish(output);
				// ROS_INFO("filtered cloud published at base");
			}	
			catch (tf2::TransformException &e){
				// ROS_WARN("%s", e.what());
			}

			pcl::toROSMsg(*filteredCloud, filteredOutput);
			filteredCloudPub.publish(filteredOutput);

			// ROS_INFO("I WAS ALLOWED TO UPDATE");
		}

		void combinePointCloud(const sensor_msgs::PointCloud2Ptr& pc2){
			if(isAllowedToUpdate){
				if(!currentPCMes.data.empty()){
					currentPCMes.data.insert(currentPCMes.data.end(), pc2->data.begin(), pc2->data.end());
					currentPCMes.width += pc2->width;
				}
				else{
					currentPCMes = *pc2;
				}

				pointCloudOutPub.publish(currentPCMes);
			}
		}
};

int main (int argc, char** argv) {
	ros::init(argc, argv, "arm_point_cloud_assembler"); // start node
	ROS_INFO("node started");
	PointCloudAssembler pointCloudAssembler;

	// Spin
	ros::spin ();
	return 0;
}
