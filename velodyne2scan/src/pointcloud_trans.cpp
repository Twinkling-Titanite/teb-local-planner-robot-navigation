#include <ros/ros.h>
// PCL 的相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>//滤波的头文件
#include <pcl/filters/passthrough.h>//滤波的头文件
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher pub; //声明发布器

float box_x_min=0, box_x_max=10;
float box_y_min=-2, box_y_max=2;
float box_z_min=0.0, box_z_max=1.0;

tf::TransformListener* tf_listener;

pcl::PointCloud<pcl::PointXYZ> Cloud_Transform(pcl::PointCloud<pcl::PointXYZ> cp_src)
{
    pcl::PointCloud<pcl::PointXYZ> res;

    geometry_msgs::PointStamped point_base,point_dst;
    point_base.header.frame_id=cp_src.header.frame_id;  
    point_base.header.stamp = ros::Time();

    for (auto it = cp_src.points.begin(); it!=cp_src.points.end(); ++it)
    {
        point_base.point.x = it->x;
        point_base.point.y = it->y;
        point_base.point.z = it->z;

        try{
            tf_listener->transformPoint("base_link", point_base, point_dst);
        }catch (tf::TransformException &ex){}

        it->x = point_dst.point.x;
        it->y = point_dst.point.y;
        it->z = point_dst.point.z;

        res.push_back(*it);
    }
    return res;
}
void PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{ 
	pcl::PointCloud<pcl::PointXYZ> inputPclCloud;
    pcl::fromROSMsg(*input, inputPclCloud);

    inputPclCloud=Cloud_Transform(inputPclCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);// 设置滤波的输入点云数据

	pcl::PassThrough<pcl::PointXYZ> pass;  
	pass.setInputCloud (inputPclCloud.makeShared()); 
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (box_z_min, box_z_max);
	pass.setFilterLimitsNegative (false);//设置点云中所有点的z坐标不在该范围内的点过滤掉或保留;false:保留;true:过滤
	pass.filter (*filteredCloud); 

	pass.setInputCloud (filteredCloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (box_y_min, box_y_max);
	pass.setFilterLimitsNegative (false);
	pass.filter (*filteredCloud);

	pass.setInputCloud (filteredCloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (box_x_min, box_x_max);
	pass.setFilterLimitsNegative (false);
	pass.filter (*filteredCloud);


    // 再将滤波后的点云的数据格式转换为ROS 下的数据格式发布出去
    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2); //声明的输出的点云的格式
    pcl::toROSMsg (*filteredCloud , *output);

    output->header.frame_id ="base_link"; //input->header.frame_id;
    output->header.stamp = ros::Time::now();
    pub.publish (*output);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle n;
	ros::NodeHandle nh("~");

    tf_listener = new tf::TransformListener;

    nh.getParam("box_x_max", box_x_max);
    nh.getParam("box_x_min", box_x_min);
    nh.getParam("box_y_max", box_y_max);
    nh.getParam("box_y_min", box_y_min);
    nh.getParam("box_z_max", box_z_max);
    nh.getParam("box_z_min", box_z_min);	

    ros::Subscriber cloud_sub = n.subscribe("/velodyne_points", 1, PointcloudCallback);
    pub = n.advertise<sensor_msgs::PointCloud2> ("output_cloud", 1);

    ros::spin();
}