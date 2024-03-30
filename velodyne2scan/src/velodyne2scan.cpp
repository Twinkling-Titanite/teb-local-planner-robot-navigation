#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>

class VelodyneToScan
{
public:
    VelodyneToScan()
    {
        velodyne_sub_ = nh_.subscribe("/output_points", 1, &VelodyneToScan::velodyneCallback, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
    }

    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        sensor_msgs::LaserScan scan;
        scan.header = msg->header;
        scan.angle_min = -M_PI / 2;
        scan.angle_max = M_PI / 2;
        scan.angle_increment = M_PI / 360; // 1 degree resolution
        scan.time_increment = 0.0;
        scan.range_min = 0.0;
        scan.range_max = 100.0; // Adjust according to your requirements

        std::vector<float> ranges;
        for (const auto &point : cloud->points)
        {
            // Calculate range from point to origin
            float range = std::sqrt(point.x * point.x + point.y * point.y);
            ranges.push_back(range);
        }

        scan.ranges = ranges;

        scan_pub_.publish(scan);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber velodyne_sub_;
    ros::Publisher scan_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne2scan");
    VelodyneToScan velodyne_to_scan;

    ros::spin();

    return 0;
}
