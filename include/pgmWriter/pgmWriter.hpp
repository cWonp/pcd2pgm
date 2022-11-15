#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/conditional_removal.h>

#include <iostream>
#include <fstream>
#include <vector>

typedef pcl::PointXYZ point;
typedef pcl::PointCloud<point>::Ptr pointCloudPtr;
typedef pcl::PointCloud<point> pointCloud;

class pgmWriter
{
private:
public:
    pointCloudPtr pcdMap;

    std::string pcdFileName;
    std::string pgmFileName;

    // Map height and width
    int margin_m = 5; // meter margin to create pgm map
    double resolution = 0.05; // meter/pixel

    // Floor and wall range
    float floorZmin = -0.2;
    float floorZmax = 0.2;
    float collisionZmin = 0.3;
    float collisionZmax = 1.2;

    float searchRadius = 0.27; // octree search radius

    uint8_t freeSpace = 254; // gray pixel
    uint8_t unKnownSpace = 205; // gray pixel
    uint8_t collisionSpace = 0; // black pixel

    pgmWriter(){};
    ~pgmWriter(){};

    int setPcdMap(std::string mapfileName);
    void zAxisConditionalFilter(pointCloudPtr inputCloud, double zMin, double zMax, pointCloudPtr outputCloud);
    int pcdToPgm(pointCloudPtr pcdFile, const std::string& pgmFileName);

    void run();
};
