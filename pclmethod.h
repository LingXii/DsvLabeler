#ifndef PCLMETHOD_H
#define PCLMETHOD_H
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

pcl::PointCloud<pcl::PointXYZL>::Ptr LCCP_seg(string s_time);

#endif // PCLMETHOD_H
