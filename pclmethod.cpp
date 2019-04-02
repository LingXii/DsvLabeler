#include "DsvLabelMv.h"
#include "pclmethod.h"
#include <cstdlib>
#include <cmath>
#include <string>
#include <limits.h>
#include <boost/format.hpp>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

typedef pcl::LCCPSegmentation<pcl::PointXYZRGBA>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

// this code is from https://blog.csdn.net/wi162yyxq/article/details/72823712
pcl::PointCloud<pcl::PointXYZL>::Ptr LCCP_seg(string s_time)
{
    //输入点云
    pcl::PointCloud<pcl::PointXYZRGBA> input_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>("pcd/" + s_time + ".pcd", *input_pc);
    input_cloud.width = LINES_PER_BLK*BKNUM_PER_FRM;
    input_cloud.height = PNTS_PER_LINE;
    input_cloud.is_dense = false;
    input_cloud.points.resize(input_cloud.width * input_cloud.height);
    for(int i=0; i<LINES_PER_BLK*BKNUM_PER_FRM*PNTS_PER_LINE; i++)
    {
        input_cloud.points[i].x = input_pc->points[i].x;
        input_cloud.points[i].y = input_pc->points[i].y;
        input_cloud.points[i].z = input_pc->points[i].z;
    }
    input_cloud_ptr = input_cloud.makeShared();

    //超体聚类 参数依次是粒子距离、晶核距离、颜色容差、
    float voxel_resolution = 0.3f;
    float seed_resolution = 1.5f;
    float color_importance = 0.0f;
    float spatial_importance = 1.0f;
    float normal_importance = 0.0f;
    bool use_single_cam_transform = false;

    unsigned int k_factor = 0;

    //voxel_resolution is the resolution (in meters) of voxels used、seed_resolution is the average size of resulting supervoxels
    pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(use_single_cam_transform);
    super.setInputCloud(input_cloud_ptr);
    //Set the importance of color for supervoxels.
    super.setColorImportance(color_importance);
    //Set the importance of spatial distance for supervoxels.
    super.setSpatialImportance(spatial_importance);
    //Set the importance of scalar normal product for supervoxels.
    super.setNormalImportance(normal_importance);
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;

    super.extract(supervoxel_clusters);
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZRGBA>::makeSupervoxelNormalCloud(supervoxel_clusters);

    //LCCP分割
    float concavity_tolerance_threshold = 10;
    float smoothness_threshold = 0.1;
    uint32_t min_segment_size = 0;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = false;
    pcl::LCCPSegmentation<pcl::PointXYZRGBA> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor(k_factor);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);
    SuperVoxelAdjacencyList sv_adjacency_list;
    lccp.getSVAdjacencyList(sv_adjacency_list);

    string name = "LCCP_pcd/" + s_time + ".pcd";
    pcl::io::savePCDFileASCII(name,*lccp_labeled_cloud);
    return lccp_labeled_cloud;
}
