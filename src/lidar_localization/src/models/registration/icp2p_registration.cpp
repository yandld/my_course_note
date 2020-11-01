/*
 * @Description: ICP 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/icp2p_registration.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include "glog/logging.h"

namespace lidar_localization {

ICP2PRegistration::ICP2PRegistration(const YAML::Node& node)
    :icp2p(new pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> ()),
    cloud_source_normals( new pcl::PointCloud<pcl::PointNormal> () ) ,
    cloud_target_normals( new pcl::PointCloud<pcl::PointNormal> () )
{
    
    int MaximumIterations = node["MaximumIterations"].as<int>();

    ICP2PRegistration::SetRegistrationParam(MaximumIterations);
}

ICP2PRegistration::ICP2PRegistration(int MaximumIterations)
    :icp2p(new pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> ()) ,
    cloud_source_normals( new pcl::PointCloud<pcl::PointNormal> () ) ,
    cloud_target_normals( new pcl::PointCloud<pcl::PointNormal> () )
{

    ICP2PRegistration::SetRegistrationParam(MaximumIterations);
}

bool ICP2PRegistration::SetRegistrationParam(int MaximumIterations) {
    icp2p->setMaximumIterations ( MaximumIterations );

    LOG(INFO) << "ICP2P 的匹配参数为：" << std::endl
              << "MaximumIterations: " << MaximumIterations 
              << std::endl << std::endl;

    return true;
}

bool ICP2PRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    addNormal( input_target, cloud_target_normals );
    icp2p->setInputTarget(cloud_target_normals);
    return true;
}

bool ICP2PRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    addNormal( input_source, cloud_source_normals );
    icp2p->setInputSource(cloud_source_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<pcl::PointNormal> () );
    icp2p->align(*cloud_source_trans_normals,predict_pose);
    result_pose = icp2p->getFinalTransformation();
    pcl::transformPointCloud ( *input_source, *result_cloud_ptr, result_pose );
    return true;
}




void ICP2PRegistration::addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	       pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals
)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );

  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud ( cloud );

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  normalEstimator.setKSearch (50 );
  normalEstimator.compute ( *normals );
  
  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}


}