/*
 * @Description: ICP2P 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP2P_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP2P_REGISTRATION_HPP_
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class ICP2PRegistration: public RegistrationInterface {
  public:
    ICP2PRegistration(const YAML::Node& node);
    ICP2PRegistration(int MaximumIterations);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(int MaximumIterations);

  private:
    void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	       pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>::Ptr icp2p;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source_normals ;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target_normals ;
 
};
}
#endif



