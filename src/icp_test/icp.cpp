#include <iostream>
#include <string>
#include "icp.h"


inline std::vector<Eigen::Vector3d> cloud2vector(int N,PointCloudT cloud){
  std::vector<Eigen::Vector3d> cloud_;
  for (int i=0;i<N;i++){
    cloud_.push_back(Eigen::Vector3d(cloud[i].x,cloud[i].y,cloud[i].z));
  }
  cout<<"finishing trans"<<endl;
  return cloud_;
}


int main (int argc,
      char** argv)
{
  
  pcl::console::TicToc time;     
  time.tic ();    
  //read cloud
  PointCloudT::Ptr cloud_1 (new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>("../data/key_frame_0.pcd", *cloud_1 ) == -1)
    {
        PCL_ERROR("Couldn't read file 0.pcd \n");
        return (-1);
    }
/*
  pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_1->points.begin();
  while (it != cloud_1->points.end())
   {
        float x, y, z;
	x = it->x;
	y = it->y;
	z = it->z;
	if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z) )
	{
		it = cloud_1->points.erase(it);
	}
	else
		++it;
  }*/
  int N1=cloud_1->size();
  std::cout << "Loaded " << N1 << " data points from 0.pcd" << std::endl;

  PointCloudT::Ptr cloud_2(new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>("../data/key_frame_4.pcd", *cloud_2) == -1)
    {
        PCL_ERROR("Couldn't read file 1.pcd \n");
        return (-1);
    }
/*
  pcl::PointCloud<pcl::PointXYZ>::iterator it1 = cloud_2->points.begin();
  while (it1 != cloud_2->points.end())
   {
        float x, y, z;
	x = it1->x;
	y = it1->y;
	z = it1->z;
	if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z) )
	{
		it1 = cloud_2->points.erase(it1);
	}
	else
		++it1;
  }*/
  int N2=cloud_2->size();
  std::cout << "Loaded " << N2 << " data points from 1.pcd" << std::endl;
  int N=std::min(N1,N2);
  
#if 0
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);	
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>icp;
  icp.setInputSource (cloud_2);   
  icp.setInputTarget (cloud_1);    
  icp.setMaxCorrespondenceDistance(1.0);  
  icp.setTransformationEpsilon(0.01); 
  icp.setEuclideanFitnessEpsilon(0.01); 
  icp.setMaximumIterations (50);  
  Eigen::AngleAxisf init_rotation(1, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0.1, 0.1, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  icp.align(*cloud_icp,init_guess);
  cout<<icp.getFinalTransformation()<<endl;
  Eigen::Affine3f t;
  Eigen::Matrix4f a;
  a=icp.getFinalTransformation();
  t.matrix() = a;
#endif
  
#if 1
  std::vector<Eigen::Vector3d> match_cloud1,match_cloud2;
  match_cloud1=cloud2vector(N1,*cloud_1);
  match_cloud2=cloud2vector(N2,*cloud_2);
  Eigen::Matrix3d R_=Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_( 0.1,0.1,0.0 );
  findTransformation ( match_cloud1, match_cloud2, 100, 1.0e-4, 0.01, R_, t_ );
  Eigen::Matrix4f T;
  T<<R_(0,0),R_(0,1),R_(0,2),t_(0),
     R_(1,0),R_(1,1),R_(1,2),t_(1),
     R_(2,0),R_(2,1),R_(2,2),t_(2),
     0,0,0,1;

  cout<<"Trans_Matrix="<<T<<endl;
  Eigen::Affine3f t;
  t.matrix() = T;
  PointCloudT::Ptr cloud_icp(new PointCloudT);
  pcl::transformPointCloud(*cloud_1,*cloud_icp,t); 
#endif 
  
  
  double error=computer_error(N,*cloud_icp,*cloud_2);
  time.toc ();        
  cout<<"cost"<<time.toc ()<<"ms"<<endl;
  
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // define two viewer
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // define white&blank
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // cloud_1==white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_1,  255, 255,
                                                                               255);
  viewer.addPointCloud (cloud_1, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_1, cloud_in_color_h, "cloud_in_v2", v2);

  // cloud_2==green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_2, 0, 255, 0);
  viewer.addPointCloud (cloud_2, cloud_tr_color_h, "cloud_tr_v1", v1);

  // cloud_icp_color==red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 255, 0, 0);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
  //viewer.addCoordinateSystem (1.0,0,0,0,"cloud_icp_v2",v2);
  viewer.addCoordinateSystem (1.0,0,0,0,"cloud_in_v2",v2);
  viewer.addCoordinateSystem (1.0,0,0,0,"cloud_in_v1",v1);
  viewer.addCoordinateSystem (1.0,0,0,0,"cloud_tr_v1",v1);
  viewer.addCoordinateSystem (1.0,t,"cloud_icp_v2",v2);


 
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
   boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    
  }
  return (0);
}
