#ifndef _ICP_H_
#define _ICP_H_

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>    
#include <pcl/point_types.h>  
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


inline double computer_error(int N,PointCloudT cloud1,PointCloudT cloud2){
  double error(0);
  for (int i=0;i<N;i++){
    error+=sqrt(pow((cloud1[i].x-cloud2[i].x),2)+pow((cloud1[i].y-cloud2[i].y),2)+pow((cloud1[i].z-cloud2[i].z),2));
  }
  error=error/N;
  cout<<"error="<<error<<endl;
}


bool findTransformation ( const std::vector< Eigen::Vector3d >& pts_model, const std::vector< Eigen::Vector3d >& pts_cloud, double n_iters, double epsilon, double min_err, Eigen::Matrix3d& R, Eigen::Vector3d& t )
{
	// default settings.
	const double min_err2 = min_err * min_err;
	const double factor = 9.0;
	const int n_selected_pts = 500;
	const int step = pts_cloud.size() / n_selected_pts;	// step for select points.
	
	// two vectors for matched points.
    std::vector<Eigen::Vector3d> pts_cloud_matched;
	pts_cloud_matched.reserve ( n_selected_pts );
    std::vector<Eigen::Vector3d> pts_model_matched;
	pts_model_matched.reserve ( n_selected_pts );

    // construct kd-tree for model cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    for ( size_t i = 0; i < pts_model.size(); ++i ) {
        const Eigen::Vector3d& ptm = pts_model.at ( i );
        pcl::PointXYZ pt ( ptm[0], ptm[1], ptm[2] );
        model_cloud->push_back ( pt );
    }
    pcl::KdTreeFLANN<pcl::PointXYZ>* kd_tree = new pcl::KdTreeFLANN <pcl::PointXYZ>();
    kd_tree->setInputCloud ( model_cloud );

	// used for search.
	std::vector <int> index (1);
	std::vector <float> squared_distance (1);
	
	// Dth
	double squared_distance_th = std::numeric_limits <double>::max ();
	double cur_squared_dist = 0.0;
	double last_squared_dist = std::numeric_limits<double>::max();
	
    //for n_iters
    for ( int n = 0; n < n_iters; n ++ ) {
		// clear two point clouds.
		pts_cloud_matched.clear();
		pts_model_matched.clear();
        // step 1. construct matched point clouds.
		double sum_squared_dist = 0.0;
		for ( size_t i = 0; i < pts_cloud.size(); i += step ) {	
            // transformed by T.
            Eigen::Vector3d pt = R *  pts_cloud.at ( i ) + t;
            // find the nearest points
			pcl::PointXYZ pt_d(pt[0], pt[1], pt[2]);
			if (!kd_tree->nearestKSearch (pt_d, 1, index, squared_distance))
			{
				std::cerr << "ERROR: no points found.\n";
				return false;
			}
			
			if(squared_distance[0] < squared_distance_th)
			{
				// add squared distance.
				sum_squared_dist += squared_distance[0];
				// add the pt in cloud.
				pts_cloud_matched.push_back(pts_cloud.at(i));
				// add the pt in model.
				pts_model_matched.push_back(pts_model.at(index[0]));
			}
        } // for all pts_cloud


        //std::cout << "iter:" << n << " mathced size: " << pts_model_matched.size() << " " << pts_cloud_matched.size() << std::endl;
        
        // step 2. Get R and t.
        // step 2.1 find centor of model(X) and cloud(P)
        Eigen::Vector3d mu_x(0.0, 0.0, 0.0);
		Eigen::Vector3d mu_p(0.0, 0.0, 0.0);
        for(size_t i = 0; i < pts_cloud_matched.size(); i ++)
		{
			mu_x += pts_model_matched.at(i);
			mu_p += pts_cloud_matched.at(i);
		}
        mu_x = mu_x / double(pts_model_matched.size());
		mu_p = mu_p / double(pts_cloud_matched.size());
		
		// step 2.2 Get W.
		
		Eigen::Matrix3d W;
		for(size_t i = 0; i < pts_cloud_matched.size(); i ++)
		{
			W += (pts_model_matched.at(i)-mu_x) * ( (pts_cloud_matched.at(i)-mu_p).transpose() );
		}
		
		// step 2.3 Get R
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		
		R = svd.matrixU() * (svd.matrixV().transpose());
		
		// step 2.4 Get t
		t  = mu_x - R * mu_p;
		
		// step 3. Check if convergenced.
		cur_squared_dist = sum_squared_dist / (double)pts_cloud_matched.size();
		double squared_dist_change = last_squared_dist -cur_squared_dist;
		
		//std::cout << "iter:" << n << " squared_dist_change: " << squared_dist_change << " cur distance " << cur_squared_dist  << std::endl;
		
		if(squared_dist_change < epsilon || cur_squared_dist < min_err2)
		{
			break;
		}
		last_squared_dist = cur_squared_dist;
		squared_distance_th = factor * cur_squared_dist;
		
    } // for n_iters
    
    delete kd_tree;
    return true;
} // findTransformation



#endif
