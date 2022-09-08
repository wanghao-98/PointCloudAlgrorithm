#include "NormalEstimationUser.h"

void pcl::NormalEstimationUser::computePointNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int index, 
	pcl::Normal &normal)
{	
	std::vector<int> indice;
	std::vector<float> dis;
	tree_->radiusSearch(cloud->points[index], search_radius_, indice, dis);
	Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
	computeCovarianceMatrix(*cloud, indice, dis, cov); 
	pcl::solvePlaneParameters(cov.cast<float>(), normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
}

unsigned int pcl::NormalEstimationUser::computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ> &cloud,
	const std::vector<int> &indices, const std::vector<float> &dis,
	Eigen::Matrix<double, 3, 3> &covariance_matrix)
{
	if (indices.empty())
		return (0);
	// Initialize to 0
	covariance_matrix.setZero();
	Eigen::Matrix<double, 4, 1> centroid;
	pcl::compute3DCentroid(cloud, indices, centroid);
	size_t point_count;
	double dis_sum =0;
	// If the data is dense, we don't need to check for NaN
	point_count = indices.size();
	// For each point in the cloud
	for (size_t i = 0; i < point_count; ++i)
	{
		double distance = search_radius_ -std::sqrt( dis[i]);
		dis_sum += distance;
		covariance_matrix += (distance*(cloud[indices[i]].getVector3fMap().cast<double>() - centroid.head(3))*
			(cloud[indices[i]].getVector3fMap().cast<double>() - centroid.head(3)).transpose());
	}
	covariance_matrix /= dis_sum;
	return (static_cast<unsigned int> (point_count));
}

void pcl::NormalEstimationUser::computeFeature(pcl::PointCloud<pcl::Normal> &normal)
{
	normal.resize(input_->size());
	tree_->setInputCloud(input_);
	for (size_t i = 0; i < input_->size(); i++)
	{
		computePointNormal(input_, i, normal.points[i]);
		flipNormalTowardsViewpoint(input_->points[i], vpx_, vpy_, vpz_,
			normal.points[i].normal_x, normal.points[i].normal_y, normal.points[i].normal_z);
	}
}




