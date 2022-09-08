#include "NormalEstimateUser.h"

NormalEstimationUser::NormalEstimationUser():cloud(),r(),tree(new pcl::KdTreeFLANN<pcl::PointXYZ>)
{
	
}

NormalEstimationUser::~NormalEstimationUser()
{
}


void NormalEstimationUser::computePointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index, pcl::Normal &normal)
{
	tree->setInputCloud(cloud);
	std::vector<int> indice;
	std::vector<float> dis;
	tree->radiusSearch(cloud->points[index], r, indice, dis);
	Eigen::Matrix3f cov=Eigen::Matrix3f::Identity();
	computeCovarianceMatrix(*cloud, indice, dis, cov);

	pcl::solvePlaneParameters(cov, normal.normal_x,normal.normal_y,normal.normal_z,normal.curvature);

}

 unsigned int NormalEstimationUser::computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ> &cloud,
	const std::vector<int> &indices,const std::vector<float> &dis,	
	Eigen::Matrix<float, 3, 3> &covariance_matrix)
{
	if (indices.empty())
		return (0);

	// Initialize to 0
	covariance_matrix.setZero();
	Eigen::Matrix<float, 4, 1> centroid;
	pcl::compute3DCentroid(cloud, indices, centroid);
	size_t point_count;
	float dis_sum;
	// If the data is dense, we don't need to check for NaN
	if (cloud.is_dense)
	{
		point_count = indices.size();
		// For each point in the cloud
		for (size_t i = 0; i < point_count; ++i)
		{
			Eigen::Matrix<float, 4, 1> pt;
			pt[0] = cloud[indices[i]].x - centroid[0];
			pt[1] = cloud[indices[i]].y - centroid[1];
			pt[2] = cloud[indices[i]].z - centroid[2];

			covariance_matrix(1, 1) += pt.y() * pt.y();
			covariance_matrix(1, 2) += pt.y() * pt.z();

			covariance_matrix(2, 2) += pt.z() * pt.z();

			pt *= pt.x();
			covariance_matrix(0, 0) += pt.x();
			covariance_matrix(0, 1) += pt.y();
			covariance_matrix(0, 2) += pt.z();
		}
	}
	// NaN or Inf values could exist => check for them
	else
	{
		point_count = 0;
		// For each point in the cloud
		for (size_t i = 0; i < indices.size(); ++i)
		{
			// Check if the point is invalid
			if (!isFinite(cloud[indices[i]]))
				continue;

			float distance =std::sqrt( r*r - dis[i]);

			dis_sum += distance;
			Eigen::Matrix<float, 4, 1> pt;
			pt[0] = distance*cloud[indices[i]].x - centroid[0];
			pt[1] = distance*cloud[indices[i]].y - centroid[1];
			pt[2] = distance*cloud[indices[i]].z - centroid[2];

			covariance_matrix(1, 1) += pt.y() * pt.y();
			covariance_matrix(1, 2) += pt.y() * pt.z();

			covariance_matrix(2, 2) += pt.z() * pt.z();

			pt *= pt.x();
			covariance_matrix(0, 0) += pt.x();
			covariance_matrix(0, 1) += pt.y();
			covariance_matrix(0, 2) += pt.z();
			++point_count;
		}
	}
	covariance_matrix(1, 0) = covariance_matrix(0, 1);
	covariance_matrix(2, 0) = covariance_matrix(0, 2);
	covariance_matrix(2, 1) = covariance_matrix(1, 2);

	covariance_matrix /= dis_sum;
	return (static_cast<unsigned int> (point_count));
}

void NormalEstimationUser::compute(pcl::PointCloud<pcl::Normal> &normal)
{
	normal.resize(cloud->size());
	for (size_t i = 0; i < cloud->size(); i++)
	{
		computePointNormal(cloud, i, normal.points[i]);
	}
}


