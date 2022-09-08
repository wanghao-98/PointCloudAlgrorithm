#include "NormalUser.h"

//template<typename PointInT, typename PointOutT>
//unsigned int NormalUser<PointInT, PointOutT>::computeCovarianceMatrix(const pcl::PointCloud<PointInT> &cloud,
//	const std::vector<int> &indices, const std::vector<float> &dis,
//	Eigen::Matrix<float, 3, 3> &covariance_matrix)
//{
//	if (indices.empty())
//		return (0);
//
//	// Initialize to 0
//	covariance_matrix.setZero();
//	Eigen::Matrix<float, 4, 1> centroid;
//	pcl::compute3DCentroid(cloud, indices, centroid);
//	size_t point_count;
//	float dis_sum;
//	// If the data is dense, we don't need to check for NaN
//	if (cloud.is_dense)
//	{
//		point_count = indices.size();
//		// For each point in the cloud
//		for (size_t i = 0; i < point_count; ++i)
//		{
//			Eigen::Matrix<float, 4, 1> pt;
//			pt[0] = cloud[indices[i]].x - centroid[0];
//			pt[1] = cloud[indices[i]].y - centroid[1];
//			pt[2] = cloud[indices[i]].z - centroid[2];
//
//			covariance_matrix(1, 1) += pt.y() * pt.y();
//			covariance_matrix(1, 2) += pt.y() * pt.z();
//
//			covariance_matrix(2, 2) += pt.z() * pt.z();
//
//			pt *= pt.x();
//			covariance_matrix(0, 0) += pt.x();
//			covariance_matrix(0, 1) += pt.y();
//			covariance_matrix(0, 2) += pt.z();
//		}
//	}
//	// NaN or Inf values could exist => check for them
//	else
//	{
//		point_count = 0;
//		// For each point in the cloud
//		for (size_t i = 0; i < indices.size(); ++i)
//		{
//			// Check if the point is invalid
//			if (!isFinite(cloud[indices[i]]))
//				continue;
//
//			float distance = search_radius_ - dis[i];
//
//			dis_sum += distance;
//			Eigen::Matrix<float, 4, 1> pt;
//			pt[0] = distance*cloud[indices[i]].x - centroid[0];
//			pt[1] = distance*cloud[indices[i]].y - centroid[1];
//			pt[2] = distance*cloud[indices[i]].z - centroid[2];
//
//			covariance_matrix(1, 1) += pt.y() * pt.y();
//			covariance_matrix(1, 2) += pt.y() * pt.z();
//
//			covariance_matrix(2, 2) += pt.z() * pt.z();
//
//			pt *= pt.x();
//			covariance_matrix(0, 0) += pt.x();
//			covariance_matrix(0, 1) += pt.y();
//			covariance_matrix(0, 2) += pt.z();
//			++point_count;
//		}
//	}
//	covariance_matrix(1, 0) = covariance_matrix(0, 1);
//	covariance_matrix(2, 0) = covariance_matrix(0, 2);
//	covariance_matrix(2, 1) = covariance_matrix(1, 2);
//
//	covariance_matrix /= dis_sum;
//	return (static_cast<unsigned int> (point_count));
//}
//
//
//
//template <typename PointInT, typename PointOutT> void
//NormalUser<PointInT, PointOutT>::computeNormals(PointCloudOut &output)
//{
//	tree.setInputCloud(input_);
//	// Allocate enough space to hold the results
//	// \note This resize is irrelevant for a radiusSearch ().
//	std::vector<int> nn_indices(k_);
//	std::vector<float> nn_dists(k_);
//
//	output.is_dense = true;
//	// Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
//	if (input_->is_dense)
//	{
//		// Iterating over the entire index vector
//		for (size_t idx = 0; idx < indices_->size(); ++idx)
//		{
//			if (tree.radiusSearch((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
//				!computePointNormal(*surface_, nn_indices,nn_dists, output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature))
//			{
//				output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN();
//
//				output.is_dense = false;
//				continue;
//			}
//
//			flipNormalTowardsViewpoint(input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
//				output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);
//
//		}
//	}
//	else
//	{
//		// Iterating over the entire index vector
//		for (size_t idx = 0; idx < indices_->size(); ++idx)
//		{
//			if (!isFinite((*input_)[(*indices_)[idx]]) ||
//				this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
//				!computePointNormal(*surface_, nn_indices, nn_dists,output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature))
//			{
//				output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN();
//
//				output.is_dense = false;
//				continue;
//			}
//
//			flipNormalTowardsViewpoint(input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
//				output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);
//
//		}
//	}
//}
