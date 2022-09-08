#pragma once
#include<pcl\features\normal_3d.h>


namespace pcl {


	
template<typename PointInT,typename PointOutT>
class  NormalUser:public NormalEstimation<PointInT, PointOutT>
{
public:
	
	//这里只是VS智能提示不显示，代码本身没有问题
	NormalEstimation::vpx_;
	//NormalUser():tree(new pcl::KdTreeFLANN<PointInT>) {};
	//~NormalUser() {};
	//unsigned int computeCovarianceMatrix(const pcl::PointCloud<PointInT> &cloud,
	//	const std::vector<int> &indices, const std::vector<float> &dis,
	//	Eigen::Matrix<float, 3, 3> &covariance_matrix);

	//inline bool
	//	computePointNormal(const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
	//		const std::vector<float> &dis,float &nx, float &ny, float &nz, float &curvature)
	//{
	//	if (indices.size() < 3 ||
	//		computeCovarianceMatrix(cloud, indices, dis,covariance_matrix_) == 0)
	//	{
	//		nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN();
	//		return false;
	//	}

	//	// Get the plane normal and surface curvature
	//	solvePlaneParameters(covariance_matrix_, nx, ny, nz, curvature);
	//	return true;
	//}

	//void test() {
	//	std::cout << "sucess" << std::endl;
	//};

	//void computeNormals(PointCloudOut &output);

	//pcl::KdTreeFLANN<PointInT> tree;
};
}







