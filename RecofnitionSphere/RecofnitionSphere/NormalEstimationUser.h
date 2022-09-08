#pragma once
#include <pcl/features/normal_3d.h>

namespace pcl {
class NormalEstimationUser :
	public NormalEstimation<pcl::PointXYZ,pcl::Normal>
{
public:
	NormalEstimationUser():NormalEstimation<pcl::PointXYZ, pcl::Normal>()
	{
		//此时会先调用派生类的构造函数，如何自动调用基类的默认构造函数，
		//也可以通过初始化列表初始化一个基类对象，不能直接对基类中的数据成员进行处理
		tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(true));
		//这里为什么没有错误，VS反应不正常，有时能智能提示基类成员，有时不能，有鬼一般
		feature_name_ = "NormalEstimationUser";
	};

	~NormalEstimationUser() {
	}
	
	unsigned int computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ> &cloud,
		const std::vector<int> &indices, const std::vector<float> &dis,
		Eigen::Matrix<double, 3, 3> &covariance_matrix);
	void computePointNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int index, pcl::Normal &normal);
protected:
	void computeFeature(pcl::PointCloud<pcl::Normal> &normal);

};

}