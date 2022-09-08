#ifndef PCL_NORMAL_3D_USER_H_
#define PCL_NORMAL_3D_USER_H_

#include<pcl\features\normal_3d.h>

class NormalEstimationUser
{
	

public:
	NormalEstimationUser();
	~NormalEstimationUser();
	unsigned int computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ> &cloud,
		const std::vector<int> &indices, const std::vector<float> &dis,
		Eigen::Matrix<float, 3, 3> &covariance_matrix);
	void computePointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index, pcl::Normal &normal);
	void compute(pcl::PointCloud<pcl::Normal> &normal);
	inline void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud)
	{
		cloud = _cloud;
	}
	inline void setRadiusSearch(float _r)
	{
		r = _r;
	}

	float r;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree;


};

#endif

