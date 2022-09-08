#pragma once
#include <pcl/features/normal_3d.h>

namespace pcl {
class NormalEstimationUser :
	public NormalEstimation<pcl::PointXYZ,pcl::Normal>
{
public:
	NormalEstimationUser():NormalEstimation<pcl::PointXYZ, pcl::Normal>()
	{
		//��ʱ���ȵ���������Ĺ��캯��������Զ����û����Ĭ�Ϲ��캯����
		//Ҳ����ͨ����ʼ���б��ʼ��һ��������󣬲���ֱ�ӶԻ����е����ݳ�Ա���д���
		tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(true));
		//����Ϊʲôû�д���VS��Ӧ����������ʱ��������ʾ�����Ա����ʱ���ܣ��й�һ��
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