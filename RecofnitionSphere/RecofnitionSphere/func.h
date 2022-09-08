#pragma once

#include <pcl/point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\common\centroid.h>

enum NUMOFCOL {
	THREE = 3,
	FOUR = 4
};

//����txt����
inline void loadTXTFile(const std::string &path, const NUMOFCOL &clos, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
#pragma region Reader5
	//40s,824M,20M/s,���txt�����к������е�����
	pcl::PointXYZ txtPoint;
	//����׼�����ض����ļ�
	freopen(path.c_str(), "r", stdin);
	//�ж�txt�ļ��е�����
	switch (clos)
	{
	case NUMOFCOL::THREE:
		while (scanf("%f %f %f", &txtPoint.x, &txtPoint.y, &txtPoint.z) != EOF)
		{
			cloud->push_back(txtPoint);
		}
	case NUMOFCOL::FOUR:
		float a;
		while (scanf("%f %f %f %f", &txtPoint.x, &txtPoint.y, &txtPoint.z, &a) != EOF)
		{
			cloud->push_back(txtPoint);
		}
	default:
		break;
	}
	//����׼�����ض������봰��
	freopen("CON", "r", stdin);
#pragma endregion

}

//����Ϊtxt����
inline void saveTXTFile(const std::string &path, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	//40s,824M,20M/s,���txt�����к������е�����
	//����׼����ض����ļ�
	freopen(path.c_str(), "w", stdout);

	for (size_t i = 0; i < cloud->size(); i++)
	{
		printf("%.4f %.4f %.4f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}
	//�ض��򵽱�׼�������֤�����������
	freopen("CON", "w", stdout);
}

//����txt���ݣ���������ֵ���б���
inline void saveTXTFile(const std::string &path, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const std::vector<int> &indices)
{
	//40s,824M,20M/s,���txt�����к������е�����
	freopen(path.c_str(), "w", stdout);

	for (size_t i = 0; i < indices.size(); i++)
	{
		printf("%.4f %.4f %.4f\n", cloud->points[indices[i]].x, cloud->points[indices[i]].y, cloud->points[indices[i]].z);
	}
	freopen("CON", "w", stdout);
}

