#pragma once

#include <pcl/point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\common\centroid.h>

enum NUMOFCOL {
	THREE = 3,
	FOUR = 4
};

//加载txt数据
inline void loadTXTFile(const std::string &path, const NUMOFCOL &clos, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
#pragma region Reader5
	//40s,824M,20M/s,针对txt数据中含有四列的数据
	pcl::PointXYZ txtPoint;
	//将标准输入重定向到文件
	freopen(path.c_str(), "r", stdin);
	//判断txt文件中的烈属
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
	//将标准输入重定向到输入窗口
	freopen("CON", "r", stdin);
#pragma endregion

}

//保存为txt数据
inline void saveTXTFile(const std::string &path, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	//40s,824M,20M/s,针对txt数据中含有四列的数据
	//将标准输出重定向到文件
	freopen(path.c_str(), "w", stdout);

	for (size_t i = 0; i < cloud->size(); i++)
	{
		printf("%.4f %.4f %.4f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}
	//重定向到标准输出，保证后续输出正常
	freopen("CON", "w", stdout);
}

//保存txt数据，根据索引值进行保存
inline void saveTXTFile(const std::string &path, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
	const std::vector<int> &indices)
{
	//40s,824M,20M/s,针对txt数据中含有四列的数据
	freopen(path.c_str(), "w", stdout);

	for (size_t i = 0; i < indices.size(); i++)
	{
		printf("%.4f %.4f %.4f\n", cloud->points[indices[i]].x, cloud->points[indices[i]].y, cloud->points[indices[i]].z);
	}
	freopen("CON", "w", stdout);
}

