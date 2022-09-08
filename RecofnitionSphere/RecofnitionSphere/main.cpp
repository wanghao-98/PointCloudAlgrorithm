#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl\features\shot_lrf.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl\registration\correspondence_estimation.h>
#include <pcl\segmentation\extract_clusters.h>
#include <pcl\filters\extract_indices.h>
#include <pcl\common\time.h>
#include <math.h>

#include "NormalEstimationUser.h"
//#include "NormalEstimateUser.h"
#include "NormalUser.h"
#include "func.h"


//typedef pcl::PointXYZRGBA PointType;
//typedef pcl::Normal NormalType;
//typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;

using PointType = pcl::PointXYZ;
using NormalType = pcl::Normal;
using RFType = pcl::ReferenceFrame;
using DescriptorType = pcl::SHOT352;

#define FLT_EPSILON      1.192092896e-07F        // smallest such that 1.0+FLT_EPSILON != 1.0
#define ZERO_TOLERANCE static_cast<double>(FLT_EPSILON)
#define r_reciprocal  1 / 0.0725

//最小二乘拟合
void LSSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f &sphereCenter, float &r, float &sigma, float &mse)
{
	Eigen::Matrix<double, 4, 1> X;
	Eigen::MatrixXd B(cloud->size(), 4);
	Eigen::MatrixXd L(cloud->size(), 1);
	for (size_t i = 0; i < cloud->size(); i++)
	{
		B(i, 0) = cloud->points[i].x;
		B(i, 1) = cloud->points[i].y;
		B(i, 2) = cloud->points[i].z;
		B(i, 3) = 1;
		L(i, 0) = cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z;
	}
	X = (B.transpose()*B).inverse()*B.transpose()*L;

	//Eigen::MatrixXd V(cloud->size(), 1);
	//V = B*X - L;
	//Eigen::Matrix<double, 1, 1> sigma_;
	//sigma_ = V.transpose()*V;
	//sigma = std::sqrt(sigma_(0, 0) / (cloud->size() - 3));

	//std::cout << sigma << std::endl;
	//Eigen::VectorXd sigma = (V.transpose()*V);
	//sigma = (sigma / (cloud->size() - 3));
	sphereCenter(0) = X(0) / 2;
	sphereCenter(1) = X(1) / 2;
	sphereCenter(2) = X(2) / 2;

	r = std::sqrt(X(3) + sphereCenter(0)*sphereCenter(0) + sphereCenter(1)*sphereCenter(1) + sphereCenter(2)*sphereCenter(2));
	//pcl::PointCloud<pcl::PointXYZI>::Ptr idensity(new pcl::PointCloud<pcl::PointXYZI>);

	float error = 0;
	for (size_t i = 0; i < cloud->size(); i++)
	{
		float err = (cloud->points[i].getVector3fMap() - sphereCenter).norm() - r;
		//pcl::PointXYZI pp;
		//pp.x = cloud->points[i].x;
		//pp.y = cloud->points[i].y;
		//pp.z = cloud->points[i].z;
		//pp.intensity = err*1000;
		//idensity->push_back(pp);
		error += (err*err);
	}	
	sigma = std::sqrt(error / cloud->size());
	mse = error / cloud->size() + (r - 0.0725)*(r - 0.0725);
}

//迭代最小二乘拟合球面
void LSSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_filter,
	Eigen::Vector3f &sphereCenter, float &r, float &sigma, float &mse)
{
	cloud_filter->clear();
	while (true)
	{
		LSSphere(cloud, sphereCenter, r, sigma, mse);
		std::cout << cloud->size() << "\t" << r << "\t" << sigma << "\t" << mse << std::endl;
		for (size_t i = 0; i < cloud->size(); i++)
		{
			float err = (cloud->points[i].getVector3fMap() - sphereCenter).norm() - r;
			if (std::abs(err)>3 * sigma)
			{
				continue;
			}
			cloud_filter->push_back(cloud->points[i]);
		}
		if (cloud->size()==cloud_filter->size())
		{
			break;
		}
		*cloud = *cloud_filter;
		cloud_filter->clear();
	}
}

//迭代最小二乘拟合球面,保留误差点
void LSSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_filter,
	pcl::PointCloud<PointType>::Ptr noise, Eigen::Vector3f &sphereCenter, float &r, float &sigma, float &mse)
{
	cloud_filter->clear();
	while (true)
	{
		LSSphere(cloud, sphereCenter, r, sigma, mse);
		std::cout << cloud->size() << "\t" << r << "\t" << sigma << "\t" << mse << std::endl;
		for (size_t i = 0; i < cloud->size(); i++)
		{
			float err = (cloud->points[i].getVector3fMap() - sphereCenter).norm() - r;
			if (err>3 * sigma)
			{
				noise->push_back(cloud->points[i]);
				continue;
			}
			cloud_filter->push_back(cloud->points[i]);
		}
		if (cloud->size() == cloud_filter->size())
		{
			break;
		}
		*cloud = *cloud_filter;
		cloud_filter->clear();
	}
}



//根据法线和指向球心 的向量夹角，进行识别球面点
void NormalToCenter(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr normals,
	Eigen::Vector3f sphere, pcl::PointCloud<PointType>::Ptr cloud_filter)
{
	cloud_filter->clear();

	for (size_t i = 0; i < cloud->size(); i++)
	{
		Eigen::Vector3f pointToCenter = (cloud->points[i].getVector3fMap() - sphere);
		pointToCenter.normalize();
		Eigen::Vector3f normal = normals->points[i].getNormalVector3fMap();
		normal.normalize();
		float sig = pointToCenter.dot(normal);
		if (std::abs(sig)>std::cos(5 * M_PI / 180))
		{
			cloud_filter->push_back(cloud->points[i]);
		}
	}
}



int main(int argc, char** argv)
{
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());

	//pcl::io::loadPCDFile("sphere_LS_fliter3.pcd", *model);
	////pcl::io::loadPCDFile("", *model_keypoints);
	//Eigen::Vector3f sphereCenter; float r; float sigma; float mse;
	//LSSphere(model, sphereCenter, r, sigma, mse);

	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<PointType>::Ptr scene_extraxt(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr normal_extract(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<PointType>::Ptr sphere(new pcl::PointCloud<PointType>);
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr frame(new pcl::PointCloud<pcl::ReferenceFrame>);

	//Algorithm params
	float model_ss_(0.005f);//模型均匀采样分辨率
	float scene_ss_(0.02f);//场景均匀采样
	float descr_rad_(0.0725f);//局部描述子半径
	float corres_threshold(0.3f);//特征描述对应点阈值
	float normal_rad_(0.03f);//法线估计邻域半径

	

	pcl::StopWatch time;
	//加载点云
	//if (pcl::io::loadPCDFile("model.pcd", *model) < 0)
	//{
	//	std::cout << "Error loading model cloud." << std::endl;
	//	return (-1);
	//}
	loadTXTFile("model.txt", NUMOFCOL::THREE, model);
	loadTXTFile("scene.txt", NUMOFCOL::THREE, scene);

	//if (pcl::io::loadPCDFile("scene.pcd", *scene) < 0)
	//{
	//	std::cout << "Error loading scene cloud." << std::endl;
	//	return (-1);
	//}

	//将结果输出到文件中
	//freopen("result.txt", "w", stdout);


	std::cout << "读取点云文件时间：" << time.getTime() << std::endl;
	
	time.reset();
	//  Compute Normals
	
	//pcl::NormalUser<PointType, NormalType> ess;
	//ess.setRadiusSearch(normal_rad_);
	//ess.setInputCloud(model);
	//ess.compute(*model_normals);

	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	//获取对应点395，NormalEstimationOMP获取428，并不是法线估计越准确获取的对应点越多
	//pcl::NormalEstimationUser norm_est;
	norm_est.setRadiusSearch(normal_rad_);//法线估计的半径0.03，这个数字很大了
	//norm_est.setKSearch(32);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);
	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);
	

	//去除法线中的NAN，对应的点云数据中的点也要去除
	std::vector<int> index1;
	pcl::removeNaNNormalsFromPointCloud<NormalType>(*model_normals, *model_normals, index1);
	//去除法线NAN对应的点
	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud(model);
	extract.setIndices(boost::make_shared<std::vector<int>>(index1));
	extract.filter(*model);

	pcl::removeNaNNormalsFromPointCloud<NormalType>(*scene_normals, *scene_normals, index1);
	extract.setInputCloud(scene);
	extract.setIndices(boost::make_shared<std::vector<int>>(index1));
	extract.filter(*scene);
	std::cout << "法线计算时间：" << time.getTime() << std::endl;
	time.reset();

	//  Downsample Clouds to Extract keypoints
	//此处的关键点可以修改，改正为几何特征的关键点
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);//模型采样率
	uniform_sampling.filter(*model_keypoints);
	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);//场景采样率
	uniform_sampling.filter(*scene_keypoints);
	pcl::io::savePCDFile("scene_keypoints.pcd", *scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
	std::cout << "均匀采样时间：" << time.getTime() << std::endl;

	//SHOT特征估计
	time.reset();
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);
	
	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);//法线必须自己输入
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);//法线必须自己输入
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);
	std::cout << "SHOT特征描述时间：" << time.getTime() << std::endl;
	time.reset();
	
	//  Find Model-Scene Correspondences with KdTree
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);
	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	//保证场景中的多个目标都能够识别出来，需要在场景特征点中查询最近的模型特征点
	for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		//对应点的阈值
		if (found_neighs == 1 && neigh_sqr_dists[0] < corres_threshold)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
	std::cout << "特征描述对应点计算时间：" << time.getTime() << std::endl;

	//得到的对应点里面，获取场景中的对应点索引，然后进行聚类
	std::vector<int> index_match;
	pcl::registration::getMatchIndices(*model_scene_corrs, index_match);
	std::vector<int> index_query;
	pcl::registration::getQueryIndices(*model_scene_corrs, index_query);
	pcl::io::savePCDFile("sphere.pcd", *scene_keypoints, index_match);
	time.reset();

	//欧氏聚类
	pcl::EuclideanClusterExtraction<PointType> cluster;
	cluster.setInputCloud(scene_keypoints);
	cluster.setIndices(boost::make_shared<std::vector<int> >(index_match));
	cluster.setClusterTolerance(0.0725);
	cluster.setMaxClusterSize(static_cast<int>(model_scene_corrs->size()));
	cluster.setMinClusterSize(static_cast<int>(model_scene_corrs->size()*0.1));
	std::vector<pcl::PointIndices> clusters;
	cluster.extract(clusters);
	std::cout << "欧氏聚类时间：" << time.getTime() << std::endl;

	//先粗略获取球形标靶参数，之后获取1.2半径内的点云，最后进行滤波
	pcl::ExtractIndices<NormalType> extractNormal;
	int i = 0;
	pcl::KdTreeFLANN<PointType> search;
	//自己写的距离加权协方差矩阵，求解点云法线
	pcl::NormalEstimationUser es;
	for (auto it = clusters.begin(); it != clusters.end(); ++it)
	{
		//提取聚类的球形标靶点云
		extract.setInputCloud(scene_keypoints);
		extract.setIndices(boost::make_shared<std::vector<int>>((*it).indices));
		extract.filter(*scene_extraxt);
		std::cout << "聚类结果点数：" << scene_extraxt->size() << std::endl;
		//保存聚类结果
		pcl::io::savePCDFile("cluster" + std::to_string(i) + ".pcd", *scene_keypoints, (*it).indices);
		
		time.reset();
		//最小二乘粗略获取球形标靶参数
		Eigen::Vector3f sphereCenter;
		//pcl::PointCloud<PointType>::Ptr cloud_filter1(new pcl::PointCloud<PointType>());
		float r, sigma, mse;
		LSSphere(scene_extraxt, sphereCenter, r, sigma, mse);
		std::cout << "使用LS估计球形标靶参数,球心坐标：" << sphereCenter << std::endl;
		std::cout << "球半径：" << r << std::endl;
		std::cout << "中误差" << sigma << std::endl;
		std::cout << "均方误差" << mse << std::endl;
		std::cout << "LS时间：" << time.getTime() << std::endl;

		
		//通过粗略得到的球形标靶参数，获取1.2r范围内的点云（球形标靶数据+一定量的非球面点）
		if (std::abs(r - 0.0725)<0.05)
		{
			std::vector<int> neigh_indices;
			std::vector<float> neigh_sqr_dists;
			search.setInputCloud(scene);
			pcl::PointXYZ cen(sphereCenter[0], sphereCenter[1], sphereCenter[2]);
			int found_neighs = search.radiusSearch(cen, r*1.2, neigh_indices, neigh_sqr_dists);
			extract.setInputCloud(scene);
			extract.setIndices(boost::make_shared<std::vector<int>>(neigh_indices));
			
			extract.filter(*sphere);
			
			pcl::io::savePCDFile("sphere" + std::to_string(i) + ".pcd", *sphere);

			pcl::PointCloud<PointType>::Ptr sphere_filter(new pcl::PointCloud<PointType>());

			time.reset();
			//对球形标靶点云进行迭代最小二乘滤波
			LSSphere(sphere, sphere_filter, sphereCenter, r, sigma, mse);
			std::cout << "ILS时间：" << time.getTime() << std::endl;

			std::cout << "迭代最小二乘结果点数：" << sphere_filter->size() << std::endl;
			pcl::io::savePCDFile("sphere_LS_fliter"+std::to_string(i)+".pcd", *sphere_filter);

			time.reset();
			//这里重新进行法线估计,使用距离加权的协方差矩阵进行计算
			es.setInputCloud(sphere_filter);
			es.setRadiusSearch(normal_rad_);
			es.compute(*normal_extract);

			*sphere = *sphere_filter;
			//法线方向滤波
			NormalToCenter(sphere, normal_extract, sphereCenter, sphere_filter);
			std::cout << "法线时间：" << time.getTime() << std::endl;

			std::cout << "法线方向结果点数：" << sphere_filter->size() << std::endl;
			//最后对滤波后的点云进行最小二乘拟合
			LSSphere(sphere_filter, sphereCenter, r, sigma, mse);
			std::cout << "使用LS估计球形标靶参数,球心坐标：" << sphereCenter << std::endl;
			std::cout << "球半径：" << r << std::endl;
			std::cout << "中误差" << sigma << std::endl;
			std::cout << "均方误差" << mse << std::endl;
			pcl::io::savePCDFile("sphere_filter" + std::to_string(i) + ".pcd", *sphere_filter);
		}
		i++;//更新文件名
	}
	system("pause");
	return 0;
}