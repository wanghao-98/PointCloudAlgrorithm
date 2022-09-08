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

//��С�������
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

//������С�����������
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

//������С�����������,��������
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



//���ݷ��ߺ�ָ������ �������нǣ�����ʶ�������
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
	float model_ss_(0.005f);//ģ�;��Ȳ����ֱ���
	float scene_ss_(0.02f);//�������Ȳ���
	float descr_rad_(0.0725f);//�ֲ������Ӱ뾶
	float corres_threshold(0.3f);//����������Ӧ����ֵ
	float normal_rad_(0.03f);//���߹�������뾶

	

	pcl::StopWatch time;
	//���ص���
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

	//�����������ļ���
	//freopen("result.txt", "w", stdout);


	std::cout << "��ȡ�����ļ�ʱ�䣺" << time.getTime() << std::endl;
	
	time.reset();
	//  Compute Normals
	
	//pcl::NormalUser<PointType, NormalType> ess;
	//ess.setRadiusSearch(normal_rad_);
	//ess.setInputCloud(model);
	//ess.compute(*model_normals);

	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	//��ȡ��Ӧ��395��NormalEstimationOMP��ȡ428�������Ƿ��߹���Խ׼ȷ��ȡ�Ķ�Ӧ��Խ��
	//pcl::NormalEstimationUser norm_est;
	norm_est.setRadiusSearch(normal_rad_);//���߹��Ƶİ뾶0.03��������ֺܴ���
	//norm_est.setKSearch(32);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);
	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);
	

	//ȥ�������е�NAN����Ӧ�ĵ��������еĵ�ҲҪȥ��
	std::vector<int> index1;
	pcl::removeNaNNormalsFromPointCloud<NormalType>(*model_normals, *model_normals, index1);
	//ȥ������NAN��Ӧ�ĵ�
	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud(model);
	extract.setIndices(boost::make_shared<std::vector<int>>(index1));
	extract.filter(*model);

	pcl::removeNaNNormalsFromPointCloud<NormalType>(*scene_normals, *scene_normals, index1);
	extract.setInputCloud(scene);
	extract.setIndices(boost::make_shared<std::vector<int>>(index1));
	extract.filter(*scene);
	std::cout << "���߼���ʱ�䣺" << time.getTime() << std::endl;
	time.reset();

	//  Downsample Clouds to Extract keypoints
	//�˴��Ĺؼ�������޸ģ�����Ϊ���������Ĺؼ���
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);//ģ�Ͳ�����
	uniform_sampling.filter(*model_keypoints);
	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);//����������
	uniform_sampling.filter(*scene_keypoints);
	pcl::io::savePCDFile("scene_keypoints.pcd", *scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
	std::cout << "���Ȳ���ʱ�䣺" << time.getTime() << std::endl;

	//SHOT��������
	time.reset();
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);
	
	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);//���߱����Լ�����
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);//���߱����Լ�����
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);
	std::cout << "SHOT��������ʱ�䣺" << time.getTime() << std::endl;
	time.reset();
	
	//  Find Model-Scene Correspondences with KdTree
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);
	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	//��֤�����еĶ��Ŀ�궼�ܹ�ʶ���������Ҫ�ڳ����������в�ѯ�����ģ��������
	for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		//��Ӧ�����ֵ
		if (found_neighs == 1 && neigh_sqr_dists[0] < corres_threshold)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
	std::cout << "����������Ӧ�����ʱ�䣺" << time.getTime() << std::endl;

	//�õ��Ķ�Ӧ�����棬��ȡ�����еĶ�Ӧ��������Ȼ����о���
	std::vector<int> index_match;
	pcl::registration::getMatchIndices(*model_scene_corrs, index_match);
	std::vector<int> index_query;
	pcl::registration::getQueryIndices(*model_scene_corrs, index_query);
	pcl::io::savePCDFile("sphere.pcd", *scene_keypoints, index_match);
	time.reset();

	//ŷ�Ͼ���
	pcl::EuclideanClusterExtraction<PointType> cluster;
	cluster.setInputCloud(scene_keypoints);
	cluster.setIndices(boost::make_shared<std::vector<int> >(index_match));
	cluster.setClusterTolerance(0.0725);
	cluster.setMaxClusterSize(static_cast<int>(model_scene_corrs->size()));
	cluster.setMinClusterSize(static_cast<int>(model_scene_corrs->size()*0.1));
	std::vector<pcl::PointIndices> clusters;
	cluster.extract(clusters);
	std::cout << "ŷ�Ͼ���ʱ�䣺" << time.getTime() << std::endl;

	//�ȴ��Ի�ȡ���α�в�����֮���ȡ1.2�뾶�ڵĵ��ƣ��������˲�
	pcl::ExtractIndices<NormalType> extractNormal;
	int i = 0;
	pcl::KdTreeFLANN<PointType> search;
	//�Լ�д�ľ����ȨЭ������������Ʒ���
	pcl::NormalEstimationUser es;
	for (auto it = clusters.begin(); it != clusters.end(); ++it)
	{
		//��ȡ��������α�е���
		extract.setInputCloud(scene_keypoints);
		extract.setIndices(boost::make_shared<std::vector<int>>((*it).indices));
		extract.filter(*scene_extraxt);
		std::cout << "������������" << scene_extraxt->size() << std::endl;
		//���������
		pcl::io::savePCDFile("cluster" + std::to_string(i) + ".pcd", *scene_keypoints, (*it).indices);
		
		time.reset();
		//��С���˴��Ի�ȡ���α�в���
		Eigen::Vector3f sphereCenter;
		//pcl::PointCloud<PointType>::Ptr cloud_filter1(new pcl::PointCloud<PointType>());
		float r, sigma, mse;
		LSSphere(scene_extraxt, sphereCenter, r, sigma, mse);
		std::cout << "ʹ��LS�������α�в���,�������꣺" << sphereCenter << std::endl;
		std::cout << "��뾶��" << r << std::endl;
		std::cout << "�����" << sigma << std::endl;
		std::cout << "�������" << mse << std::endl;
		std::cout << "LSʱ�䣺" << time.getTime() << std::endl;

		
		//ͨ�����Եõ������α�в�������ȡ1.2r��Χ�ڵĵ��ƣ����α������+һ�����ķ�����㣩
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
			//�����α�е��ƽ��е�����С�����˲�
			LSSphere(sphere, sphere_filter, sphereCenter, r, sigma, mse);
			std::cout << "ILSʱ�䣺" << time.getTime() << std::endl;

			std::cout << "������С���˽��������" << sphere_filter->size() << std::endl;
			pcl::io::savePCDFile("sphere_LS_fliter"+std::to_string(i)+".pcd", *sphere_filter);

			time.reset();
			//�������½��з��߹���,ʹ�þ����Ȩ��Э���������м���
			es.setInputCloud(sphere_filter);
			es.setRadiusSearch(normal_rad_);
			es.compute(*normal_extract);

			*sphere = *sphere_filter;
			//���߷����˲�
			NormalToCenter(sphere, normal_extract, sphereCenter, sphere_filter);
			std::cout << "����ʱ�䣺" << time.getTime() << std::endl;

			std::cout << "���߷�����������" << sphere_filter->size() << std::endl;
			//�����˲���ĵ��ƽ�����С�������
			LSSphere(sphere_filter, sphereCenter, r, sigma, mse);
			std::cout << "ʹ��LS�������α�в���,�������꣺" << sphereCenter << std::endl;
			std::cout << "��뾶��" << r << std::endl;
			std::cout << "�����" << sigma << std::endl;
			std::cout << "�������" << mse << std::endl;
			pcl::io::savePCDFile("sphere_filter" + std::to_string(i) + ".pcd", *sphere_filter);
		}
		i++;//�����ļ���
	}
	system("pause");
	return 0;
}