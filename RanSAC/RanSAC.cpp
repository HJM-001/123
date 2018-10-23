// RanSAC.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include <iostream>
#include <use_pcl.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <PoS.h>

REAL PS::R = 100.0f;
Vec3d PS::Center = Vec3d(0, 0, 0);

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters();
	return (viewer);
}

int
main(int argc, char** argv)
{
	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

	// populate our PointCloud with points
	srand((unsigned)time(NULL)); 	  //HJM add
	cloud->width = 500;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	int a = 0, b = 0, c = 0;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
		{
#ifdef _SPHERE	 // 球面	
			PS ps(2 * PI * rand() / (RAND_MAX + 1.0), PI * rand() / (RAND_MAX + 1.0));
			cloud->points[i].x = ps.S2V().X();
			cloud->points[i].y = ps.S2V().Y();
			if (i % 3 == 0)
				ps.SetR(152.4 * rand() / (RAND_MAX + 1.0));
			else
				ps.SetR(100.0f);;
			cloud->points[i].z = ps.S2V().Z();
#else	//柱面
			REAL r;
			r = (i % 5 == 0) ? (40 * rand() / (RAND_MAX + 1.0)) : (20.0f);
			Cyc a(r, 2 * PI * rand() / (RAND_MAX + 1.0), 152.4 * rand() / (RAND_MAX + 1.0));
			Vec3d& v = a.C2D();
			cloud->points[i].x = v[0];
			cloud->points[i].y = v[1];
			cloud->points[i].z = v[2];

#endif		

		}
		else
		{
			cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
			cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
			if (i % 2 == 0)
				cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
			else
				cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
		}
		/*
		if (pcl_isnan(cloud->points[i].z))
		c++;
		cout << cloud->points[i].x <<" " << cloud->points[i].y << " " << cloud->points[i].z << endl;
		*/
	}

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
#ifdef _SPHERE
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
		model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
#else
	pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::PointNormal>::Ptr  model_c(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::PointNormal>(cloud));

	//estimate normal
	typedef pcl::PointNormal NormalType;
	typedef pcl::PointXYZ PointType;

	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>);

	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setNumberOfThreads(4);//HJM ADD
	norm_est.setKSearch(10);
	norm_est.setInputCloud(cloud);
	norm_est.compute(*model_normals);
{ 
	model_c->setInputNormals( model_normals);
}
#endif

	if (pcl::console::find_argument(argc, argv, "-f") >= 0)
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(0.01);  //HJM: 小于些阈值可视为inlier
		ransac.computeModel();
		ransac.getInliers(inliers);
	}
	else if (pcl::console::find_argument(argc, argv, "-sf") >= 0)
	{

#ifdef _SPHERE
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
#else
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_c);
#endif
		ransac.setDistanceThreshold(.01);
		pcl::ScopeTime t("Racsac");
		ransac.computeModel();
		ransac.getInliers(inliers);
	}

	cout << inliers.size() << endl;;
	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

	// creates the visualization object and adds either our orignial cloud or all of the inliers
	// depending on the command line arguments specified.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
		viewer = simpleVis(final);
	else
		viewer = simpleVis(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}


