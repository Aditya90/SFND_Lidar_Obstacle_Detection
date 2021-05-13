/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

float calcDistance(pcl::PointXYZ pointData, long A, long B, long C, long D)
{
	long x = pointData.x;
	long y = pointData.y;
	long z = pointData.z;
	float retVal = fabs(A*x + B*y + C*z + D)/sqrt(A*A+B*B+C*C);

	return (retVal);
}

float calcDistance(pcl::PointXYZ pointData, int A, int B, int C)
{
	int x = pointData.x;
	int y = pointData.y;
	float retVal = fabs(A*x + B*y + C)/sqrt(A*A+B*B);

	return (retVal);
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for (int i=0; i<maxIterations;i++)
	{
		std::unordered_set<int> inliersIter;

		cout << "Iteration number "<< i << std::endl;
		// Randomly sample subset and fit line
		size_t randomIndexOne = (rand() % cloud->points.size());
		size_t randomIndexTwo = (rand() % cloud->points.size());
		cout << "randomIndexOne "<< randomIndexOne << std::endl;
		cout << "randomIndexTwo "<< randomIndexTwo << std::endl;

		while (randomIndexOne == randomIndexTwo)
		{
			randomIndexTwo = rand() % cloud->points.size();
		}

		int x1 = cloud->points.at(randomIndexOne).x;
		int y1 = cloud->points.at(randomIndexOne).y;
		int x2 = cloud->points.at(randomIndexTwo).x;
		int y2 = cloud->points.at(randomIndexTwo).y;
		cout << "x1 "<< x1 << std::endl;
		cout << "y1 "<< y1 << std::endl;
		cout << "x2 "<< x2 << std::endl;
		cout << "y2 "<< y2 << std::endl;

		int A = (y1-y2);
		int B = (x2-x1);
		int C = (x1*y2 - x2*y1);
		cout << "A "<< A << std::endl;
		cout << "B "<< B << std::endl;
		cout << "C "<< C << std::endl;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int j=0; j< cloud->points.size(); j++)
		{
			if ((j == randomIndexOne) || (j==randomIndexTwo))
			{
				continue;
			}

			auto pointToCheck = cloud->points.at(j);
			if (calcDistance(pointToCheck, A, B, C) <= distanceTol)
			{
				inliersIter.insert(j);
				cout << "Inlier index "<< j << std::endl;

			}
		}

		if (inliersIter.size() > inliersResult.size())
		{
			inliersResult = inliersIter;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for (int i=0; i<maxIterations;i++)
	{
		std::unordered_set<int> inliersIter;

		cout << "Iteration number "<< i << std::endl;
		// Randomly sample subset and fit line
		std::unordered_set<size_t> pointIndices{};
		while (pointIndices.size() != 3)
		{
			pointIndices.insert(rand() % cloud->points.size());
		}
		auto pointIndicesIter = pointIndices.begin();
		auto point1 =  cloud->points.at(*pointIndicesIter++);
		auto point2 =  cloud->points.at(*pointIndicesIter++);
		auto point3 =  cloud->points.at(*pointIndicesIter);

		int x1 = point1.x;
		int y1 = point1.y;
		int z1 = point1.z;
		int x2 = point2.x;
		int y2 = point2.y;
		int z2 = point2.z;
		int x3 = point3.x;
		int y3 = point3.y;
		int z3 = point3.z;


		long A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		long B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		long C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		long D = -(A*x1+B*y1+C*z1);
		cout << "A "<< A << std::endl;
		cout << "B "<< B << std::endl;
		cout << "C "<< C << std::endl;
		cout << "D "<< D << std::endl;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int j=0; j< cloud->points.size(); j++)
		{
			if (pointIndices.find(j) != pointIndices.end())
			{
				continue;
			}

			auto pointToCheck = cloud->points.at(j);
			float distanceFromPointToPlane = calcDistance(pointToCheck, A, B, C, D);
			if (distanceFromPointToPlane <= distanceTol)
			{
				inliersIter.insert(j);
			}
			else
			{
				cout << "Found Outlier. Distnace: " << distanceFromPointToPlane << std::endl;
			}
			
		}

		if (inliersIter.size() > inliersResult.size())
		{
			inliersResult = inliersIter;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 50, 1);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
