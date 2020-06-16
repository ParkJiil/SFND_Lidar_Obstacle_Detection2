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
		double rz = 2*(((double) rand() / (RAND_MAX))-0.5);  
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
		point.z = i+scatter*rz;  
  		// point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
		double rz = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
		point.z = 5*rz;  
  		// point.z = 0;

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;                  // highest inlier가 저장됨 
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers 
    
	while(maxIterations--)
	{  // randomly pick two points                     // std::unordered_set은 std::set과 달리 자동정렬을 하지 않아 빠른 장점이 있음 <int>는 set을 정수로 저장하겠다는 것을 의미  
		std::unordered_set<int> inliers;                    // std::unordered_set을 사용하면 동일한 포인트가 랜덤하게 반복 선택될시 중복하지 않고 하나만 저장하는 장점이 있어서 사용 		                                              
		
		while (inliers.size() < 2)                          // inlier 사이즈가 2 미만일 경우 (처음에는 84번 라인에서 선언하였으며 초기 value는 0)
		  inliers.insert(rand()%(cloud->points.size()));    // 포인트 갯수까지 랜덤으로 셀렉해서 inlier를 추가시킴(0부터 points.size()까지 증가)

	    float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inliers.begin();                         // beginning of inliers를 itr로 지정
		x1 = cloud->points[*itr].x;                         // de-referencing the pointer해서 seeing what the value is한다(dereference it to get that int value)
		y1 = cloud->points[*itr].y;                         // 즉, Cloud에 value가 indexing된다
		z1 = cloud->points[*itr].z;

		itr++;
		x2 = cloud->points[*itr].x;                         // get that x component and the y component and that's going to be x2 and y2
		y2 = cloud->points[*itr].y;                         // points에 itr번째 y index of cloud가 y2에 저장됨! (index에는 itr가 들어가고 index에는 x,y,z가 포함)
        z2 = cloud->points[*itr].z; 
        
		itr++;    
		x3 = cloud->points[*itr].x;                         // get that x component and the y component and that's going to be x2 and y2
		y3 = cloud->points[*itr].y;                         // points에 itr번째 y index of cloud가 y2에 저장됨! (index에는 itr가 들어가고 index에는 x,y,z가 포함)
        z3 = cloud->points[*itr].z;

		// -> So that's how we can get the x1, y1, x2, y2 values from inliers that we randomly sampled

        float a = y1*(z2-z3) + y2*(z3-z1) + y3*(z1-z2);
		float b = z1*(x2-x3) + z2*(x3-x1) + z3*(x1-x2);
		float c = x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2);
		float d = - x1*(y2*z3-y3*z2) - x2*(y3*z1-y1*z3) - x3*(y1*z2-y2*z1);

        for(int index = 0; index < cloud->points.size(); index++)
	    {
		    if(inliers.count(index)>0)                     // 포인트가 있다면 계속 진행 
			  continue;

            // 포인트가 라인 내에 위치한 포인트가 아닐 경우 아래처럼 거리 계산 
		    pcl::PointXYZ point = cloud->points[index];    // pcl::PointXYZ 형태의 point에 point by indexing into our cloud를 grab 
		    float x4 = point.x;                            // Then we can see what those x and y values of pointcloud
			float y4 = point.y;                            // get this x3 & y3 by getting the member x & the member y from point 
			float z4 = point.z; 

			float d = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);   // fabs는 절대갑 \ 교육자료 나온 Distance 구하는 공식 (figure out distance for, so x3, y3). 

			if (d <= distanceTol)                          // calculated d is less than or equal to distance tolerance , 
                 inliers.insert(index);                    // then add it to insert x3 & y3 or the inliers

	}
    
	if (inliers.size() > inliersResult.size())             // inliersResult 위에서 선언 시 초기 value는 0             
        inliersResult = inliers;                           // 인라이어가 기존 인라이어리절트보다 큰 경우 인라이어리절트에 저장됨

    // 여기까지 반복하면 size of inliers가 완성됨  

    auto startTime = std::chrono::steady_clock::now();

    auto endTime = std::chrono::steady_clock::now(); 
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime); 
	std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" << std::endl;
 
	return inliersResult;                                  // 위에서 저장한 인라이어 리절트 리턴 
  }
}


int main () 
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 1.0);  // std::unordered_set은 std::set과 달리 자동정렬을 하지 않아 빠른 장점이 있음 <int>는 set을 정수로 저장하겠다는 것을 의미 
                                                                // Ransac(inpur, iteration, distanceTol)
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    //for문 (0부터 go for the size of points in cloud) 
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