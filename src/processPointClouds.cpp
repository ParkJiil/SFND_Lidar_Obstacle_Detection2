// PCL lib Functions for processing point clouds 
#include <unordered_set>    // 텀프로젝트 추가코드 
#include "processPointClouds.h"
#include "kdtree.h"
// #include <pcl/search/kdtree.h>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{                                                                // FilterCloud                              // min point of x,y,z & max point of x,y,z for ROI (by crop box) \ fourth value marked as 1.0
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now(); // timer. This can be useful for measuring how long it takes to run the function

   // Create the filtering object
    pcl::VoxelGrid<PointT> vg; 
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>());
    //std::cout << typeid(vg).name() << endl;
    vg.setInputCloud (cloud);                           // give it that input cloud
    vg.setLeafSize (filterRes, filterRes, filterRes);   // leaf size of cube (Cell size of filterRes)
    vg.filter (*cloudFiltered);                         // new point cloud인 cloudFiltered(위 선언)에 저장
    
    //Region based interest (pcl cropbox)
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);                  // cropbox로 잘라낸 지역 내 point 저장할 경우 true         
    region.setMin(minPoint);                            // defining the box min & max size
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);                // 위 라인 36에서 복셀그리드로 필터링된 데이터가 input 
    region.filter(*cloudRegion);                        // 결과 저장 (outside of the box will be removed) -> 오직 박스안에 포인트만 저장

    // roof top points 삭제 (pcl cropbox)
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);                    // ego car의 roof 볼지 안볼지 확인 
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1.,1));      // 위에서 정의한 최대 최소 x,y,z value
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));        // 지붕 좌표는 static
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);                               // indice에 저장

    // Add these roof points to a pcl::PointIndices object
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};    // 차량 지붕을 제외한 데이터가 iteration되면서 저장됨 
    for (const auto elem: indices) 
        inliers->indices.push_back(elem);
 
    // Remove these roof points from cloudRegion (the region of interest)
    pcl::ExtractIndices<PointT> extract;               // segment와 비슷한 코드
    extract.setInputCloud(cloudRegion);     
    extract.setIndices(inliers);                       // give it that inlier (위 라인 54에서 저장)
    extract.setNegative(true);                         // inlier를 별도로 제거할 경우 true (extract these indices from inliers)
    extract.filter(*cloudRegion);                      

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    // return cloud;         // 이거로 하면 인풋 클라우드가 그대로 나오는 형태
    return cloudRegion;      // 이걸로 변경하면 인라이어만 cloudRegion에 리턴되서 속도와 정확도 향상 

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{// std::pair는 두 개의 독립된 objects를 하나의 컨테이너에 저장하는 템플릿,                                                                                                                  // SeparateClouds

    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());    // new pcl::PointCloud<PointT>인 ObsCloud 생성
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());   // new pcl::PointCloud<PointT>인 planeCloud 생성

    // generating pointcloud by iteration & passing in inlier   * 포인터 설명 : 좌측 `포인터(address만 보유)가 우측 함수(또는 메모리)를 불러오는 것   ex) A->Run() == A.Run();
    for(int index : inliers -> indices)    // inlier는 정수(int) 벡터, indices는 PointT 벡터 타입 중에 하나 \ 모든 inlier의 indice value에 해당하는 push_back 저장 
        planeCloud->points.push_back(cloud -> points[index]);  // points[index]에 해당하는 inlier들이 cloud에 저장(이를 반복하고 index를 가져와 해당 index에 해당 pointT를 planeCloud에 저장

    pcl::ExtractIndices<PointT> extract;     // extract라는 objext 생성 
    extract.setInputCloud (cloud);           // 레퍼펀스 cloud 
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);              // de-reference it when I pass it into filter (즉 inlier를 제외한 나머지를 저장한 결과)

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;                                         
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{                                                                                                                // SegmentPlane
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now(); 


    // RANSAC algorithm for plane segmentation (텀프로젝트 추가 코드)
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while (maxIterations--) {
		// Randomly sample subset and fit plane
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size()); 
		}

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a, b, c, d, v3_i, v3_j, v3_k;
		// Let v1 be a vector from point 1 to point 2 in the plane
		// Let v2 be a vector from point 1 to point 3 in the plane
		// Let v3 equal the cross product v1 x v2
		v3_i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		v3_j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		v3_k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);

		// This iteration's plane is modeled by the equation Ax + By + Cz + D = 0
		a = v3_i;
		b = v3_j;
		c = v3_k;
		d = -(v3_i*x1 + v3_j*y1 + v3_k*z1);

		for (int index = 0; index < cloud->points.size(); index++) {
			// Skip if the considered point is already an inlier.
			if (inliers.count(index) > 0) continue;
			
			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float distance = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);

			if (distance <= distanceThreshold) {
				inliers.insert(index);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

    // Populate the newly allocated point clouds
	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());  // The plane points
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());  // The not-plane points

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Return a std::pair of point clouds (obstacles, plane)
    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> (cloudOutliers, cloudInliers);
 
}

// 텀 프로젝트 추가 코드 
static void clusterHelper(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearest = tree->search(points[index], distanceTol);

	for (int idx : nearest) {
		if (!processed[idx]) {
			clusterHelper(idx, points, cluster, processed, tree, distanceTol);
		}
	}
}

static std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;

	while (i < points.size()) {
		if (processed[i]) {
            i++;
            continue;
        }

		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
        if (cluster.size() >= minSize && cluster.size() <= maxSize) {
            clusters.push_back(cluster);
        } else {
            for (int remove_index : cluster) {
                processed[remove_index] = false;
            }
        }
        i++;
	}

	return clusters;
}
// 텀 프로젝트 추가 끝


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // 텀프로젝트 (kd-tree로 유클리안 클러스터링 구현)
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> pts;

    KdTree* tree = new KdTree;
    for (int i = 0; i < cloud->points.size(); i++) {
        auto pt = cloud->points[i];
        pts.push_back(std::vector<float> {pt.x, pt.y, pt.z});
    	tree->insert(std::vector<float> {pt.x, pt.y, pt.z}, i);
    }

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(pts, tree, clusterTolerance, minSize, maxSize);

    for (auto clusterIndex : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for (auto pointIndex : clusterIndex) {
            cloudCluster->points.push_back (cloud->points[pointIndex]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }
    // 텀프로젝트 엔드 라인 (kd-tree로 유클리안 클러스터링 구현)

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}