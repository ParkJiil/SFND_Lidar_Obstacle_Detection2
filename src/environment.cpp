/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
} 

  

/// 리얼 환경 라이다 데이터 사용 시 (여기서 1.필터링(라인48), 2.세그멘테이션(라인51), 3.클러스터링(라인56), 4.렌더링(라인67)이 모두 이루어짐 
// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)  // 한개의 pcd 파일 사용 시, 멀티 pcd 파일에 적용시에는 아랫줄 'void~~'  라인 활용
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) 
{  // 힙 세그먼트는 동적 메모리 할S당에 사용되는 메모리를 추적한다. C++에서 new 연산자를 사용해서 메모리를 할당하면 이 메모리는 응용 프로그램의 힙 세그먼트에 할당
   // 스택 세그먼트(=콜 스택)는 메인() 함수부터 현재 실행 지점까지의 모든 활성 함수를 추적하고 모든 함수 매개 변수와 지역 변수의 할당을 처리

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();      // *를 사용했으니 이건 힙, 다른 방법으로 스택도 사용 가능
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");   // loadPcd 함수 활용
  
  //inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));  // 0.3을 0.1로 바꾸이 블랙 스크린 
  //  inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-20, -6.5, -3, 1), Eigen::Vector4f(25, 6.5, 3, 1)); // 정답코드 
  inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7.5, 3, 1));  // min x,y,z & max x,y,z for ROI(good)
  // ROI 박스의 대각선 좌표 2개를 입력하여 바운딩 박스 선정(데이터 사이즈를 벗어나면 블랙스크린 뜨니 더 작게 선정해줘야 함)  
  // Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) -> min point of x,y,z & max point of x,y,z for ROI (by crop box) \ x,y,z 다음 fourth value는 1.0 !!!!!!
  
  // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = (*pointProcessorI).SegmentPlane(inputCloud, 25, 0.3);
  // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 25, 0.3);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(inputCloud, 25, 0.3);

  // renderPointCloud(viewer,segmentCloud.first,"obsCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second,"groundCloud",Color(0,1,0));    // 렌더링 하기 
  
  //ClusteringFdata1
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.53, 10, 500);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      pointProcessorI.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

      Box box = pointProcessorI.BoundingBox(cluster);
      renderBox(viewer,box,clusterId);
      ++clusterId;
    }
  // renderPointCloud(viewer, inputCloud, "cloud");   
}


/// 가상환경 라이다 적용시 사용하는 함수 ///
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; 
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);     // new lidar라는 새로운 변수 선언 (car는 46번 라인에서 정의한 변수, 0은 ground plane 의미)
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); //scan function을 통해 pcl::PointCloud<pcl::PointXYZ>::Ptr 타입으로 inputCloud에 저장
    // renderRays(viewer,lidar->position, inputCloud);    // renderRays로 시각화 & lidar.h 내에 있는 Lidar structure 내 position, 위에서 정의한 inputCloud 
    // renderPointCloud(viewer, inputCloud, "inputCloud");  
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;    // point processor 생성
    // ProcessPointClouds<pcl::PointXYZ>* pointprocessor = new ProcessPointClouds<pcl::PointXYZ>();
       // 2번째 방법은 1번째와 동일한데 pointer(*) 개념을 적용한 라인 / 포인터 문법은 '타입* 포인터 이름'
       // 타입은 포인터가 가리키는 변수타입을 명시 & 포인터 이름은 포인터 선언후 포인터 접근을 위해 사용                 // pointProcessor.SegmentPlane
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    // 위 라인 뜻은 54라인에서 생성한 pointProcessor를 processPointCloud.CPP예서 만든 SegmentPlane 함수 적용  ->  100번 반복, 거리한계 0.2 적용
    // std::pair이지만 뒤에 변수가 한개인 이유는 이미 SegmentPlane function이 2개의 데이터를 가진 형태이기 때문임 
    
    //if(render_obst)
      renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));         // 클러스터만 보기 위해서는 62, 63번 라인 주석처리하고 실행하면 됨
    //if(render_plane)
      renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));  
                                                                     // segmentCloud로 object와 road로 구분되는데 segmentCloud.first가 object임
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30); // grab segmentCloud.first
                                               // hyperparameter : distance tolerance 1.0 (one distance unit안에 있어야 함), 최소 3 포인트, 클러스터 최대 30 포인트               
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};  // red, yellow, blue

    ///////// rendering bounding box //////////
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)   // grab pcl::PointCloud<pcl::PointXYZ>::Ptr and call cluster & cloudClusters(위 선언)
    {
        //if(render_clusters)
        //{
          std::cout << "cluster size ";
          pointProcessor.numPoints(cluster);
          // viewer를 통해서 cluster를 렌더링, "obsCloud"+std::to_String(clusterId)라는 unique name을 부여하고 give it colors (위에서 정의한 3가지 다른 색상이 적용됨)
          renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
        //}

        //if(Render_box)
        //{
          // Box box = pointProcessor->BoundingBox(cluster);   
          Box box = pointProcessor.BoundingBox(cluster);    // pointProcessor function의 BoundingBox 활용
          renderBox(viewer, box, clusterId);                // Call the render box in render.cpp 
        //}
        ++clusterId;      // 위에서 추가로 부여한 clusterId를 통해서 iteration 진행  
    } 
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud");
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

// 결과로 보고 싶은 것은 main 함수에서 실행
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);         // 가상 라이다 데이터 사용시 (1장 pcd)
    // cityBlock(viewer);             // 실제 라이다 데이터 사용시 선택!!! (1장 pcd)

    // create point cloud processor (160 or 161 라인 미선택시-> 여러 pcd 파일 사용시 아래 코드 적용) -> calling inside this frame update loop.
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;  // ntensity를 사용하는 pointProcessor (스택에 생성 -> everytime 이 함수를 쓰겠다는 의미, 만약 on the heap, arrow operator 사용해야 함)
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");      // stream pcd files
    auto streamIterator = stream.begin();                                                                           // creating stream iterator (start from beginning) 
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;                                                               // placeholder for cloud  

    while (!viewer->wasStopped ())
    {

    // Clear viewer (While문 돌면서 기존 포인트 초기화)
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());   // pcd파일 로드한 다음 stream iterator를 dereference 함 (string으로 convert)
    cityBlock(viewer, pointProcessorI, inputCloudI);                     // 위에서 셋업 완료되면 cityblock 호출 (with pointProcessorI & 위에서 로드한 inputCloudI)

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce ();
    }
}