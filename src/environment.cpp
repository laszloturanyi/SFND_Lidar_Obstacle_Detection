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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud = lidar->scan();
    //renderRays(viewer, lidar->position, scan_cloud);
    //renderPointCloud(viewer, scan_cloud, "scan_cloud,", Color(1, 1, 1));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_processor;// = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_cloud = point_processor.SegmentPlane(scan_cloud, 100, 0.2);
    //renderPointCloud(viewer, segment_cloud.first, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, segment_cloud.second, "plane_cloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters = point_processor.Clustering(segment_cloud.first, 1.0, 3, 30);

    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_clusters)
    {
        std::cout << "cluster size";
        point_processor.numPoints(cluster);
        Box box = point_processor.BoundingBox(cluster);
        renderPointCloud(viewer, cluster, "obstacle_cloud"+ std::to_string(cluster_id), colors[cluster_id]);
        renderBox(viewer, box, cluster_id);
        ++cluster_id;
    }
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* point_processor_i, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  Eigen::Vector4f min_point = Eigen::Vector4f(-30.0f, -6.0f, -2.0f, 1.0f);
  Eigen::Vector4f max_point = Eigen::Vector4f(30.0f, 7.0f, 2.0f, 1.0f);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = point_processor_i->FilterCloud(input_cloud, 0.15f, min_point, max_point);
  //renderPointCloud(viewer,filtered_cloud,"filtered_cloud");

  std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = point_processor_i->SegmentPlane(filtered_cloud, 100, 0.2);
  renderPointCloud(viewer, segment_cloud.second, "plane_cloud", Color(0,1,0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = point_processor_i->Clustering(segment_cloud.first, 0.5, 3, 2000);

  int cluster_id = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters)
  {
      std::cout << "cluster size";
      point_processor_i->numPoints(cluster);
      Box box = point_processor_i->BoundingBox(cluster);
      renderPointCloud(viewer, cluster, "obstacle_cloud"+ std::to_string(cluster_id), colors[cluster_id % 3]);
      renderBox(viewer, box, cluster_id);
      ++cluster_id;
  }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* point_processor_i = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = point_processor_i->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  Eigen::Vector4f min_point = Eigen::Vector4f(-30.0f, -6.0f, -2.0f, 1.0f);
  Eigen::Vector4f max_point = Eigen::Vector4f(30.0f, 7.0f, 2.0f, 1.0f);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = point_processor_i->FilterCloud(input_cloud, 0.15f, min_point, max_point);
  //renderPointCloud(viewer,filtered_cloud,"filtered_cloud");

  std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = point_processor_i->SegmentPlane(filtered_cloud, 100, 0.2);
  renderPointCloud(viewer, segment_cloud.second, "plane_cloud", Color(0,1,0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = point_processor_i->Clustering(segment_cloud.first, 0.5, 3, 2000);

  int cluster_id = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters)
  {
      std::cout << "cluster size";
      point_processor_i->numPoints(cluster);
      Box box = point_processor_i->BoundingBox(cluster);
      renderPointCloud(viewer, cluster, "obstacle_cloud"+ std::to_string(cluster_id), colors[cluster_id % 3]);
      renderBox(viewer, box, cluster_id);
      ++cluster_id;
  }
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);

    //while (!viewer->wasStopped ())
    //{
    //    viewer->spinOnce ();
    //} 

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }
}