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
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    const auto &lidar = std::make_unique<Lidar>( cars, 0.0 );
    const auto& cloud = lidar->scan();
    // renderRays( viewer, lidar->position, cloud );
    // renderPointCloud( viewer, cloud, "scan" );

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor;
    const auto& [ground_cloud, non_ground_cloud] = processor.SegmentPlane(cloud, 10, 0.5);
    // renderPointCloud( viewer, ground_cloud, "ground", Color(0.0f, 1.0f, 0.0f) );
    // renderPointCloud( viewer, non_ground_cloud, "non_ground", Color(1.0f, 0.0f, 0.0f) );

    const auto& clusters = processor.Clustering(non_ground_cloud, 1.0, 3, 1000);
    for(int i = 0; i < clusters.size(); i++)
    {
        const auto r = static_cast<double>(rand()) / RAND_MAX;
        const auto g = static_cast<double>(rand()) / RAND_MAX;
        const auto b = static_cast<double>(rand()) / RAND_MAX;
        renderPointCloud(viewer, clusters[i], "cluster_" + std::to_string(i), Color(r, g, b));
        Box box = processor.BoundingBox(clusters[i]);
        renderBox(viewer, box, i);
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, const std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> &pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    const auto &f_cloud = pointProcessorI->FilterCloud(inputCloud, 0.4, Eigen::Vector4f(-20, -6, -2, 1), Eigen::Vector4f(20, 7, 5, 1));
    // renderPointCloud(viewer, f_cloud, "f_cloud");

    const auto& [ground_cloud, non_ground_cloud] = pointProcessorI->SegmentPlane(f_cloud, 100, 0.2);
    renderPointCloud( viewer, ground_cloud, "ground", Color(0.0f, 1.0f, 0.0f) );
    // renderPointCloud( viewer, non_ground_cloud, "non_ground", Color(1.0f, 0.0f, 0.0f) );

    const auto& clusters = pointProcessorI->Clustering(non_ground_cloud, 0.75, 2, 1000);
    for(int i = 0; i < clusters.size(); i++)
    {
        const auto r = static_cast<double>(rand()) / RAND_MAX;
        const auto g = static_cast<double>(rand()) / RAND_MAX;
        const auto b = static_cast<double>(rand()) / RAND_MAX;
        renderPointCloud(viewer, clusters[i], "cluster_" + std::to_string(i), Color(r, g, b));

        const auto box = pointProcessorI->BoundingBox(clusters[i]);
        renderBox(viewer, box, i);
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
    // simpleHighway(viewer);

    auto pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
    auto stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.cbegin();
    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        cityBlock(viewer, pointProcessorI, pointProcessorI->loadPcd(streamIterator->string()));
        ++streamIterator;
        if(streamIterator == stream.cend()) streamIterator = stream.cbegin();

        viewer->spinOnce ();
    }
}