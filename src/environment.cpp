/* \author Aaron Brown */

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

/* --------------------------------------------------------------
 * Init some elements on the simple highway for visualization
 * ------------------------------------------------------------*/
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

/* --------------------------------------------------------------
 *       Coding exercise for simple highway. (one frame)
 * ------------------------------------------------------------*/
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // create a Lidar's object on the heap, constructor takes two arguments:
    // cars and the slope of the ground
    Lidar* lidar = new Lidar(cars, 0);
    // generate a lidar point cloud (PCL XYZ) using func scan() 
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud); 		//render rays to environment
    //renderPointCloud(viewer, inputCloud, "inputCloud");	//render points cloud to environment

    // TODO:: Create point cloud processor
    // create a point cloud processor on the heap
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    
    /* -----------------------------------------------------
     *          Obstacles and Plane Segmentation 
     * -----------------------------------------------------*/   
    // seperate lidar's scan to a pair of clouds: {obstacles, plane} using PCL lib
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = 
    						//pointProcessor->SegmentPlane(inputCloud, 100, 0.2);

    // seperate lidar's scan to a pair of clouds: {obstacles, plane} using TZ's RANSAC implementation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = 
    pointProcessor->RansacPlane(inputCloud, 100, 0.2); 
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    /* -----------------------------------------------------
     *          Obstacles Clustering KD-Tree 
     * -----------------------------------------------------*/	
    // clustering using pcl
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    // clustering using TZ's KD tree implementation
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor
				->ClusteringKDTree(segmentCloud.first, 1.0, 3, 30);
		
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);	// num of points in the curr cluster
	// The renderPointCloud is expecting each point cloud to have a unique identifier
	// so clusters are counted with clusterId and appended to the obstCloud string. 
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // render boundary boxes around each cluster
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }   
}


/* --------------------------------------------------------------
 *       Coding exercise for multiple frame
 * ------------------------------------------------------------*/
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //renderPointCloud(viewer,inputCloud,"inputCloud");
    /* -----------------------------------------------------
     *  0. Filtering, voxel grid point reduction + crop box
     * -----------------------------------------------------*/  
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, 
    Eigen::Vector4f (-10, -6, -5, 1), Eigen::Vector4f (40, 6, 2, 1));
    //renderPointCloud(viewer,filterCloud,"filterCloud");
    /* -----------------------------------------------------
     *         1. Obstacles and Plane Segmentation 
     * -----------------------------------------------------*/ 
    // segmentation using pcl lib
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = 
    //pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    // segmentation using RANSAC
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = 
    pointProcessorI->RansacPlane(filterCloud, 100, 0.2); 
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0)); 	// render obstacles
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0)); 	// render road

    /* -----------------------------------------------------
     *         2. Obstacles Clustering KD-Tree 
     * -----------------------------------------------------*/	
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI
     				    	->ClusteringKDTree(segmentCloud.first, 0.4, 15, 1000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);	// num of points in the curr cluster
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);

        // render boundary boxes around each cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


/* --------------------------------------------------------------
 *         SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
 * ------------------------------------------------------------*/
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
    
    // viewer is a pointer of our simulation environment, it handles all the graphic func
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // camera indicates the viewing selection of the simulation environment
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // create and process things, i.e. lidar, cars, road, in our environment
    //simpleHighway(viewer);
    /************** For one PCD file / simple highway *************
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }  

    ********************* For stream PCD files ********************/
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // a folder directory that contains all the sequentially ordered pcd files
    // streamPcd returns a chronologically ordered vector of all those file names
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd
    							("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // PCL Viewer Update Loop 
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
