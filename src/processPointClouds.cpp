// PCL lib Functions for processing point clouds 
#include <unordered_set>
#include "processPointClouds.h"


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


//typename is used whenever a pointer that depends on a template

//The function will return the downsampled cloud with only points that were inside the region specified. 
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time filtering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes,filterRes,filterRes);
    vg.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT> ());
    pcl::CropBox<PointT> region(true);// dealing with points inside the cropbox
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // filter static points relected from roof of veh
    std::vector<int> indices;       		// to store roof points' indices

    pcl::CropBox<PointT> roof(true);		// dealing with points inside the cropbox
    roof.setMin(Eigen::Vector4f {-1.5, -1.7, -1, 1});
    roof.setMax(Eigen::Vector4f {2.6, 1.7, -0.4, 1});
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
 
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int index : indices)
	inliers->indices.push_back(index);
    
    pcl::ExtractIndices<PointT> extract;	// take out the roof points from region
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    // push the inlier's point to the plane cloud
    for (int index : inliers->indices)
    	planeCloud->points.push_back(cloud->points[index]);

    // Extract the inliers, this lib subtracts the plane cloud from the input cloud, the rest points are obstcles cloud
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

/* SegmentPlane
The function accepts a point cloud, max iterations, and distance tolerance as arguments.
Segmentation uses an iterative process. More iterations have a chance of returning better
results but take longer. The segmentation algorithm fits a plane to the points and uses 
the distance tolerance to decide which points belong to that plane. A larger tolerance 
includes more points in the plane.
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);		// fit a plane
    seg.setMethodType (pcl::SAC_RANSAC);		// random sample consensus
    seg.setMaxIterations (maxIterations);		// max iteration
    seg.setDistanceThreshold (distanceThreshold);	// threshold
    
    seg.setInputCloud (cloud);				// feed in point cloud
    // operator (*) dereferenced ptr inliers, which represents points belong to the plane(road)
    seg.segment (*inliers, *coefficients);		
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    
    // Extract the inliers
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

// implementation of RANSAC plane fitting
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// Time segmentation process
    	auto startTime = std::chrono::steady_clock::now();
	// For max iterations
	while(maxIterations--)
	{
		std::unordered_set<int> inliersTmp;
		// Randomly sample subset and fit plane
		// randomly select three indices, inner while prevent selecting identical indices
		while (inliersTmp.size() < 3)
			inliersTmp.insert(rand()%(cloud->points.size()));
		
		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto itr = inliersTmp.begin();
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
		// fit a plane using (x1,y1,z1), (x2,y2,z2) and (x3,y3,z3), lineformula: Ax+By+Cz+D=0
		float A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float B = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float C = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		float D = -(A*x1 + B*y1 + C*z1);
		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		// d = |A*x+B*y+C*z+D|/sqrt(A^2+B^2+C^2)
		for (int index = 0; index < cloud->points.size(); index++)
		{
			// chk IF index exists already
			if (inliersTmp.count(index) > 0)
				continue;
			
			float x = cloud->points[index].x;
			float y = cloud->points[index].y;
			float z = cloud->points[index].z;
			float dis = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

			if (dis <= distanceTol)
				inliersTmp.insert(index);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliersTmp.size() > inliersResult.size())
			inliersResult = inliersTmp;
	}
	
	// seperate inliers and outliers and push them to a pair of clouds
	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
        typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
	
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
	// track time
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC plane fit took " << elapsedTime.count() << " milliseconds" << std::endl;

	return segResult;
}

// pcl lib euclidean clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;	// i.e. cluster_indices[0] contains all indices of the first cluster in our point cloud.
    
    pcl::EuclideanClusterExtraction<PointT> ec;		// The euclidean clustering object
    ec.setClusterTolerance (clusterTolerance);   	// distance tolerance
    ec.setMinClusterSize (minSize);			// If a cluster is really small, it’s probably just noise
    ec.setMaxClusterSize (maxSize);			// If a cluster is very large it might just be that many other clusters are overlapping,
    ec.setSearchMethod (tree);				// kd tree
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    // use result indices to form cluters
    for (pcl::PointIndices cluster_index : cluster_indices)
    {
	typename pcl::PointCloud<PointT>::Ptr cloudTmp (new pcl::PointCloud<PointT>);
    	for (int index : cluster_index.indices)	
    	    cloudTmp->points.push_back(cloud->points[index]);
	
	cloudTmp->width = cloudTmp->points.size();
	cloudTmp->height = 1;
	cloudTmp->is_dense = true;

        clusters.push_back(cloudTmp);			// vector to store different clusters
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices & cluster, std::vector<bool> &processed, KdTree<PointT>* tree, float distanceTol)
{
	cluster.indices.push_back(indice);
	processed[indice] = true;
	// find all indices of nearby point[i]	
	std::vector<int> nearby = tree->search(cloud->points[indice], distanceTol);
	// Iterate through each nearby point
	for (int id : nearby)
	{
		if (processed[id] == false)
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
	}	
}

template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<pcl::PointIndices> clusters;
	// to track which points have been processed
 	std::vector<bool> processed(cloud->points.size(), false);
	// Iterate through each point
	int i = 0;
	while (i < cloud->points.size())
	{
		if (processed[i] == true)
		{
			i++;
			continue;
		}
		// non processed point
		pcl::PointIndices cluster;	// Create cluster
		clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);    // push grouped points
		i++;
	}

	return clusters;
}

// TZ's KD tree euclidean clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringKDTree(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Create and build the KdTree object
    KdTree<PointT>* tree = new KdTree<PointT>();
    for(int index = 0; index < cloud->points.size(); index++)
    	tree->insert(cloud->points[index],index);
    // start clustering using kd-tree
    std::vector<pcl::PointIndices> cluster_indices;		// i.e. cluster_indices[0] contains all indices of the first cluster in our point cloud. 
    cluster_indices = euclideanCluster(cloud, tree, clusterTolerance);

    // use result indices to form cluters
    for (pcl::PointIndices cluster_index : cluster_indices)
    {
	// check IF cluster is too small or too big
	if (cluster_index.indices.size() < minSize || cluster_index.indices.size() > maxSize)
	    continue;
	
	typename pcl::PointCloud<PointT>::Ptr cloudTmp (new pcl::PointCloud<PointT>);
    	for (int index : cluster_index.indices)	
    	    cloudTmp->points.push_back(cloud->points[index]);
	
	cloudTmp->width = cloudTmp->points.size();
	cloudTmp->height = 1;
	cloudTmp->is_dense = true;

        clusters.push_back(cloudTmp);			// vector to store different clusters
    }

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
