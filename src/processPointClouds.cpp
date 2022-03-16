// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

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
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Voxel Grid downsampling
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    //Region based filtering with Cropbox
    pcl::CropBox<PointT> crop_box (false);
    crop_box.setInputCloud (cloud_filtered);
    crop_box.setMin (minPoint);
    crop_box.setMax (maxPoint);
    crop_box.filter (*cloud_filtered);
    
    // (Optional) filter ego car roof points
    pcl::CropBox<PointT> crop_box2 (false);
    crop_box2.setInputCloud (cloud_filtered);
    Eigen::Vector4f roof_min = Eigen::Vector4f(-1.35f, -1.45f, -0.95f, 1.0f);
    Eigen::Vector4f roof_max = Eigen::Vector4f(2.55f, 1.35f, -0.25f, 1.0f);
    crop_box2.setMin (roof_min);
    crop_box2.setMax (roof_max);
    crop_box2.setNegative(true);
    crop_box2.filter (*cloud_filtered);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
   // Extract the inliers
    pcl::ExtractIndices<PointT> extract;

    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT> ());
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    
    //Separate planes
    extract.setNegative (false);
    extract.filter (*plane);

    //Separate obstacles
    extract.setNegative (true);
    extract.filter(*obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr RansacPlane(int maxIterations, float distanceTol,const pcl::PointCloud<PointT>& cloud)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    // For max iterations
    for (int it=0; it<maxIterations; it++)
    {
        // Randomly sample subset and fit line
        // These two should not be the same index
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % cloud.points.size());
        }

        auto iter = inliers.begin();
        float x1 = cloud.points[*iter].x;
        float y1 = cloud.points[*iter].y;
        float z1 = cloud.points[*iter].z;
        iter++;
        float x2 = cloud.points[*iter].x;
        float y2 = cloud.points[*iter].y;
        float z2 = cloud.points[*iter].z;
        iter++;
        float x3 = cloud.points[*iter].x;
        float y3 = cloud.points[*iter].y;
        float z3 = cloud.points[*iter].z;
        
        float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
        float D = -(A*x1 + B*y1 + C*z1);

        // Measure distance between every point and fitted line
        for (int i=0;  i<cloud.points.size(); i++)
        {
            auto point = cloud.points[i];
            float distance = fabs(A*point.x + B*point.y + C*point.z + D) / sqrt(A*A + B*B + C*C);

            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol)
            {
                inliers.insert(i);
            }
        }

        if(inliers.size() > inliersResult.size())
        {
            std::swap(inliers, inliersResult);
        }
    }

    pcl::PointIndices::Ptr inliers_indices (new pcl::PointIndices ());
    for (auto inlier : inliersResult)
    {
        inliers_indices->indices.push_back(inlier);
    }
	// Return indicies of inliers from fitted line with most inliers
	return inliers_indices;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    bool use_my_ransac = false; 

    if (use_my_ransac)
    {
        inliers = RansacPlane(maxIterations, distanceThreshold, *cloud);
    }
    else
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxIterations);
        seg.setDistanceThreshold (distanceThreshold);
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
    }

    // Create the filtering object

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


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

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (int cluster_idx = 0; cluster_idx < cluster_indices.size(); cluster_idx++)
    {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        pcl::PointIndices::Ptr index_ptr (new pcl::PointIndices());
        *index_ptr = cluster_indices[cluster_idx];
        extract.setIndices(index_ptr);
        extract.setNegative(false);
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT> ());
        extract.filter(*cluster_cloud);
        clusters.push_back(cluster_cloud);
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