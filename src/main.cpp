#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <chrono>

int main(int _argc, char **_argv){


    // Create flat surface
    pcl::PointCloud<pcl::PointXYZ> cloud;
    double stepSize = 0.01;
    for(double  i = -1; i < 1; i +=stepSize){
        for(double j = -1; j < 1; j+=stepSize){
            cloud.push_back(pcl::PointXYZ(i,(double(rand())/RAND_MAX-0.5)*0.1,j));
        }
    }

    pcl::visualization::PCLVisualizer viewer("3d viewer");

    double minimumResolutionVoxelTracer = 0.05;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> rayTracer(minimumResolutionVoxelTracer);
    rayTracer.setInputCloud(cloud.makeShared());
    rayTracer.defineBoundingBox();

    Eigen::Vector3f cameraOrigin =  {0,1,0};
    Eigen::Vector3f ejRayDir = {0,-1,0};

    //pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::AlignedPointTVector collisionCenters;
    //rayTracer.getIntersectedVoxelCenters(cameraOrigin, ejRayDir, collisionCenters);

    std::vector<int> indices;
    rayTracer.getIntersectedVoxelIndices(cameraOrigin, ejRayDir, indices);

    std::cout << "Raytracing from " <<cameraOrigin.transpose() << ", with direction "  << ejRayDir.transpose()  << ", and resolution of " << minimumResolutionVoxelTracer<<std::endl;
    std::cout << "Found "<< indices.size() <<"collisions" <<std::endl;

    viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "surface");
    pcl::PointXYZ p1(0,1,0);
    pcl::PointXYZ p2(0,-4,0);
    viewer.addLine<pcl::PointXYZ>(p1,p2, "ray1");

    //for(auto &p: collisionCenters){
    //    viewer.addSphere(p, 0.1, "sphere"+std::to_string(rand()));
    //}
    while(!viewer.wasStopped()){
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        viewer.spinOnce(15);
    }


}
