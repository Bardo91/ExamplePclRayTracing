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

    double minimumResolutionVoxelTracer = 0.02;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> rayTracer(minimumResolutionVoxelTracer);
    rayTracer.setInputCloud(cloud.makeShared());
    rayTracer.addPointsFromInputCloud();
    //rayTracer.defineBoundingBox();

    viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "surface");

    // VISUALIZATION TEST
    const int MAX_SAMPLES = 10;
    for(unsigned sample = 0 ; sample <MAX_SAMPLES ; sample++ ){
        Eigen::Vector3f cameraOrigin =  {float(rand())/RAND_MAX*2-1,1,float(rand())/RAND_MAX*2-1};
        Eigen::Vector3f rayEnd = {float(rand())/RAND_MAX*2-1,-1,float(rand())/RAND_MAX*2-1};

        Eigen::Vector3f rayDir = rayEnd - cameraOrigin;

        std::vector<int> indices;
        rayTracer.getIntersectedVoxelIndices(cameraOrigin, rayDir, indices);

        std::cout << "Raytracing from " <<cameraOrigin.transpose() << ", with direction "  << rayDir.transpose()  << ", and resolution of " << minimumResolutionVoxelTracer<<std::endl;
        std::cout << "Found "<< indices.size() <<"collisions" <<std::endl;

        double r = double(rand())/RAND_MAX;
        double g = double(rand())/RAND_MAX;
        double b = double(rand())/RAND_MAX;

        pcl::PointXYZ p1(cameraOrigin[0], cameraOrigin[1], cameraOrigin[2]);
        pcl::PointXYZ p2(rayEnd[0], rayEnd[1], rayEnd[2]);
        viewer.addLine<pcl::PointXYZ>(p1,p2, "ray_"+std::to_string(sample));
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "ray_"+std::to_string(sample));

        for(auto &p: indices){
            viewer.addSphere(cloud[p], stepSize, "sphere"+std::to_string(sample)+std::to_string(p));
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "sphere"+std::to_string(sample)+std::to_string(p));
        }
    }


    //SPEED TEST
    double  timeAccum = 0;
    int SPEED_TEST_SAMPLES = 10000;
    for(unsigned sample = 0 ; sample <SPEED_TEST_SAMPLES ; sample++ ){
        Eigen::Vector3f cameraOrigin =  {float(rand())/RAND_MAX*2-1,1,float(rand())/RAND_MAX*2-1};
        Eigen::Vector3f rayEnd = {float(rand())/RAND_MAX*2-1,-1,float(rand())/RAND_MAX*2-1};

        Eigen::Vector3f rayDir = rayEnd - cameraOrigin;

        std::vector<int> indices;
        auto t0 = std::chrono::high_resolution_clock::now();
        rayTracer.getIntersectedVoxelIndices(cameraOrigin, rayDir, indices);
        auto t1 = std::chrono::high_resolution_clock::now();
        timeAccum += std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
    }

    timeAccum /=SPEED_TEST_SAMPLES;
    std::cout <<"AVG. time per sample after " << SPEED_TEST_SAMPLES<< " samples: " << timeAccum << " microseconds." << std::endl;
    while(!viewer.wasStopped()){
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        viewer.spinOnce(15);
    }



}
