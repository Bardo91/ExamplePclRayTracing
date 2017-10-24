#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <chrono>

#include <time.h>

int main(int _argc, char **_argv){

	// Random seed
	srand (time(NULL));

    // Create flat surface
    pcl::PointCloud<pcl::PointXYZ> cloud;
    double stepSize = 0.01;
    for(double  i = -1; i < 1; i +=stepSize){
        for(double j = -1; j < 1; j+=stepSize){
            cloud.push_back(pcl::PointXYZ(i,(double(rand())/RAND_MAX-0.5)*0.02,j));
        }
    }
	int num_plane_points = cloud.size();

	// Insert some apples
	for(int a = 0; a < 20; a++)
	{
		double c_x = (2.0*double(rand())/RAND_MAX-1.0);
		double c_y = (2.0*double(rand())/RAND_MAX-1.0);
		double c_z = (double(rand())/RAND_MAX-0.5)*1.0;
		double rho = 0.1+0.05*double(rand())/RAND_MAX;

		// Pick random point on surface of sphere
		for(int b = 0; b < 1000; b++){
			// see http://mathworld.wolfram.com/SpherePointPicking.html
			double u = (double(rand())/RAND_MAX);
			double v = (double(rand())/RAND_MAX);

			double theta = 2*M_PI*u;
			double phi = acos(2*v-1);
			double a_x = c_x + rho*cos(theta)*sin(phi);
			double a_y = c_y + rho*sin(theta)*sin(phi);
			double a_z = c_z + rho*cos(phi);
        	cloud.push_back(pcl::PointXYZ(a_x, a_y, a_z));
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

	// fix the camera position
	Eigen::Vector3f cameraOrigin =  {float(rand())/RAND_MAX*2-1,1,float(rand())/RAND_MAX*2-1};

	// Camera field of view
	// (defines the "pixels")
	double min_x_angle = -M_PI / 6.0;
	double max_x_angle = +M_PI / 6.0;
	double x_angle_centre = - M_PI / 2.0;
	double step_x_angle = (max_x_angle - min_x_angle) / 8;
	double min_y_angle = -M_PI / 6.0;
	double max_y_angle = +M_PI / 6.0;
	double y_angle_centre = + M_PI / 2.0;
	double step_y_angle = (max_y_angle - min_y_angle) / 8;

	int sample = 0;

	// Go through all camera "pixels"
    for(double x_angle = min_x_angle ; x_angle <= max_x_angle ; x_angle+=step_x_angle ){
        for(double y_angle = min_y_angle ; y_angle <= max_y_angle ; y_angle+=step_y_angle ){

			
			// laser direction
			float rho = 2.0; // arbitrary
		    Eigen::Vector3f rayEnd = {	cameraOrigin[0] + (float)(rho*cos(x_angle+x_angle_centre)*sin(y_angle+y_angle_centre)), 
										cameraOrigin[1] + (float)(rho*sin(x_angle+x_angle_centre)*sin(y_angle+y_angle_centre)), 
										cameraOrigin[2] + (float)(rho*cos(y_angle+y_angle_centre))};

			// laser vector
		    Eigen::Vector3f rayDir = rayEnd - cameraOrigin;

		    std::vector<int> indices;
		    rayTracer.getIntersectedVoxelIndices(cameraOrigin, rayDir, indices);

		    std::cout << "Raytracing from " <<cameraOrigin.transpose() << ", with direction "  << rayDir.transpose()  << ", and resolution of " << minimumResolutionVoxelTracer<<std::endl;
		    std::cout << "Found "<< indices.size() <<"collisions" <<std::endl;

		    double r = double(rand())/RAND_MAX;
		    double g = double(rand())/RAND_MAX;
		    double b = double(rand())/RAND_MAX;
	

			const bool FIRST_ONLY = true;
			if(FIRST_ONLY)
			{
				// Plot only the intersection point

				int first_index = -1;
				float first_distance = 100000;
				
				// Find the closest point
				for(auto &p: indices){
					pcl::PointXYZ point = cloud[p];
					Eigen::Vector3f vector = {point.x - cameraOrigin[0], point.y - cameraOrigin[1], point.z - cameraOrigin[2]};
					float distance = vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2];
					if(distance < first_distance){
						first_distance = distance;
						first_index = p;
					}
				}

				// Circle the points where intersection occur
				if(first_index >= 0)	{

					if( first_index >= num_plane_points ){
						// apple
						r=1.0;g=0.0;b=0.0;
					}else{
						// plane
						r=0.0;g=0.6;b=0.0;
					}

					// draw ray, only up to point
					pcl::PointXYZ p1(cameraOrigin[0], cameraOrigin[1], cameraOrigin[2]);
		    		pcl::PointXYZ p2(cloud[first_index].x, cloud[first_index].y, cloud[first_index].z);
		    		viewer.addLine<pcl::PointXYZ>(p1,p2, "ray_"+std::to_string(first_index));
					viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "ray_"+std::to_string(first_index));
					
					// draw point
			    	viewer.addSphere(cloud[first_index], stepSize*2.0, "sphere"+std::to_string(first_index)+std::to_string(first_index));
			    	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "sphere"+std::to_string(first_index)+std::to_string(first_index));
				}else{
					// draw ray
					r=0.0;g=1.0;b=1.0;
					pcl::PointXYZ p1(cameraOrigin[0], cameraOrigin[1], cameraOrigin[2]);
		    		pcl::PointXYZ p2(rayEnd[0], rayEnd[1], rayEnd[2]);
		    		viewer.addLine<pcl::PointXYZ>(p1,p2, "ray_"+std::to_string(sample));
					viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "ray_"+std::to_string(sample));
				}
			} else {

				// Plot all points that intersect

				pcl::PointXYZ p1(cameraOrigin[0], cameraOrigin[1], cameraOrigin[2]);
		    	pcl::PointXYZ p2(rayEnd[0], rayEnd[1], rayEnd[2]);
		    	viewer.addLine<pcl::PointXYZ>(p1,p2, "ray_"+std::to_string(sample));
				viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "ray_"+std::to_string(sample));

				// Circle the points where intersection occur
				for(auto &p: indices){
				    viewer.addSphere(cloud[p], stepSize, "sphere"+std::to_string(sample)+std::to_string(p));
				    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r,g,b, "sphere"+std::to_string(sample)+std::to_string(p));
				}
			}
			sample++;
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
