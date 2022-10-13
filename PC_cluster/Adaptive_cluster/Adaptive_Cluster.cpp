#include <opencv/highgui.h>

// PCL
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// others
#include <math.h>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <typeinfo>
#include <cmath>
#include <algorithm>
#include <queue>
#include <pybind11/complex.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>

// #include <boost/shared_ptr.hpp>
// #include <boost/array.hpp>

using namespace Eigen;
namespace py = pybind11;

class Adaptive_Cluster{
    public:

    double delta_distance=0.1;
    int min_size = 3;
    int max_size = 10000;
    double z_axis_min = -10;
    double z_axis_max =  10;

    // parameter fix
    std::string sensor_model = "HDL-64E";
    float vert_res = 0.4;
    int regions_idxMax_ = 13; // max >= 13 in current region
    int regions_[13];

    Adaptive_Cluster(double delta_distance, int min_size, int max_size, double z_axis_min, double z_axis_max):delta_distance(delta_distance),min_size(min_size),max_size(max_size),z_axis_min(z_axis_min),z_axis_max(z_axis_max) {
        if (sensor_model.compare("VLP-16") == 0) { 
            // Velodyne VLP 16's vertical anglular resolution is approximately 2
            vert_res = 2;
        } else if (sensor_model.compare("HDL-32E") == 0) { 
            // Velodyne HDL 32E's vertical anglular resolution is approximately 1.3
            vert_res = 1.3;
        } else if (sensor_model.compare("HDL-64E") == 0) { 
            // Velodyne HDL 64E's vertical anglular resolution is approximately 0.4
            vert_res = 0.4;
        } else {
            std::cout << "LiDAR model select Error" << std::endl;
        }

        float d_past = 0;
        for (int i = 0; i < regions_idxMax_; i++ ) {
            auto d_current = delta_distance * (i+1) / (2* tan(  (vert_res/2) * M_PI / 180 ) );
            regions_[i] = round(d_current - d_past);
            d_past = d_current;
        }

        std::cout <<"\t The radius of regions (First 3): " << regions_[0] << " " << regions_[1] << " " << regions_[2] << " " << std::endl;
        
        
        // Oringional Manully assignment for regions
        // if (sensor_model.compare("VLP-16") == 0) {
        //     regions_[0] = 2; regions_[1] = 3; regions_[2] = 3; regions_[3] = 3; regions_[4] = 3;
        //     regions_[5] = 3; regions_[6] = 3; regions_[7] = 2; regions_[8] = 3; regions_[9] = 3;
        //     regions_[10]= 3; regions_[11]= 3; regions_[12]= 3; regions_[13]= 3;
        // } else if (sensor_model.compare("HDL-32E") == 0) {
        //     regions_[0] = 4; regions_[1] = 5; regions_[2] = 4; regions_[3] = 5; regions_[4] = 4;
        //     regions_[5] = 5; regions_[6] = 5; regions_[7] = 4; regions_[8] = 5; regions_[9] = 4;
        //     regions_[10]= 5; regions_[11]= 5; regions_[12]= 4; regions_[13]= 5;
        // } else if (sensor_model.compare("HDL-64E") == 0) {
        //     regions_[0] = 14;  regions_[1] = 14; regions_[2] = 14; regions_[3] = 15; regions_[4] = 14;
        //     regions_[5] = 14;  regions_[6] = 14; regions_[7] = 14; regions_[8] = 15; regions_[9] = 14;
        //     regions_[10] = 14; regions_[11] = 14; regions_[12] = 14; regions_[13] = 15;
        // } else {
        //     std::cout << "LiDAR model select Error" << std::endl;
        // }

    }
    
    std::array<int, 131072>  Adaptive_cluster(py::array_t<double> input_array_x_,py::array_t<double> input_array_y_, py::array_t<double> input_array_z_, py::array_t<double> mask_, int num_points){
    // use np.reshape(-1) to strip the 2D image to 1d array
    std::array<int, 131072> label_instance=std::array<int, 131072>();
    auto input_array_x = input_array_x_.request();
    double *ptr_x = (double *) input_array_x.ptr;
    auto input_array_y = input_array_y_.request();
    double *ptr_y = (double *) input_array_y.ptr;
    auto input_array_z = input_array_z_.request();
    double *ptr_z = (double *) input_array_z.ptr;
    auto mask = mask_.request();
    double *ptr_mask = (double *) mask.ptr;


    // 1.  Get the point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    
    cloud.width=num_points;
    cloud.height=1;
    cloud.is_dense=false;
    cloud.points.resize(cloud.width*cloud.height);
    std::vector<int> index_original;
    int count_points=0;
    for (int i=0; i<131072;i++){
        if (ptr_mask[i]>0){
            cloud.points[count_points].x=ptr_x[i];
            cloud.points[count_points].y=ptr_y[i];
            cloud.points[count_points].z=ptr_z[i];
            index_original.push_back(i);
            count_points+=1; 
        }
    }
    *cloud_ptr = cloud;

    cloud_ptr->points.resize (cloud_ptr->width * cloud_ptr->height);

    std::cout  << "\t 1. PCL Point Cloud Get " << cloud.width << " "<< cloud.height << std::endl;

    // 2. Remove points out of the range of objects
    pcl::IndicesPtr pc_indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> pt;
    pt.setInputCloud(cloud_ptr);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(z_axis_min, z_axis_max);
    pt.filter(*pc_indices);

    std::cout << "\t 2. Remove the points out of the scale "<< pc_indices->size()  << std::endl;

    // 3. Divide the point cloud into nested circular regions
    std::array<std::vector<int>, 13> indices_array;
    for(int i = 0; i < pc_indices->size(); i++) {
        float range = 0.0;
        for(int j = 0; j < 13; j++) {
            float d2 =  cloud_ptr->points[(*pc_indices)[i]].x * cloud_ptr->points[(*pc_indices)[i]].x +
                        cloud_ptr->points[(*pc_indices)[i]].y * cloud_ptr->points[(*pc_indices)[i]].y +
                        cloud_ptr->points[(*pc_indices)[i]].z * cloud_ptr->points[(*pc_indices)[i]].z;
            if(d2 > range * range && d2 <= (range+regions_[j]) * (range+regions_[j])) {
            indices_array[j].push_back((*pc_indices)[i]);
            break;
            }
            range += regions_[j];
        }
    }

    std::cout << "\t 3. Get the region points (indicese): "
              << indices_array[0].size() << "\t" 
              << indices_array[1].size() << "\t"
              << indices_array[2].size() << "\t"
              << indices_array[3].size() << "\t"
              << std::endl;


    // 4. Implement euclidean culstering based on the region
    float tolerance = 0.0;
    std::vector<std::vector<pcl::PointIndices>> regionClusters_indices;
    
    for(int i = 0; i < regions_idxMax_; i++) {
        tolerance += delta_distance;
        if(indices_array[i].size() > min_size) {
        // std::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
        pcl::IndicesPtr tree_indices(new std::vector<int>);
        *tree_indices = indices_array[i];
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_ptr, tree_indices);       
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_ptr);
        ec.setIndices(tree_indices);
        ec.extract(cluster_indices);        
        regionClusters_indices.push_back(cluster_indices);
        }
    }
    std::cout << "\t 4. Get the regional clusters " << regionClusters_indices.size() << std::endl;

    int cluster_index = 1;
    for (int j = 0; j<regionClusters_indices.size(); j++) {
        auto cluster_indices = regionClusters_indices[j];
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices) label_instance[index_original[idx]]=cluster_index;
        cluster_index++;
        }  
    }

    std::cout << "\t 5. Update the instance cluster labels " << regionClusters_indices.size() << cluster_index << std::endl;
  

    return label_instance;



    }
};


PYBIND11_MODULE(Adaptive_Cluster, m) {
    py::class_<Adaptive_Cluster>(m, "Adaptive_Cluster")
    	.def(py::init<float, int, int, float, float>())
        .def("Adaptive_cluster", &Adaptive_Cluster::Adaptive_cluster);

}