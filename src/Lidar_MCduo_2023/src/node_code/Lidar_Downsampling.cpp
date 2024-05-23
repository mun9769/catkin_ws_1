// #include <Lidar_MCduo_2023/Lidar_declare.h>

// void signal_process(const Lidar_MCduo_2023::totalInfoConstPtr& signal) {
//     switch_DownSampling = (signal->State == 4) ? true : false;
// }

// void ds_process(const sensor_msgs::PointCloud2ConstPtr& scan){
//     PCXYZI::Ptr combined_points(new PCXYZI ());

//     PCXYZI rawData;
//     pcl::fromROSMsg(*scan,rawData);
//     PCXYZI::Ptr downsampledCloud (new PCXYZI);
//     if(switch_DownSampling) {
//         DownSampling(rawData, downsampledCloud, combined_points);
//     }
//     else{
//         *combined_points = rawData + objs_points;
//         sensor_msgs::PointCloud2 output;
//         pub_process(*combined_points, output);
//         pub_DS.publish(output);
//     }
// }

// void save_points_process(const sensor_msgs::PointCloud2ConstPtr& scan) { 
//     PCXYZI tmp;
//     pcl::fromROSMsg(*scan, tmp);
//     objs_points = tmp;
// }

// int main(int argc, char** argv){

// 	ros::init(argc, argv, "Lidar_Downsampling");
// 	ros::NodeHandle nh;
//     nh.getParam("/Downsampling_node/voxel_size_x", voxel_size_x);
//     nh.getParam("/Downsampling_node/voxel_size_y", voxel_size_y);
//     nh.getParam("/Downsampling_node/voxel_size_z", voxel_size_z);

// 	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/2_2_RANSAC_walls_PCL2", 1, ds_process);
//     ros::Subscriber sub_ransac = nh.subscribe<sensor_msgs::PointCloud2> ("/2_1_RANSAC_others_PCL2", 1, save_points_process);
//     ros::Subscriber sub_totalInfo = nh.subscribe<Lidar_MCduo_2023::totalInfo> ("/Totalcom", 1, signal_process);
// 	pub_DS = nh.advertise<sensor_msgs::PointCloud2> ("/3_Downsampling_PCL2", 1);
    
//     ros::spin();
// }