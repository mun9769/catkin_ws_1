#include <Lidar_MCduo_2023/Lidar_declare.h>

void signal_process(const Lidar_MCduo_2023::totalInfoConstPtr& signal) {
    switch_RanSaC = (signal->State == 4) ? true : false;
}

void ransac_process(const sensor_msgs::PointCloud2ConstPtr& aft_ROI){
    PCXYZI::Ptr RANSAC_cloud (new PCXYZI);
    pcl::fromROSMsg(*aft_ROI,*RANSAC_cloud);

    if( switch_RanSaC ) {
        for (int i = 0; i < RANSAC_cloud->points.size(); i++) {
            RANSAC_cloud->points[i].intensity = RANSAC_cloud->points[i].z;
            RANSAC_cloud->points[i].z = 0;
        }
        RanSaC(RANSAC_cloud);
    }
    else{
        sensor_msgs::PointCloud2 output; 
        pub_process(*RANSAC_cloud, output); 
        pub_RS.publish(output);
        PCXYZI empty_cloud;
        pub_process(empty_cloud, output);
        pub_walls.publish(output);

        Lidar_MCduo_2023::RANSAC_points_arr output_ransac;
        Lidar_MCduo_2023::RANSAC_points tmp;
        for(int i = 0; i < RANSAC_cloud->points.size(); i++) {
            tmp.Rx = RANSAC_cloud->points[i].x;
            tmp.Ry = RANSAC_cloud->points[i].y;
            tmp.Rz = RANSAC_cloud->points[i].z;
            tmp.Ri = RANSAC_cloud->points[i].intensity;
            output_ransac.data.push_back(tmp);
        }
        pub_RS_RPA.publish(output_ransac);
        printf("\033[38;2;139;232;229mRANSAC runtime\033[0m : \033[0mOFF\033[0m\n");
    }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ransac"); //node name 
	ros::NodeHandle nh;         //nodehandle
    nh.getParam("/ransac_node/switch_RanSaC", switch_RanSaC);
    nh.getParam("/ransac_node/ransac_distanceThreshold", ransac_distanceThreshold);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/1_ROI_PCL2", 1, ransac_process);
    ros::Subscriber sub_totalInfo = nh.subscribe<Lidar_MCduo_2023::totalInfo> ("/Totalcom", 1, signal_process);
    pub_RS = nh.advertise<sensor_msgs::PointCloud2> ("/2_1_RANSAC_others_PCL2", 1);
    pub_walls = nh.advertise<sensor_msgs::PointCloud2> ("/2_2_RANSAC_walls_PCL2", 1);
    pub_RS_RPA = nh.advertise<Lidar_MCduo_2023::RANSAC_points_arr> ("/2_3_RANSAC_points_RPA", 1);
    
	ros::spin();
}
