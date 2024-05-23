#include <Lidar_MCduo_2023/Lidar_declare.h>

void param_ROI(const Lidar_MCduo_2023::totalInfoConstPtr& signal) {
    lidar_state = signal->State;

    if(signal->State == 2) {
        ROI_xMax = ROI_xMax_re;
        ROI_yMin = ROI_yMin_2;
        ROI_yMax = ROI_yMax_2;
        ROI_zMin = ROI_zMin_2;
    }
    else if(signal->State == 4) {
        ROI_xMin = ROI_xMin_8;
        ROI_xMax = ROI_xMax_4;
        ROI_yMin = ROI_yMin_4;
        ROI_yMax = ROI_yMax_4;
        ROI_zMin = ROI_zMin_re;      
    }
    else if(signal->State == 8) {
        ROI_xMin = ROI_xMin_8;
        ROI_xMax = ROI_xMax_8;
        ROI_yMin = ROI_yMin_8;
        ROI_yMax = ROI_yMax_8;
        ROI_zMax = ROI_zMax_8;
        ROI_zMin = ROI_zMin_re; 
    }
    else {
        ROI_xMin = ROI_xMin_re;
        ROI_xMax = ROI_xMax_re;
        ROI_yMin = ROI_yMin_re;
        ROI_yMax = ROI_yMax_re;
        ROI_zMin = ROI_zMin_re;
        ROI_zMax = ROI_zMax_re;
    }
}

void ROI_process(const sensor_msgs::PointCloud2ConstPtr& scan) {
    ROI(scan, lidar_state);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "input_ROI");         //node name 
	ros::NodeHandle nh;                         //nodehandle

    nh.getParam("/ROI_node/switch_ROI", switch_ROI);
    nh.getParam("/ROI_node/ROI_xMin", ROI_xMin);
    nh.getParam("/ROI_node/ROI_xMax", ROI_xMax);
    nh.getParam("/ROI_node/ROI_yMin", ROI_yMin);
    nh.getParam("/ROI_node/ROI_yMax", ROI_yMax);
    nh.getParam("/ROI_node/ROI_zMin", ROI_zMin);
    nh.getParam("/ROI_node/ROI_zMax", ROI_zMax);

    nh.getParam("/ROI_node/ROI_zMin_2", ROI_zMin_2);
    nh.getParam("/ROI_node/ROI_yMin_2", ROI_yMin_2);
    nh.getParam("/ROI_node/ROI_yMax_2", ROI_yMax_2);
    nh.getParam("/ROI_node/ROI_xMax_4", ROI_xMax_4);
    nh.getParam("/ROI_node/ROI_yMin_4", ROI_yMin_4);
    nh.getParam("/ROI_node/ROI_yMax_4", ROI_yMax_4);
    nh.getParam("/ROI_node/ROI_xMin_8", ROI_xMin_8);
    nh.getParam("/ROI_node/ROI_xMax_8", ROI_xMax_8);
    nh.getParam("/ROI_node/ROI_yMin_8", ROI_yMin_8);
    nh.getParam("/ROI_node/ROI_yMax_8", ROI_yMax_8);
    nh.getParam("/ROI_node/ROI_zMax_8", ROI_zMax_8);
    nh.getParam("/ROI_node/ROI_xMin_re", ROI_xMin_re);
    nh.getParam("/ROI_node/ROI_xMax_re", ROI_xMax_re);
    nh.getParam("/ROI_node/ROI_yMin_re", ROI_yMin_re);
    nh.getParam("/ROI_node/ROI_yMax_re", ROI_yMax_re);
    nh.getParam("/ROI_node/ROI_zMin_re", ROI_zMin_re);
    nh.getParam("/ROI_node/ROI_zMax_re", ROI_zMax_re);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/0_UDP_others", 1, ROI_process); //0_UDP_others
    ros::Subscriber sub_totalInfo = nh.subscribe<Lidar_MCduo_2023::totalInfo> ("/Totalcom", 1, param_ROI);

    pub_ROI = nh.advertise<sensor_msgs::PointCloud2> ("/1_ROI_PCL2", 1);

	ros::spin();
}
