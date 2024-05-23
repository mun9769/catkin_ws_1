#include <Lidar_MCduo_2023/Lidar_declare.h>

Fps FPS1;

void param_clustering(const Lidar_MCduo_2023::totalInfoConstPtr& signal) {
    if(signal->State == 1) {
        //장애물
        DBscan_eps = DBscan_eps_1;
        DBscan_minPts = DBscan_minPts_1;
        DB_MinClusterSize = DB_MinClusterSize_1;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_1;
        switch_TC_filter = false;
    }
    else if(signal->State == 2) {
        //표지판 
        DBscan_eps = DBscan_eps_2;
        DBscan_minPts = DBscan_minPts_2;
        DB_MinClusterSize = DB_MinClusterSize_2;
        switch_jiwon_filter = false;
        switch_TC_filter = false;
    }
    else if(signal->State == 3) {
        //장애물
        DBscan_eps = DBscan_eps_3;
        DBscan_minPts = DBscan_minPts_3;
        DB_MinClusterSize = DB_MinClusterSize_3;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_3;
        switch_TC_filter = false;
    }
    else if(signal->State == 4) {
        //터널
        DBscan_eps = DBscan_eps_4;
        DBscan_minPts = DBscan_minPts_4;
        DB_MinClusterSize = DB_MinClusterSize_4;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_4;
        switch_TC_filter = false;
    }
    else if(signal->State == 8 || signal->State == 9 || signal->State == -1) {
        //협로, 유턴, 평행주차
        DBscan_eps = DBscan_eps_8;
        DBscan_minPts = DBscan_minPts_8;
        DB_MinClusterSize = DB_MinClusterSize_8;
        switch_jiwon_filter = true; //false
        switch_TC_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_8; 

    }
    else {
        DBscan_eps = DBscan_eps_re;
        DBscan_minPts = DBscan_minPts_re;
        DB_MinClusterSize = DB_MinClusterSize_re;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_re;
        switch_TC_filter = false;
    }
}

// void Clustering_process(const sensor_msgs::PointCloud2ConstPtr& aft_ransac){
//     RT::start();

//     PCXYZI::Ptr clustering_cloud (new PCXYZI);
//     pcl::fromROSMsg(*aft_ransac,*clustering_cloud);
//     PCXYZI Fin_Cloud;
//     Clustering(clustering_cloud, Fin_Cloud, switch_DBscan, switch_Euclid);

//     RT::end_cal("clustering");
//     FPS1.update();
// }

void Clustering_process_RANSAC_msg(const Lidar_MCduo_2023::RANSAC_points_arrConstPtr& msg) {
    RT::start();

    PCXYZI::Ptr clustering_cloud (new PCXYZI);
    pcl::PointXYZI tmp;
    for(int i = 0; i < msg->data.size(); i++) {
        tmp.x = msg->data[i].Rx;
        tmp.y = msg->data[i].Ry;
        tmp.z = msg->data[i].Rz;
        tmp.intensity = msg->data[i].Ri;
        clustering_cloud->push_back(tmp);
    }

    for(int i = 0; i < msg->coeff.size(); i++) {
        ransac_coeff.push_back(msg->coeff[i]);
    }

    PCXYZI Fin_Cloud;
    Clustering(clustering_cloud, Fin_Cloud, switch_DBscan, switch_Euclid);

    ransac_coeff.clear();
    if(msg->data.size() != 0) RT::end_cal("Clustering");
    else printf("\033[38;2;139;232;229mClustering runtime\033[0m : \033[1;31mNo input_data\033[0m\n");
    FPS1.update();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Clustering");    //node name 
	ros::NodeHandle nh;                     //nodehandle    

    //Euclid
    nh.getParam("/Clustering_node/switch_Euclid", switch_Euclid);
    nh.getParam("/Clustering_node/EC_eps", EC_eps);
    nh.getParam("/Clustering_node/EC_MinClusterSize", EC_MinClusterSize);
    nh.getParam("/Clustering_node/EC_MaxClusterSize", EC_MaxClusterSize);
    //DBSCAN
    nh.getParam("/Clustering_node/switch_DBscan", switch_DBscan);
    nh.getParam("/Clustering_node/DBscan_eps", DBscan_eps);
    nh.getParam("/Clustering_node/DBscan_minPts", DBscan_minPts);
    nh.getParam("/Clustering_node/DB_MinClusterSize", DB_MinClusterSize);
    nh.getParam("/Clustering_node/DB_MaxClusterSize", DB_MaxClusterSize);
    //etc
    nh.getParam("/Clustering_node/switch_jiwon_filter", switch_jiwon_filter);
    nh.getParam("/Clustering_node/switch_DY_filter", switch_DY_filter);
    nh.getParam("/Clustering_node/Ransac_Z_ROI", Ransac_Z_ROI);  //DY_filter param
    nh.getParam("/Clustering_node/REMOVE_FACTOR", REMOVE_FACTOR);

    nh.getParam("/Clustering_node/switch_TC_filter", switch_TC_filter);

    //sinal
    nh.getParam("/Clustering_node/REMOVE_FACTOR_1", REMOVE_FACTOR_1);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_3", REMOVE_FACTOR_3);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_4", REMOVE_FACTOR_4);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_8", REMOVE_FACTOR_8);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_re", REMOVE_FACTOR_re);

    nh.getParam("/Clustering_node/DBscan_eps_1", DBscan_eps_1);
    nh.getParam("/Clustering_node/DBscan_minPts_1", DBscan_minPts_1);
    nh.getParam("/Clustering_node/DB_MinClusterSize_1", DB_MinClusterSize_1);
    nh.getParam("/Clustering_node/DBscan_eps_2", DBscan_eps_2);
    nh.getParam("/Clustering_node/DBscan_minPts_2", DBscan_minPts_2);
    nh.getParam("/Clustering_node/DB_MinClusterSize_2", DB_MinClusterSize_2);
    nh.getParam("/Clustering_node/DBscan_eps_3", DBscan_eps_3);
    nh.getParam("/Clustering_node/DBscan_minPts_3", DBscan_minPts_3);
    nh.getParam("/Clustering_node/DB_MinClusterSize_3", DB_MinClusterSize_3);
    nh.getParam("/Clustering_node/DBscan_eps_4", DBscan_eps_4);
    nh.getParam("/Clustering_node/DBscan_minPts_4", DBscan_minPts_4);
    nh.getParam("/Clustering_node/DB_MinClusterSize_4", DB_MinClusterSize_4);
    nh.getParam("/Clustering_node/DBscan_eps_8", DBscan_eps_8);
    nh.getParam("/Clustering_node/DBscan_minPts_8", DBscan_minPts_8);
    nh.getParam("/Clustering_node/DB_MinClusterSize_8", DB_MinClusterSize_8);
    nh.getParam("/Clustering_node/DBscan_eps_re", DBscan_eps_re);
    nh.getParam("/Clustering_node/DBscan_minPts_re", DBscan_minPts_re);
    nh.getParam("/Clustering_node/DB_MinClusterSize_re", DB_MinClusterSize_re);

	//ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/3_Downsampling_PCL2", 1, Clustering_process);
    ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::RANSAC_points_arr> ("/3_1_Interior_points_RPA", 1, Clustering_process_RANSAC_msg);
    ros::Subscriber sub_totalInfo = nh.subscribe<Lidar_MCduo_2023::totalInfo> ("/Totalcom", 1, param_clustering);
    pub_Clu = nh.advertise<sensor_msgs::PointCloud2> ("/4_Clustering_PCL2", 1);

    pub_obj = nh.advertise<Lidar_MCduo_2023::object_msg_arr> ("/4_Clustering_OMA", 1);
    pub_LC = nh.advertise<Lidar_MCduo_2023::objsInfo> ("/LiDARcom", 1);
    //<패키지 명/메시지 파일 명>

    ros::spin();
}
