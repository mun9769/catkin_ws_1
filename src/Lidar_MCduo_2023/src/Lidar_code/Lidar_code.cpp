#include <Lidar_MCduo_2023/Lidar_declare.h>

void ROI(const sensor_msgs::PointCloud2ConstPtr& scan, int state){
    RT::start();
    PCXYZI rawData;
    PCXYZI cropedData;
    pcl::fromROSMsg(*scan, rawData);

    if(switch_ROI) {
        makeCropBox(rawData, ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax);

        if(state == 8 || state == 4) {
            PCXYZI::Ptr rawData_Ptr (new PCXYZI ());
            *rawData_Ptr = rawData;

            PCXYZI::Ptr except_points (new PCXYZI ());
            PCXYZI::Ptr non_except_points (new PCXYZI ());
            PCXYZI::Ptr x_except_points (new PCXYZI ());
            PCXYZI::Ptr x_non_except_points (new PCXYZI ());
            PCXYZI::Ptr xy_non_except_points_neg (new PCXYZI ());
            PCXYZI::Ptr xy_non_except_points_pos (new PCXYZI ());

            pcl::PassThrough<PXYZI> x_filter;
            pcl::PassThrough<PXYZI> y_filter;

            x_filter.setInputCloud (rawData_Ptr);
            x_filter.setFilterFieldName ("x");
            x_filter.setFilterLimits (-0.5, 0.5);
            x_filter.filter (*x_except_points);
            x_filter.setFilterLimits (0.5, 20);
            x_filter.filter (*x_non_except_points);

            y_filter.setInputCloud (x_except_points);
            y_filter.setFilterFieldName ("y");
            y_filter.setFilterLimits (-1, 1);
            y_filter.filter (*except_points);
            y_filter.setFilterLimits(-20, -1);
            y_filter.filter (*xy_non_except_points_neg);
            y_filter.setFilterLimits(1, 20);
            y_filter.filter(*xy_non_except_points_pos);

            *non_except_points = (*x_non_except_points) + (*xy_non_except_points_neg) + (*xy_non_except_points_pos);
            rawData = *non_except_points;
        }
    }

    sensor_msgs::PointCloud2 output;                        //to output ROIdata formed PC2
    pub_process(rawData,output);
    pub_ROI.publish(output);
    RT::end_cal("ROI");
}

void makeCropBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    pcl::CropBox<PXYZI> boxfilter;
    boxfilter.setMin(Eigen::Vector4f(xMin, yMin, zMin, std::numeric_limits<float>::lowest()));
    boxfilter.setMax(Eigen::Vector4f(xMax, yMax, zMax, std::numeric_limits<float>::max()));
    boxfilter.setInputCloud(Cloud.makeShared());
    boxfilter.filter(Cloud);
}

// void DownSampling(PCXYZI& rawData, PCXYZI::Ptr downsampledCloud, PCXYZI::Ptr combined_points){ //Voxelization = DownSampling 
//     PCXYZI Data_for_voxel;
//     pcl::VoxelGrid<PXYZI> vg;                            //declare voxel

//     copyPointCloud(rawData, Data_for_voxel);            //rawData  ->  Data_for_voxel  ... just copy for modify
//     vg.setInputCloud (Data_for_voxel.makeShared());     //Data_for_voxel  ->  vg space  ... deep copy , makeShared() return PC
//     vg.setLeafSize (voxel_size_x, voxel_size_y, voxel_size_z);		        //voxel size setting(x,y,z)
//     vg.filter (*downsampledCloud);                      //voxelized datas are included in downsampledCloud
//     //cout << "PointCloud after downsampling has: " << downsampledCloud->points.size ()  << " data points." << endl; 

//     *combined_points = *downsampledCloud + objs_points;

//     sensor_msgs::PointCloud2 output;
//     pub_process(*combined_points, output);             
//     pub_DS.publish(output);
    
// }

void Clustering (PCXYZI::Ptr inputCloud, PCXYZI& retCloud, bool switch_DBscan, bool switch_Euclid){
    if( switch_DBscan ) DBScanClustering( inputCloud, retCloud); //prior DBSCAN
    else if( switch_Euclid ) EuclideanClustering( inputCloud, retCloud );
    else retCloud = *inputCloud; //doesn't process clustering

    sensor_msgs::PointCloud2 output; 
    pub_process(retCloud,output); 
    pub_Clu.publish(output); 
}

void afterClusteringProcess(PCXYZI::Ptr inputCloud, PCXYZI& retCloud, vector<pcl::PointIndices>& cluster_indices){
    vector<pair<PXYZI,string>> sorted_OBJ; //여기에 minmax가 포함되지 않아서 발생한 문제이므로 이를 포함하는 struct를 만들자
    vector<struct objectInfo> objs;

    int intensityValue = 0;
    for (vector<pcl::PointIndices>::iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, intensityValue++){

        pair<float,float> x(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()); //first = min, second = max
        pair<float,float> y(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
        pair<float,float> z(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());

    	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            PXYZI pt = inputCloud->points[*pit];
            pt.intensity = intensityValue % 10;
            //retCloud.push_back(pt); //change >> not make retcloud here  >> make at filter
            if(pt.x < x.first)      x.first = pt.x;
            if(pt.x > x.second)     x.second = pt.x;
            if(pt.y < y.first)      y.first = pt.y;
            if(pt.y > y.second)     y.second = pt.y;
            if(pt.z < z.first)      z.first = pt.z;
            if(pt.z > z.second)     z.second = pt.z;
    	}

        PXYZI* tmp = new PXYZI();
        tmp->x = MidPt(x.first,x.second); tmp->y = MidPt(y.first,y.second); tmp->z = z.second; //z = max
        pair<PXYZI,string> temp = make_pair(*tmp,send_msg_minmax(x.first, x.second, y.first, y.second));
        sorted_OBJ.push_back(temp);

        objectInfo tmp_obj = {&(*it), "unknown", (unsigned int)intensityValue, 
                            MidPt(x.first,x.second), MidPt(y.first,y.second), MidPt(z.first,z.second),
                            x.first, y.first, z.first, x.second, y.second, z.second,
                            (short)(intensityValue % 10)};
        objs.push_back(tmp_obj);
    }
    //cout << "------------------ DF & JF ------------------" << endl;
    FT.DY_filter(sorted_OBJ, switch_DY_filter);
    FT.DY_filter(objs, switch_DY_filter); //indices vector를 수정하는 filter
    FT.jiwon_filter(sorted_OBJ, switch_jiwon_filter);
    FT.jiwon_filter(objs, switch_jiwon_filter); //indices vector를 수정하는 filter
    FT.TC_filter(objs, switch_TC_filter);
    FT.generate_return_PointCloud(inputCloud, retCloud, objs);
    
    object_msg_process(objs);

}

void EuclideanClustering(PCXYZI::Ptr inputCloud, PCXYZI& retCloud){
    pcl::search::KdTree<PXYZI>::Ptr tree (new pcl::search::KdTree<PXYZI>);  // Creating the KdTree for searching PC
    tree->setInputCloud(inputCloud);                     // setting the KdTree

    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    pcl::EuclideanClusterExtraction<PXYZI> ec;           // clustering with Euclidean method
    ec.setInputCloud(inputCloud);   	                 // setting ec with inputCloud
    ec.setClusterTolerance(EC_eps); 	                 // dist between points ..  cur : 30cm
    ec.setMinClusterSize(EC_MinClusterSize);		     // minSize the number of point for clustering
    ec.setMaxClusterSize(EC_MaxClusterSize);	         // minSize the number of point for clustering
    ec.setSearchMethod(tree);				             // searching method : tree 
    ec.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    //cout << "Number of clusters is equal to " << cluster_indices.size() << endl;    //return num of clusteringObj
    afterClusteringProcess(inputCloud, retCloud, cluster_indices);
}

void DBScanClustering(PCXYZI::Ptr inputCloud, PCXYZI& retCloud){
    pcl::search::KdTree<PXYZI>::Ptr tree (new pcl::search::KdTree<PXYZI>);  // Creating the KdTree for searching PC
    if(inputCloud->points.size() != 0) tree->setInputCloud(inputCloud);                     // setting the KdTree
    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    
    DBSCAN<PXYZI> db;
    db.setCorePointMinPts(DBscan_minPts);                // minimum points of cluster judge
    db.setClusterTolerance(DBscan_eps);                  // dist between points
    db.setMinClusterSize(DB_MinClusterSize);		     // minSize the number of point for clustering
    db.setMaxClusterSize(DB_MaxClusterSize);	         // maxSize the number of point for clustering
    db.setSearchMethod(tree);				             // searching method : tree
    db.setInputCloud(inputCloud);   	                 // setting ec with inputCloud
    db.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    //cout << "Number of clusters is equal to " << cluster_indices.size() << endl;    //return num of clusteringObj
    afterClusteringProcess(inputCloud, retCloud, cluster_indices);
}

void RanSaC(PCXYZI::Ptr inputCloud){ //for walls
    RT::start();

    pcl::ModelCoefficients coefficients0;
    pcl::ModelCoefficients coefficients1;
    pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices ());
    PCXYZI::Ptr inlier_points1 (new PCXYZI ());
    PCXYZI::Ptr inlier_points2 (new PCXYZI ());
    PCXYZI::Ptr outlier_points1 (new PCXYZI ());
    PCXYZI::Ptr outlier_points2 (new PCXYZI ());
    PCXYZI::Ptr combined_inliers(new PCXYZI ());

    pcl::SACSegmentation<PXYZI> seg;
    pcl::ExtractIndices<PXYZI> extract;
/////////////////////////////////////////////////////////////////////////////
    PCXYZI::Ptr except_points (new PCXYZI ());
    PCXYZI::Ptr non_except_points (new PCXYZI ());
    PCXYZI::Ptr x_except_points (new PCXYZI ());
    PCXYZI::Ptr x_non_except_points (new PCXYZI ());
    PCXYZI::Ptr xy_non_except_points_neg (new PCXYZI ());
    PCXYZI::Ptr xy_non_except_points_pos (new PCXYZI ());

    pcl::PassThrough<PXYZI> x_filter;
    pcl::PassThrough<PXYZI> y_filter;
    x_filter.setInputCloud (inputCloud);
    x_filter.setFilterFieldName ("x");
    x_filter.setFilterLimits (-0.5, 2);
    x_filter.filter (*x_except_points);
    x_filter.setFilterLimits (2, 20);
    x_filter.filter (*x_non_except_points);

    y_filter.setInputCloud (x_except_points);
    y_filter.setFilterFieldName ("y");
    y_filter.setFilterLimits (-1.5, 1.5);
    y_filter.filter (*except_points);
    y_filter.setFilterLimits(-20, -1.5);
    y_filter.filter (*xy_non_except_points_neg);
    y_filter.setFilterLimits(1.5, 20);
    y_filter.filter(*xy_non_except_points_pos);

    *non_except_points = (*x_non_except_points) + (*xy_non_except_points_neg) + (*xy_non_except_points_pos);

    Lidar_MCduo_2023::RANSAC_points_arr RANSAC_msg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE); 
    seg.setMethodType (pcl::SAC_RANSAC); 
    seg.setDistanceThreshold (ransac_distanceThreshold);
    seg.setMaxIterations (1000);

    if(non_except_points->points.size() != 0) {
        seg.setInputCloud(non_except_points); 
        seg.segment (*inliers1, coefficients0);   

        pcl::copyPointCloud<PXYZI>(*non_except_points, *inliers1, *inlier_points1);
        extract.setInputCloud (non_except_points);
        extract.setIndices (inliers1);
        extract.setNegative (true);     //false
        extract.filter (*outlier_points1); 

//////////////////////////////////////////////////////////////////////////////////

        if(outlier_points1->points.size() != 0) {
            seg.setInputCloud(outlier_points1); 
            seg.segment (*inliers2, coefficients1);
        }
        else {
            coefficients1.values.resize(6);
            coefficients1.values[3] = coefficients0.values[4];
            coefficients1.values[4] = -(coefficients0.values[3]);
        }

        Lidar_MCduo_2023::coeff coeff0;
        Lidar_MCduo_2023::coeff coeff1;
        
        float radian0 = atan2(coefficients0.values[4], coefficients0.values[3]);
        float radian1 = atan2(coefficients1.values[4], coefficients1.values[3]);
        float slope0 = coefficients0.values[4] / coefficients0.values[3];
        float slope1 = coefficients1.values[4] / coefficients1.values[3];
        float y_intercept0 = -(slope0 * coefficients0.values[0]) + coefficients0.values[1];
        float y_intercept1 = -(slope1 * coefficients1.values[0]) + coefficients1.values[1];

        if(abs(radian0 - radian1) > 0.035) { // default 11
            slope1 = slope0;
            if(y_intercept0 > 0) { // left
                if(slope0 != 0) y_intercept1 = -11.1/cos(atan(abs(slope0))) + y_intercept0;
                else y_intercept1 = y_intercept0 - 11.1; //11.2
            }
            else { // right
                if(slope0 != 0)  y_intercept1 = 11.1/cos(atan(abs(slope0))) + y_intercept0;
                else y_intercept1 = y_intercept0 + 11.1; //11.2
            }
            *outlier_points2 = *outlier_points1 + *except_points;
            *combined_inliers = *inlier_points1;
        }
        else {
            pcl::copyPointCloud<PXYZI>(*outlier_points1, *inliers2, *inlier_points2);
            extract.setInputCloud (outlier_points1);
            extract.setIndices (inliers2);
            extract.setNegative (true);     //false
            extract.filter (*outlier_points2);
            *outlier_points2 += *except_points;
            *combined_inliers = *inlier_points1 + *inlier_points2;
        }
        coeff0.Slope = slope0;
        coeff1.Slope = slope1;
        coeff0.Yintercept = y_intercept0;
        coeff1.Yintercept = y_intercept1;

        RANSAC_msg.coeff.push_back(coeff0);
        RANSAC_msg.coeff.push_back(coeff1);
    }

    for(int i = 0; i < outlier_points2->points.size(); i++) {
        Lidar_MCduo_2023::RANSAC_points tmp;
        outlier_points2->points[i].z = outlier_points2->points[i].intensity;
        tmp.Rx = outlier_points2->points[i].x;
        tmp.Ry = outlier_points2->points[i].y;
        tmp.Rz = outlier_points2->points[i].z;
        tmp.Ri = outlier_points2->points[i].intensity;
        RANSAC_msg.data.push_back(tmp);
    }
    pub_RS_RPA.publish(RANSAC_msg);

    sensor_msgs::PointCloud2 output;
    pub_process(*outlier_points2, output); 
    pub_RS.publish(output);
    pub_process(*combined_inliers, output);
    pub_walls.publish(output);
    
    // y = [4]/[3] * (x - [0]) + [1]
    if(inputCloud->points.size() != 0) {
        RT::end_cal("RANSAC");
    }
    else {
        printf("\033[38;2;139;232;229mRANSAC runtime\033[0m : \033[1;31mNo input_data\033[0m\n");
    }
}

void Parsing_lidar_data(const char* buffer) {
    if(switch_UDP_communication) {
        for(int i = 0; i < 1200; i += 100) {
            //azimuth
            int j = i + 2;
            uint8_t byte1 = buffer[j];
            uint8_t byte2 = buffer[j + 1];
            uint16_t fa_byte = 0;
            fa_byte = fa_byte | byte2;
            fa_byte = fa_byte << 8;
            fa_byte = fa_byte | byte1;

            double azimuth = fa_byte * 0.01;

            //distance, reflectivity, laserid
            int laserid = 0;
            for (int k = 4 + i; k < 100 + i; k += 3) {
                uint8_t byte1 = buffer[k];
                uint8_t byte2 = buffer[k + 1];
                uint8_t byte3 = buffer[k + 2];
                uint16_t fa_byte = 0;
                fa_byte = fa_byte | byte2;
                fa_byte = fa_byte << 8;
                fa_byte = fa_byte | byte1;
                // azimuth = (float)(uint8_t(buffer[k+1] << 8) | uint8_t(buffer[k])); 

                double distance = fa_byte * 0.002;

                arr.push_back(cal_point(distance, azimuth, byte3, laserid));

                laserid++;
            }
        }
    }
    ground_extraction();
    arr.clear();
    arr2.clear();
}

void ground_extraction() {
    for(int i = 0; i < arr.size(); i++) {
        pcl::PointXYZI tmp;
        tmp.x = arr[i].x;
        tmp.y = arr[i].y;
        tmp.z = arr[i].z;
        // tmp.intensity = arr[i].reflec;
        rawdata.push_back(tmp);
        arr2.push_back(arr[i]);
    }
    
    for(int i = 0; i < arr.size(); i++) {
        pcl::PointXYZI tmp1;
        Lidar_MCduo_2023::ground_msg tmpg;
        if(i%32 == 14 || i%32 == 15) continue;
        double d1 = cal_dist_21(arr[i+2].x, arr[i].x, arr[i+2].y, arr[i].y);
        double d2 = abs(arr[i+2].z - arr[i].z);
        double slope = atan2(d2, d1);

        if(slope < GE_slope && slope > 0) {
            if(arr[i].z < GE_Z) {
                tmp1.x = arr[i].x;
                tmp1.y = arr[i].y;
                tmp1.z = arr[i].z;
                ground.push_back(tmp1);
                tmpg.Gx = arr[i].x;
                tmpg.Gy = arr[i].y;
                tmpg.Gz = arr[i].z;
                tmpg.LID = arr[i].laserID;
                ground_cam.ground_msg_arr.push_back(tmpg);
                arr2[i].x = 0;
                arr2[i].y = 0; 
                arr2[i].z = 0; 
            }
            if(arr[i+2].z < GE_Z) {
                tmp1.x = arr[i+2].x;
                tmp1.y = arr[i+2].y;
                tmp1.z = arr[i+2].z;
                ground.push_back(tmp1);
                tmpg.Gx = arr[i+2].x;
                tmpg.Gy = arr[i+2].y;
                tmpg.Gz = arr[i+2].z;
                tmpg.LID = arr[i+2].laserID;
                ground_cam.ground_msg_arr.push_back(tmpg);
                arr2[i+2].x = 0;
                arr2[i+2].y = 0;
                arr2[i+2].z = 0;
            }
        }
        else if((arr[i].laserID == 0 || arr[i].laserID == 16) && d1 < 0.5 && d1 > 0.1) {
            tmp1.x = arr[i].x;
            tmp1.y = arr[i].y;
            tmp1.z = arr[i].z;
            ground.push_back(tmp1);
            arr2[i].x = 0;
            arr2[i].y = 0; 
            arr2[i].z = 0; 
        }
        if((arr2[i].x != 0) && (arr2[i].y != 0) && (arr2[i].z != 0)) {
            tmp1.x = arr2[i].x;
            tmp1.y = arr2[i].y;
            tmp1.z = arr2[i].z;  // tmp1.z = (lidar_state == 4) ? 0 : arr2[i].z;
            // tmp1.intensity = arr2[i].reflec;
            others.push_back(tmp1);
        }
    }

    k++;
    if(k == 50) {
        printf("\033[38;2;139;232;229mState\033[0m : \033[1;37m%d\033[0m\n", lidar_state);
        if(switch_UDP_communication) RT::end_cal("UDP_communication");
        else printf("\033[38;2;139;232;229mUPD_communication runtime\033[0m : \033[0mOFF\033[0m\n");
        RT::start();

        sensor_msgs::PointCloud2 output;
        pub_process(rawdata, output);
        pub_rawdata.publish(output);
        pub_process(others, output);
        pub_others.publish(output);
        pub_process(ground, output);
        pub_ground.publish(output);
        ground_cam.size = ground_cam.ground_msg_arr.size();
        pub_ground_to_cam.publish(ground_cam);
        k = 0;
        rawdata.clear();
        ground.clear();
        others.clear();
        ground_cam.ground_msg_arr.clear();
        ground_cam.size = 0;
    }
}