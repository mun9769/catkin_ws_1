#include <Lidar_MCduo_2023/Lidar_declare.h>

#define EquatorialRadius 6378137.0 //m
#define Flattening 1.0 / 298.257223563
#define EccentricitySquare 2 * Flattening - pow(Flattening, 2) // pow(0.081819804, 2)
#define LiDAR_GPS_distance 0.7 //m

float wgs84_a = 6378137.0;
float wgs84_f = 1.0 / 298.257223563;
float wgs84_e2 = 2 * wgs84_f - pow(wgs84_f, 2);

using Eigen::MatrixXf;
using Eigen::VectorXf;

ros::Time time_start;
ros::Time time_end;
float dt;

int id_cnt = 0;

// get_param
float P_0_0, P_1_1, P_2_2, P_3_3;
float Q_0_0, Q_1_1, Q_2_2, Q_3_3;
float R_0_0, R_1_1, R_2_2, R_3_3;
int max_erase_cnt;
float min_near_dist;
float min_new_dist;
float max_pred_dist, max_pred_vel;
//

// from file
double ref_lat, ref_lon, ref_alt; //ref_lat, ref_lon = radian
//

double degree_to_radian(double x) {
    return x * PI / 180;
}

struct Car {
    float x; // x = East
    float y; // y = North
    float heading;

    void LLA_to_ENU(double lat, double lon, double alt);
}car;

void Car::LLA_to_ENU(double lat, double lon, double alt) {
    // // LLA_to_ECEF (car)
    // lat = degree_to_radian(lat);
    // lon = degree_to_radian(lon);
    // double x, y, z;
    // double tan_radius = EquatorialRadius / sqrt(1 - EccentricitySquare * pow(sin(lat), 2));
    // x = (tan_radius + alt) * cos(lat) * cos(lon);
    // y = (tan_radius + alt) * cos(lat) * sin(lon);
    // z = (tan_radius * (1 - EccentricitySquare) + alt) * sin(lat);

    // // LLA_to_ECEF (reference)
    // double ref_x, ref_y, ref_z;
    // double ref_tan_radius = EquatorialRadius / sqrt(1 - EccentricitySquare * pow(sin(ref_lat), 2));
    // ref_x = (ref_tan_radius + ref_alt) * cos(ref_lat) * cos(ref_lon);
    // ref_y = (ref_tan_radius + ref_alt) * cos(ref_lat) * sin(ref_lon);
    // ref_z = (ref_tan_radius * (1 - EccentricitySquare) + ref_alt) * sin(ref_lat);

    // // car - reference
    // x = x - ref_x;
    // y = y - ref_y;
    // z = z - ref_z;

    // // ECEF_to_ENU (east = x, north = y)
    // car.x = (-sin(ref_lon)) * x + cos(ref_lon) * y;
    // car.y = (-sin(ref_lat) * cos(ref_lon)) * x + (-sin(ref_lat) * sin(ref_lon)) * y + cos(ref_lat) * z;
    // // car.z = cos(ref_lat) * cos(ref_lon) * x + cos(ref_lat) * sin(ref_lon) * y + sin(ref_lat) * z;

    float chi = sqrt(1 - wgs84_e2 * pow(sin(degree_to_radian(lat)), 2));
	float q = (wgs84_a / chi + alt) * cos(degree_to_radian(lat));
	float x = q * cos(degree_to_radian(lon));
	float y = q * sin(degree_to_radian(lon));
	float z = ((wgs84_a * (1 - wgs84_e2) / chi) + alt) * sin(degree_to_radian(lat));

	float ref_lat, ref_long, ref_alt;
    // Deliv A, Left, Parallel Parking, Small Avoid : 37.449159051666669 , 126.650967493333340 , 34.570000000000000
    // Deliv B : 37.44710714915139, 126.64982756190735, 34.570000000000000  
    // Big Avoid : 37.4475626473582 , 126.64939212506711 , 34.570000000000000
    // U-turn : 37.44780714915139, 126.64882756190735, 34.570000000000000
	ref_lat = degree_to_radian(37.2389005); // 37.4478071491513
	ref_long = degree_to_radian(126.7729385); // 126.64882756190735
	ref_alt = 29.809;
	float ref_chi = sqrt(1 - wgs84_e2 * pow(sin(ref_lat), 2));
	float ref_q = (wgs84_a / ref_chi + ref_alt) * cos(ref_lat);
	float ref_x = ref_q * cos(ref_long);
	float ref_y = ref_q * sin(ref_long);
	float ref_z = ((wgs84_a * (1 - wgs84_e2) / ref_chi) + ref_alt) * sin(ref_lat);
	float dx = x - ref_x;
	float dy = y - ref_y;
	float dz = z - ref_z;

	float east = -sin(ref_long) * dx + cos(ref_long) * dy;
	float north = -sin(ref_lat) * cos(ref_long) * dx - sin(ref_lat) * sin(ref_long) * dy + cos(ref_lat) * dz;
	float up = cos(ref_lat) * cos(ref_long) * dx + cos(ref_lat) * sin(ref_long) * dy + sin(ref_lat) * dz;

	car.x = east;
    car.y = north;

}

struct Tracking_object {
    int obj_id;

    float x;
    float y;
    float vx;
    float vy;

    float clu_x;
    float clu_y;
    float clu_x_prev;
    float clu_y_prev;

    int erase_cnt = 0;
    bool initlz_flag = false;
    bool update_flag = false;
    
    ros::Time update_time_start;
    ros::Time update_time_end;
    float update_dt;

    MatrixXf F = MatrixXf(4, 4);
    VectorXf X = VectorXf(4);
    VectorXf X_prev = VectorXf(4);
    MatrixXf P = MatrixXf(4, 4);
    VectorXf Z = VectorXf(4);
    MatrixXf K = MatrixXf(4, 4);

    MatrixXf H = Eigen::Matrix4f::Identity();
    MatrixXf R = MatrixXf(4, 4);
    MatrixXf Q = MatrixXf(4, 4);
};
vector<Tracking_object> trk_obj;

void Tracking_object_information_update(vector<Tracking_object>::iterator it) {
    it->x = it->X(0);
    it->y = it->X(1);
    it->vx = it->X(2);
    it->vy = it->X(3);
}

struct Clustering_object {
    float x;
    float y;

    bool new_obj_flag = true;
};
vector<Clustering_object> clu_obj;
vector<Clustering_object> clu_obj_prev;
ros::Publisher pub_map;

void Trans_data_set(const Lidar_MCduo_2023::object_msg_arrConstPtr& objects) {
    Clustering_object tmp;
    for(Lidar_MCduo_2023::object_msg obj : objects->object_msg_arr) {
        tmp.x = obj.x + LiDAR_GPS_distance;
        tmp.y = obj.y;
        clu_obj.push_back(tmp);
    }

    vector<Clustering_object>::iterator it;
    for(it = clu_obj.begin(); it != clu_obj.end(); it++) {
        float x = cos(degree_to_radian(90 - car.heading)) * it->x - sin(degree_to_radian(90 - car.heading)) * it->y;
        float y = sin(degree_to_radian(90 - car.heading)) * it->x + cos(degree_to_radian(90 - car.heading)) * it->y;

        it->x = x + car.x;
        it->y = y + car.y;
    }

    Lidar_MCduo_2023::object_msg_arr tmp_arr;
    Lidar_MCduo_2023::object_msg tmp_om;
    for(int i = 0; i < clu_obj.size(); i++) {
        tmp_om.x = clu_obj[i].x;
        tmp_om.y = clu_obj[i].y;
        tmp_arr.object_msg_arr.push_back(tmp_om);
    }
    pub_map.publish(tmp_arr);
}

void Compare_distance() {
    // near_obj
    vector<Tracking_object>::iterator it_trk;
    vector<Clustering_object>::iterator it_clu;

    for(it_trk = trk_obj.begin(); it_trk != trk_obj.end(); it_trk++) {
        float dist;
        float dist_min = FLT_MAX;
        vector<Clustering_object>::iterator it_best_obj;

        for(it_clu = clu_obj.begin(); it_clu != clu_obj.end(); it_clu++) {
            if(it_clu->new_obj_flag == true) {
                dist = cal_dist_21(it_trk->x, it_clu->x, it_trk->y, it_clu->y);

                if(dist <= min_near_dist && dist <= dist_min) {
                    dist_min = dist;
                    it_best_obj = it_clu;
                    it_trk->update_flag = true;
                }
            }
        }

        if(it_trk->update_flag == true) {
            it_trk->clu_x = it_best_obj->x;
            it_trk->clu_y = it_best_obj->y;
            it_best_obj->new_obj_flag = false;
        }
        else {
            it_trk->erase_cnt++;
            // it_trk->update_flag = false;
            if(it_trk->erase_cnt > max_erase_cnt) {
                trk_obj.erase(it_trk);
                it_trk--;
            }
        }
    }

    // new_obj
    vector<Clustering_object>::iterator it_clu_prev;
    for(it_clu = clu_obj.begin(); it_clu != clu_obj.end(); it_clu++) {
        if(it_clu->new_obj_flag == true) {
            float dist;
            float dist_min = FLT_MAX;
            bool save_clu_obj = false;
            vector<Clustering_object>::iterator it_new_obj;
            vector<Clustering_object>::iterator it_new_obj_prev;
            

            for(it_clu_prev = clu_obj_prev.begin(); it_clu_prev != clu_obj_prev.end(); it_clu_prev++) {
                if(it_clu_prev->new_obj_flag == true) {
                    dist = cal_dist_21(it_clu->x, it_clu_prev->x, it_clu->y, it_clu_prev->y);

                    if(dist < min_new_dist && dist < dist_min) {
                        dist_min = dist;
                        it_new_obj = it_clu;
                        it_new_obj_prev = it_clu_prev;
                        save_clu_obj = true;
                        it_clu->new_obj_flag = false;
                    }
                }
            }

            if(save_clu_obj == true) {
                Tracking_object tmp;
                tmp.x = it_new_obj->x;
                tmp.y = it_new_obj->y;
                tmp.vx = (it_new_obj->x - it_new_obj_prev->x) / dt;
                tmp.vy = (it_new_obj->y - it_new_obj_prev->y) / dt;
                tmp.clu_x = it_new_obj->x;
                tmp.clu_y = it_new_obj->y;
                tmp.clu_x_prev = it_new_obj_prev->x;
                tmp.clu_y_prev = it_new_obj_prev->y;
                tmp.initlz_flag = true;
                trk_obj.push_back(tmp);
            }
        }
    }
    clu_obj_prev = clu_obj;
}

void Initialization_tracking_object() {
    vector<Tracking_object>::iterator it;
    for(it = trk_obj.begin(); it != trk_obj.end(); it++) {
        if(it->initlz_flag == true) {
            it->obj_id = id_cnt;
            id_cnt++;

            it->F.setZero();
            it->X.setZero();
            it->X_prev.setZero();
            it->P.setZero();
            it->Z.setZero();
            it->K.setZero();
            it->R.setZero();
            it->Q.setZero();

            it->X(0) = it->x;
            it->X(1) = it->y;
            it->X(2) = it->vx;
            it->X(3) = it->vy;

            it->P(0, 0) = P_0_0;
            it->P(1, 1) = P_1_1;
            it->P(2, 2) = P_2_2;
            it->P(3, 3) = P_3_3;

            it->Q(0, 0) = Q_0_0;
            it->Q(1, 1) = Q_1_1;
            it->Q(2, 2) = Q_2_2;
            it->Q(3, 3) = Q_3_3;

            it->R(0, 0) = R_0_0;
            it->R(1, 1) = R_1_1;
            it->R(2, 2) = R_2_2;
            it->R(3, 3) = R_3_3;
        }
    }
}

void State_prediction() {
    vector<Tracking_object>::iterator it;
    for(it = trk_obj.begin(); it != trk_obj.end(); it++) {
        if(it->initlz_flag == false) {
            it->F << 1, 0, dt, 0,
                     0, 1, 0, dt,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
            
            it->X_prev = it->X;

            it->X = it->F * it->X;
            it->P = it->F * it->P * it->F.transpose() + it->Q;

            Tracking_object_information_update(it);

            float dist = cal_dist_21(it->X(0), it->X_prev(0), it->X(1), it->X_prev(1));
            float velocity = cal_dist_21(it->X(2), it->X_prev(2), it->X(3), it->X_prev(3));

            if(dist > max_pred_dist) {
                trk_obj.erase(it);
                it--;
            }
            else if(velocity * 3.6 > max_pred_vel) {
                trk_obj.erase(it);
                it--;
            }
        }
    }
}

void Measurement_update() {
    vector<Tracking_object>::iterator it;
    for(it = trk_obj.begin(); it != trk_obj.end(); it++) {
        if(it->update_flag == true && it->initlz_flag == false) {
            it->update_time_end = ros::Time::now();
            it->update_dt = (it->update_time_end - it->update_time_start).toSec();
            it->update_time_start = ros::Time::now();

            it->Z(0) = it->clu_x;
            it->Z(1) = it->clu_y;
            it->Z(2) = (it->clu_x - it->clu_x_prev) / it->update_dt;
            it->Z(3) = (it->clu_y - it->clu_y_prev) / it->update_dt;

            it->K = it->P * it->H.transpose() * (it->H * it->P * it->H.transpose() + it->R).inverse();
        
            it->P = it->P - it->K * it->H * it->P;

            it->X = it->X + it->K * (it->Z - it->H * it->X);

            it->clu_x_prev = it->clu_x;
            it->clu_y_prev = it->clu_y;

            Tracking_object_information_update(it);
        }
        else {
            it->initlz_flag = false; // for_new_object
            it->update_time_start = ros::Time::now();
        }

        it->update_flag = false;
    }
}

void Tracking_process(const Lidar_MCduo_2023::object_msg_arrConstPtr& objects) {
    time_end = ros::Time::now();
    dt = (time_end - time_start).toSec();
    time_start = ros::Time::now();

    Trans_data_set(objects);

    Compare_distance();

    Initialization_tracking_object();

    State_prediction();

    Measurement_update();

    PCXYZI point_tmp;
    for(int i = 0; i < trk_obj.size(); i++) {
        pcl::PointXYZI tmp_pcl;
        tmp_pcl.x = trk_obj[i].X(0);
        tmp_pcl.y = trk_obj[i].X(1);
        point_tmp.push_back(tmp_pcl);
    }
    sensor_msgs::PointCloud2 output_pcl;
    pub_process(point_tmp, output_pcl);
    pub_tracking_pcl.publish(output_pcl);

    // + visualization
    Lidar_MCduo_2023::obj_tracking_msg output;
    Lidar_MCduo_2023::obj_tracking tmp;
    for(int i = 0; i < trk_obj.size(); i++) {
        tmp.trk_idx = trk_obj[i].obj_id;
        tmp.trk_x = trk_obj[i].x;
        tmp.trk_y = trk_obj[i].y;
        tmp.trk_vx = trk_obj[i].vx;
        tmp.trk_vy = trk_obj[i].vy;
        output.data.push_back(tmp);
    }
    output.obj_num = trk_obj.size();
    output.car_x = car.x;
    output.car_y = car.y;
    pub_tracking_msg.publish(output);

    clu_obj.clear();
}

void GNSS_process(const Lidar_MCduo_2023::GNSSInfoConstPtr& GNSS_msg) {
    car.LLA_to_ENU(GNSS_msg->latitude, GNSS_msg->longitude, GNSS_msg->altitude);
    car.heading = GNSS_msg->heading;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_tracking_global_node");
    ros::NodeHandle nh;

    time_start = ros::Time::now();

    nh.getParam("/tracking_global_node/P_0_0", P_0_0);
    nh.getParam("/tracking_global_node/P_1_1", P_1_1);
    nh.getParam("/tracking_global_node/P_2_2", P_2_2);
    nh.getParam("/tracking_global_node/P_3_3", P_3_3);
    nh.getParam("/tracking_global_node/Q_0_0", Q_0_0);
    nh.getParam("/tracking_global_node/Q_1_1", Q_1_1);
    nh.getParam("/tracking_global_node/Q_2_2", Q_2_2);
    nh.getParam("/tracking_global_node/Q_3_3", Q_3_3);
    nh.getParam("/tracking_global_node/R_0_0", R_0_0);
    nh.getParam("/tracking_global_node/R_1_1", R_1_1);
    nh.getParam("/tracking_global_node/R_2_2", R_2_2);
    nh.getParam("/tracking_global_node/R_3_3", R_3_3);

    nh.getParam("/tracking_global_node/max_erase_cnt", max_erase_cnt);
    nh.getParam("/tracking_global_node/min_near_dist", min_near_dist);
    nh.getParam("/tracking_global_node/min_new_dist", min_new_dist);
    nh.getParam("/tracking_global_node/max_pred_dist", max_pred_dist);
    nh.getParam("/tracking_global_node/max_pred_vel", max_pred_vel);

    string where_are_you;
    ifstream ifs;
    nh.getParam("/tracking_global_node/where_are_you", where_are_you);

    if(where_are_you == "K_city") {
        ref_lat = degree_to_radian(37.449895758333334);
        ref_lon = degree_to_radian(126.651063116666663);
        ref_alt = 34.789000000000001;
    }
    else if(where_are_you == "SNU_Siheung") {
        ref_lat = degree_to_radian(37.365585531666667);
        ref_lon = degree_to_radian(126.725147028333339);
        ref_alt = 33.353000000000002;
    }

    // if(where_are_you == "K_city") {
    //     ref_lat = degree_to_radian(37.2389005);
    //     ref_lon = degree_to_radian(126.7729385);
    //     ref_alt = 29.809;
    // }
    // else if(where_are_you == "SNU_Siheung") {
    //     ref_lat = degree_to_radian(37.365585531666667);
    //     ref_lon = degree_to_radian(126.725147028333339);
    //     ref_alt = 33.353000000000002;
    // }

    ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::object_msg_arr> ("/4_Clustering_OMA", 1, Tracking_process);
    ros::Subscriber sub_GNSS = nh.subscribe<Lidar_MCduo_2023::GNSSInfo> ("/GNSScom", 1, GNSS_process);
    pub_tracking_pcl = nh.advertise<sensor_msgs::PointCloud2> ("/5_Tracking_global_PCL2", 1);
    pub_tracking_msg = nh.advertise<Lidar_MCduo_2023::obj_tracking_msg> ("/LiDAR_tracking_global", 1);
    pub_map = nh.advertise<Lidar_MCduo_2023::object_msg_arr> ("/LiDAR_tracking_global_map", 1);

    ros::spin();
}