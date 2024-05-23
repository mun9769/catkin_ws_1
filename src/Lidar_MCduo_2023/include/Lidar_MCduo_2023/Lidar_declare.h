#pragma once

#ifndef LIDAR_DECLARE
#define LIDAR_DECLARE
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <limits>
#include <algorithm>
#include <string>
#include <time.h>
#include <signal.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>   //noise filtering
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Lidar_MCduo_2023/SDW_DBSCAN.h>
//#include <Lidar_process_div/DBSCAN_hanbin.h>
#include <Lidar_MCduo_2023/object_msg_arr.h>
#include <Lidar_MCduo_2023/objsInfo.h>
#include <Lidar_MCduo_2023/totalInfo.h>
#include <Lidar_MCduo_2023/GNSSInfo.h>
#include <Lidar_MCduo_2023/obj_tracking_msg.h>
#include <Lidar_MCduo_2023/ground_msg_arr.h>
#include <Lidar_MCduo_2023/RANSAC_points_arr.h>
//---------------------------------------------------
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//---------------------------------------------------
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace std;
using namespace std::chrono;
using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PCXYZI;
typedef pcl::PointXYZ PXYZ;
typedef pcl::PointXYZI PXYZI;

ros::Publisher pub_ROI;     //ROI
ros::Publisher pub_RS;      //ransac
ros::Publisher pub_walls;
ros::Publisher pub_Clu;
ros::Publisher pub_obj;     //user defined msg
ros::Publisher pub_LC;     //user defined msg
ros::Publisher pub_TS;
ros::Publisher pub_DS;
ros::Publisher pub_RS_RPA;

ros::Publisher pub_rawdata;
ros::Publisher pub_ground;
ros::Publisher pub_ground_to_cam;
ros::Publisher pub_others;
ros::Publisher pub_tracking_pcl;
ros::Publisher pub_tracking_msg;

//ros::Publisher OUT_MSG;     //out message
int lidar_state;
double REMOVE_FACTOR, REMOVE_FACTOR_re, REMOVE_FACTOR_1, REMOVE_FACTOR_3, REMOVE_FACTOR_4, REMOVE_FACTOR_8;
float voxel_size_x, voxel_size_y, voxel_size_z;
float ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax, ROI_yMin_2, ROI_yMax_2, ROI_zMin_2, ROI_xMax_4, ROI_yMin_4, ROI_yMax_4,ROI_xMin_8, ROI_xMax_8, ROI_yMin_8, ROI_yMax_8, ROI_zMax_8, ROI_zMin_8, ROI_xMin_re, ROI_xMax_re, ROI_yMin_re, ROI_yMax_re, ROI_zMin_re, ROI_zMax_re;
float GE_Z, GE_Z_4, GE_Z_re, GE_slope;
float Ransac_Z_ROI;
float DBscan_eps, DBscan_eps_re, DBscan_eps_1, DBscan_eps_2, DBscan_eps_3, DBscan_eps_4, DBscan_eps_8;
float DBscan_minPts, DBscan_minPts_re, DBscan_minPts_1, DBscan_minPts_2, DBscan_minPts_3, DBscan_minPts_4, DBscan_minPts_8;
int DB_MinClusterSize, DB_MinClusterSize_re, DB_MinClusterSize_1, DB_MinClusterSize_2, DB_MinClusterSize_3, DB_MinClusterSize_4, DB_MinClusterSize_8;
int DB_MaxClusterSize;
float EC_eps;
int EC_MinClusterSize, EC_MaxClusterSize;
double ransac_distanceThreshold; //함수 : double형 parameter
bool switch_jiwon_filter;
bool switch_DY_filter;
bool switch_ROI;
bool switch_DownSampling;
bool switch_Euclid;
bool switch_RanSaC;
bool switch_DBscan;
bool switch_visual;
bool switch_visual_2D;
bool switch_TC_filter;
bool switch_interior_points;
bool switch_UDP_communication;
//-----------------------------------
int cnt = 0;
double yaw_start = 0;
double yaw_end = 0;
double dyaw;
double yaw_now;

bool shouldExit = false;
int Port_Num;
string UDP_IP;
PCXYZI rawdata;
PCXYZI others;
PCXYZI ground;
int k = 0;
int m = 0;
int p = 0;
static const double PI = acos(-1.0);
const int LASER_ANGLES[] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15,
                            -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

struct Point {
    double x, y, z, dist, azimuth;
    uint8_t reflec;
    int laserID;
    Point() {}
    Point(double _x, double _y, double _z, double _dist, uint8_t _reflec, double _azimuth, int _laserID): x(_x), y(_y), z(_z), dist(_dist), reflec(_reflec), azimuth(_azimuth), laserID(_laserID) {}
};

Point cal_point(double dist, double azi, uint8_t re, int ld) {
  double omega = LASER_ANGLES[ld] * PI / 180.0;
  double alpha = azi * PI / 180.0;
  double X = dist * cos(omega) * sin(alpha);
  double Y = dist * cos(omega) * cos(alpha);
  double Z = dist * sin(omega);

  return Point(Y, -X, Z, dist, re, azi, ld);
}

vector<Lidar_MCduo_2023::coeff> ransac_coeff;

vector<Point> arr;
vector<Point> arr2;
Lidar_MCduo_2023::ground_msg_arr ground_cam;

PCXYZI objs_points;

//cluster information struct using at after clustering
struct objectInfo {
    pcl::PointIndices* objPoints;
    string classes;
    unsigned int idx;
    float x;
    float y;
    float z;
    float xMin;
    float yMin;
    float zMin;
    float xMax;
    float yMax;
    float zMax;
    short intensity;
}; //순서 유지 필수

//func
void ROI(const sensor_msgs::PointCloud2ConstPtr&, int);
void makeCropBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
//void DownSampling(PCXYZI&, PCXYZI::Ptr, PCXYZI::Ptr);
void Clustering (PCXYZI::Ptr, PCXYZI&, bool, bool);
void afterClusteringProcess(PCXYZI::Ptr, PCXYZI&, vector<pcl::PointIndices>&);
void DBScanClustering(PCXYZI::Ptr, PCXYZI&);
void EuclideanClustering(PCXYZI::Ptr, PCXYZI&);
void RanSaC(PCXYZI::Ptr);
string send_msg_minmax(float, float, float, float);
void object_msg_process(const vector<struct objectInfo>&);
//--------------------------
void ground_extraction();
void Parsing_lidar_data(const char*);


class Filter{
public:
    void DY_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag);
    void DY_filter(vector<objectInfo>& objs, bool flag);
    void jiwon_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag);
    void jiwon_filter(vector<objectInfo>& objs, bool flag);
    inline bool check_in(PXYZI a, PXYZI b) { return ((abs(a.x - b.x) <= REMOVE_FACTOR) && (abs(a.y - b.y) <= REMOVE_FACTOR)); }
    void generate_return_PointCloud(PCXYZI::Ptr inputCloud, PCXYZI& returnCloud, vector<objectInfo>& objs);
    
    void TC_filter(vector<objectInfo>& objs, bool flag);
};
Filter FT;

class Fps{
public:
    Fps();
    void update();
private:
    double prev_clock;
    double cur_clock;
    double interval;
    double m_fps;
    size_t m_count;
};

class RT{
public:
    RT();
    static void start();
    static void end_cal(const char*);
private:
    static double prev_clock;
    static double cur_clock;
    static double interval;
};
double RT::prev_clock;
double RT::cur_clock;
double RT::interval;

inline float cal_dist(float x, float y){ return sqrt(x*x+y*y); }
inline double cal_dist_21(double x2, double x1, double y2, double y1) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));}
inline float cal_dist_21(float x2, float x1, float y2, float y1) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));}
inline float MidPt(float a, float b){ return (a + b) / 2; }

template<typename T> //this func is used all code
void pub_process(T& input, sensor_msgs::PointCloud2& output){
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(input, tmp_PCL);                     //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
}

float Distance2D(float x1, float y1,float x2,float y2)
{
  return pow( pow(x1-x2, 2) + pow(y1-y2, 2), 0.5);
}

struct TRACKING_OBJ {
    float Target_X;  
    float Target_Y;  
    float Predicted_X;  
    float Predicted_Y;  
    float Update_X;
    float Update_Y;

    int idx;
    
    MatrixXd F_;
    MatrixXd X_;
    MatrixXd Prev_X_;
    MatrixXd P_;
    MatrixXd H_;
    MatrixXd Z_;
    MatrixXd R_;
    MatrixXd Q_;
    MatrixXd K_;
    MatrixXd I4 = MatrixXd::Identity(4, 4);

    int ObjValid;  // 있, 없 판단
    int InitFlag;  // 초기화 판단
    int UpdateCnt;  //  
    int DeadCnt;  // Delete 변수
    time_point<system_clock> StartTime;  
    time_point<system_clock> EndTime;  
    float ar_dt;
    float Prev_X;
    float Prev_Y;
    float Init_Vx;
    float Init_Vy;
};

struct CANDIDATE{
    float curX;
    float curY;
    int curValid;
    int NewFlag;
};

vector<TRACKING_OBJ> TrackingObj;
vector<CANDIDATE> ObjCandidate; 
vector<CANDIDATE> ObjCandidate_prev;

void UpdatePreviousPosition(int);
void PushTrackingObj(int, int);
void UpdateTrackingObj(int, int, float, float);
void FindNearObj();
void Initialization(int);
void DeleteTrackingObj(int);
void FindNew();
void Prediction(int);
void Update(int) ;
void tracking() ;

#endif