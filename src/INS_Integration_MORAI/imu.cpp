#include "utility.h"
#include "gps.h"
#include "ins.h"
#include "imu_emulator/Navigation.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>



#define PI 3.14

using namespace std;



class ImuEmulator
{
public:

    std::mutex mtx;
    ros::NodeHandle nh;
    ros::Subscriber subGPS;
    ros::Subscriber subIMU;

    ros::Publisher pubGpsPath;
    ros::Publisher pubINS;
    ros::Publisher pubImuPath;


    Eigen::MatrixXd X;
    Eigen::MatrixXd dX;
    Eigen::MatrixXd dX0;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd H;
    Eigen::MatrixXd K;
    Eigen::MatrixXd P;

    Eigen::MatrixXd w_ibb;
    Eigen::MatrixXd w_enn;
    Eigen::MatrixXd w_ien;
    Eigen::MatrixXd w_inn;
    Eigen::MatrixXd Cbn;
    Eigen::MatrixXd f_ned;
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    Eigen::MatrixXd A;

    double ecc ;
    double R0 ;
    double Ome ;
    double g = 9.81 ;
    double ecc_2 ;
    double degree ;
    double radian ;
    double get_data_init_flag;
    double dt;

    double Rm; 
    double Rt ;
    double Rmm;
    double Rtt;

    double rho_n;
    double rho_e; 
    double rho_d; 

    double f_n;
    double f_e;
    double f_d;

    double lat;
    double lon;
    double h; 

    double v_n;
    double v_e; 
    double v_d;

    double cur_accel_x; 
    double cur_accel_y; 
    double cur_accel_z;

    double cur_gyro_x; 
    double cur_gyro_y; 
    double cur_gyro_z;

    double gps_v_n;
    double gps_v_e;
    double gps_v_d;

    double cur_N;
    double cur_E;
    double cur_D;

    double prev_N;
    double prev_E;
    double prev_D;



    double init_longitude =  37.29839822283;
    double init_latitude  =  127.20551035053;
    double init_altitude  =  81.432;
    double roll = -1.509754626;
    double pitch= -1.33087089;
    double yaw=   167.094096014; 



    double imuTime = 0;
    double lastImuT_opt = 0;
    int flag = 0;
    double gpsTime = 0;
    double lastGpsT_opt = 0;
    int gps_flag = 0;
    double dt_gps;
    int bias_cnt = 0;




    // init
    double real_lat_init;
    double real_lon_init;
    double real_h_init;

    double sim_lat_init;
    double sim_lon_init;
    double sim_h_init;

    double real_roll_init;
    double real_pitch_init; 
    double real_yaw_init;

    double sim_roll_init;
    double sim_pitch_init;
    double sim_yaw_init;

    // currnet time LLH
    double real_lat;
    double real_lon;
    double real_h;

    double sim_lat;
    double sim_lon;
    double sim_h;

    // currnet time velocity in NED Frame
    double real_v_n;
    double real_v_e; 
    double real_v_d;

    double sim_v_n;
    double sim_v_e;
    double sim_v_d;

    // currnet time attitude in NED Frame
    double real_roll;
    double real_pitch; 
    double real_yaw;

    double sim_roll;
    double sim_pitch;
    double sim_yaw;

    // currnet time velocity in NED Frame
    double real_accel_x;
    double real_accel_y; 
    double real_accel_z;

    double sim_accel_x;
    double sim_accel_y;
    double sim_accel_z;

    // currnet time velocity in NED Frame
    double real_gyro_x;
    double real_gyro_y; 
    double real_gyro_z;

    double sim_gyro_x;
    double sim_gyro_y;
    double sim_gyro_z;

    double modeled_lat;
    double modeled_lon;
    double modeled_h;
    double modeled_v_n;
    double modeled_v_e;
    double modeled_v_d;
    double modeled_roll;
    double modeled_pitch;
    double modeled_yaw;
    double modeled_accel_x;
    double modeled_accel_y;
    double modeled_accel_z;
    double modeled_gyro_x;
    double modeled_gyro_y;
    double modeled_gyro_z;

    IMUDataType imu;
    INSDataType ins;
    
    ImuEmulator()
    {

      ROS_INFO("Initial Setting Start!\n");

      ROS_INFO("Start!\n");

      InitialSetting();

      // subGPS = nh.subscribe<sensor_msgs::NavSatFix>("/vectornav/GPS",      5, &ImuEmulator::GPSHandler,   this, ros::TransportHints().tcpNoDelay());
      
      subIMU   = nh.subscribe<sensor_msgs::Imu>    ("/vectornav/IMU",   2000, &ImuEmulator::IMUHandler,   this, ros::TransportHints().tcpNoDelay());

      // subGPS = nh.subscribe<sensor_msgs::NavSatFix>("/vectornav/GPS",      5, &ImuEmulator::GPSHandler,   this, ros::TransportHints().tcpNoDelay());
      
      // subIMU   = nh.subscribe<sensor_msgs::Imu>    ("/vectornav/IMU",   2000, &ImuEmulator::IMUHandler,   this, ros::TransportHints().tcpNoDelay());

      // pubINS   = nh.advertise<nav_msgs::Odometry>("INS", 2000);

      // pubImuPath  = nh.advertise<nav_msgs::Path>    ("ins/imu/path", 1);
      // pubImuPath  = nh.advertise<nav_msgs::Path>    ("ins/path", 1);
      // pubGpsPath  = nh.advertise<nav_msgs::Path>    ("GPS_ins", 1);

    }


    // void Gt2ImuEmulator()
    // {

    // pos = [Lat, lon, hgt];

    // wx = typecast(uint8(data1(58:65)),'double');
    // wy = typecast(uint8(data1(66:73)),'double');
    // wz = typecast(uint8(data1(74:81)),'double');
    // ax = typecast(uint8(data1(82:89)),'double');
    // ay = typecast(uint8(data1(90:97)),'double');
    // az = typecast(uint8(data1(98:105)),'double');

    // if cnt == 1

    //     ax_er = (1 + S_aL)*ax + b_a + n_a*sqrt(Hz)*randn();
    //     ay_er = (1 + S_aL)*ay + b_a + n_a*sqrt(Hz)*randn();
    //     az_er = (1 + S_aL)*az + b_a + n_a*sqrt(Hz)*randn();

    //     wx_er = (1 + S_gL)*wx + deg2rad(b_g)/3600 + deg2rad(n_g)/3600*sqrt(Hz)*randn();
    //     wy_er = (1 + S_gL)*wy + deg2rad(b_g)/3600 + deg2rad(n_g)/3600*sqrt(Hz)*randn();
    //     wz_er = (1 + S_gL)*wz + deg2rad(b_g)/3600 + deg2rad(n_g)/3600*sqrt(Hz)*randn();
    //     time0 = time;
    // else

    //     ax_er = (1 + S_aL)*ax + b_a + axer_prev*exp(-1/Hz/tau_a) + b_aM*randn() + n_a*sqrt(Hz)*randn();
    //     ay_er = (1 + S_aL)*ay + b_a + ayer_prev*exp(-1/Hz/tau_a) + b_aM*randn() + n_a*sqrt(Hz)*randn();
    //     az_er = (1 + S_aL)*az + b_a + azer_prev*exp(-1/Hz/tau_a) + b_aM*randn() + n_a*sqrt(Hz)*randn();

    //     wx_er = (1 + S_gL)*wx + deg2rad(b_g)/3600 + wxer_prev*exp(-1/Hz/tau_g) + deg2rad(b_gM)/3600*randn() + deg2rad(n_g)/3600*sqrt(Hz)*randn();
    //     wy_er = (1 + S_gL)*wy + deg2rad(b_g)/3600 + wyer_prev*exp(-1/Hz/tau_g) + deg2rad(b_gM)/3600*randn() + deg2rad(n_g)/3600*sqrt(Hz)*randn();
    //     wz_er = (1 + S_gL)*wz + deg2rad(b_g)/3600 + wzer_prev*exp(-1/Hz/tau_g) + deg2rad(b_gM)/3600*randn() + deg2rad(n_g)/3600*sqrt(Hz)*randn();

    // end

    // }




    void IMUHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {

      std::lock_guard<std::mutex> lock(mtx);

      double imuTime = ROS_TIME(imu_raw);

      if (flag ==0)
      {
        lastImuT_opt = imuTime; 
        dt = 0.005;
        flag = 1;
      } 
      else{
        
        dt = imuTime - lastImuT_opt;

        lastImuT_opt = imuTime; 
      
      }
    
      imu.accel_x = imu_raw -> linear_acceleration.x; 
      imu.accel_y = imu_raw -> linear_acceleration.y; 
      imu.accel_z = imu_raw -> linear_acceleration.z;

      imu.gyro_x = imu_raw -> angular_velocity.x; 
      imu.gyro_y = imu_raw -> angular_velocity.y; 
      imu.gyro_z = imu_raw -> angular_velocity.z;
      imu.dt = dt;

      cur_accel_x = imu.accel_x;
      cur_accel_y = imu.accel_y;
      cur_accel_z = imu.accel_z;
      cur_gyro_x  = imu.gyro_x;
      cur_gyro_y  = imu.gyro_y;
      cur_gyro_z  = imu.gyro_z;

      INS_error_model();
      
      INS_Mechanization();

      Prediction();

      Nedconverter();

      plot();

      imu_emulator::Navigation Nav;


    }


    void ODOMHandler(const nav_msgs::Odometry::ConstPtr& imu_raw)
    {
      std::lock_guard<std::mutex> lock(mtx);


      double imuTime = ROS_TIME(imu_raw);

      if (flag ==0)
      {
        lastImuT_opt = imuTime; 
        dt = 0.005;
        flag = 1;
      } 
      else{
        
        dt = imuTime - lastImuT_opt;

        lastImuT_opt = imuTime; 
      
      }
    
      cur_accel_x = imu_raw -> twist.twist.linear.x; 
      cur_accel_y = imu_raw -> twist.twist.linear.y;
      cur_accel_z = imu_raw -> twist.twist.linear.z;

      cur_gyro_x = imu_raw -> twist.twist.angular.x;; 
      cur_gyro_y = imu_raw -> twist.twist.angular.y;; 
      cur_gyro_z = imu_raw -> twist.twist.angular.z;;

      INS_error_model();

      Nedconverter();
      nav_msgs::Odometry INSOdometry;
      // INSOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(span_roll, span_pitch, span_yaw);
      INSOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * degree, pitch* degree, yaw* degree);
      INSOdometry.pose.pose.position.x = cur_E;
      INSOdometry.pose.pose.position.y = cur_N;
      INSOdometry.pose.pose.position.z = -cur_D;
      
      INSOdometry.twist.twist.linear.x = v_n; 
      INSOdometry.twist.twist.linear.y = v_e;
      INSOdometry.twist.twist.linear.z = v_d;

      pubINS.publish(INSOdometry);

      // static tf
      static tf::TransformBroadcaster tfMap2Odom;
      static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
      tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, ros::Time::now(), mapFrame, odometryFrame));

      // publish tf
      static tf::TransformBroadcaster tfOdom2BaseLink;
      tf::Transform tCur;
      tf::poseMsgToTF(INSOdometry.pose.pose, tCur);

      tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, ros::Time::now(), odometryFrame, baselinkFrame);
      tfOdom2BaseLink.sendTransform(odom_2_baselink);


      static nav_msgs::Path imuPath;

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = odometryFrame;
      pose_stamped.pose = INSOdometry.pose.pose;

      imuPath.poses.push_back(pose_stamped);
      imuPath.header.stamp = ros::Time::now();
      imuPath.header.frame_id = odometryFrame;

      pubImuPath.publish(imuPath);

    }




    void plot()
    {
      nav_msgs::Odometry INSOdometry;
      
      INSOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * degree, pitch* degree, yaw* degree);
      INSOdometry.pose.pose.position.x = cur_E;
      INSOdometry.pose.pose.position.y = cur_N;
      INSOdometry.pose.pose.position.z = -cur_D;
      
      INSOdometry.twist.twist.linear.x = v_n; 
      INSOdometry.twist.twist.linear.y = v_e;
      INSOdometry.twist.twist.linear.z = v_d;

      pubINS.publish(INSOdometry);

      // static tf
      static tf::TransformBroadcaster tfMap2Odom;
      static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
      tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, ros::Time::now(), mapFrame, odometryFrame));

      // publish tf
      static tf::TransformBroadcaster tfOdom2BaseLink;
      tf::Transform tCur;
      tf::poseMsgToTF(INSOdometry.pose.pose, tCur);

      tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, ros::Time::now(), odometryFrame, baselinkFrame);
      tfOdom2BaseLink.sendTransform(odom_2_baselink);


      static nav_msgs::Path imuPath;

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = odometryFrame;
      pose_stamped.pose = INSOdometry.pose.pose;

      imuPath.poses.push_back(pose_stamped);
      imuPath.header.stamp = ros::Time::now();
      imuPath.header.frame_id = odometryFrame;

      pubImuPath.publish(imuPath);



    }
    
void InitialSetting()
    {

        //zero matrix

        X       = Eigen::MatrixXd(1,9);
        R       = Eigen::MatrixXd(6,6);
        dX      = Eigen::MatrixXd(15,1);
        dX0     = Eigen::MatrixXd(15,1);
        Q       = Eigen::MatrixXd(15,15);
        H       = Eigen::MatrixXd(6,15);
        K       = Eigen::MatrixXd(1,15);
        P       = Eigen::MatrixXd(15,15);
        w_ibb   = Eigen::MatrixXd(1,3);
        w_enn   = Eigen::MatrixXd(1,3);
        w_ien   = Eigen::MatrixXd(1,3);
        w_inn   = Eigen::MatrixXd(1,3);
        Cbn     = Eigen::MatrixXd(3,3);
        f_ned   = Eigen::MatrixXd(3,1);
        V       = Eigen::MatrixXd(3,1);
        F       = Eigen::MatrixXd(15,15);
        A       = Eigen::MatrixXd(15,15);

        X(0,0) =  37.44778470266;
        X(0,1) =  126.65343931204;
        X(0,2) =  10.3622;
        X(0,3) =  0;
        X(0,4) =  0;
        X(0,5) =  0;
        X(0,6) =  -1.509754626;
        X(0,7) =  1.46028007;
        X(0,8) =  141.0565;

        dX(0,0)  = -6.97664e-09     ;
        dX(1,0)  = 7.15927e-09      ;
        dX(2,0)  = 0.33608         ;
        dX(3,0)  = -0.00283069      ;
        dX(4,0)  = -0.00158075      ;
        dX(5,0)  = 0.000746894      ;
        dX(6,0)  = 0.000721   ;
        dX(7,0)  = -0.00100877      ;
        dX(8,0)  = 0.000739909      ;
        dX(9,0)  = -0.231594       ;
        dX(10,0) = 0.152261      ;
        dX(11,0) = 0.08364    ;
        dX(12,0) = 3.3118e-06 ;
        dX(13,0) = 2.10972e-05      ;
        dX(14,0) = -6.86601e-06   ;

        dX0 = dX;
        
        Q.setZero();
        Q(0,0) = 1;  //  LLH
        Q(1,1) = 1;
        Q(2,2) = 100000;
        Q(3,3) = 0.00000000000001; // v_ned
        Q(4,4) = 0.00000000000001;
        Q(5,5) = 0;
        Q(6,6) = 100000000000000;
        Q(7,7) = 100000000000000;
        Q(8,8) = 100000000000000;
        Q = Q * 0.00000000000000001;

        R.setIdentity();
        R(0,0) = 0.00000000000000000001;
        R(1,1) = 0.00000000000000000001;
        R(2,2) = 0.0000000000000000000000000001;
        R(3,3) = 0.000000000000000000000000000000000000000000000000000000000000000000001;
        R(4,4) = 0.00000000000000000000000000000000000000000000000000000000000001;
        R(5,5) = 0.000000000000000000000000000000000000000000000000001;
        R = R * 0.1;


        H.setZero();
        H.topLeftCorner(6,6).setIdentity();


        P.setZero();
        for (int i = 0; i < P.cols(); i++)
        {
        P(i,i) = 0.1;
        }

        ecc     = 0.0818192;         // earth's eccentricity;
        R0      = 6378137.0;          // earth's mean radius (m)
        Ome     = 7.2921151467e-5;   //  earth's rotational rate (rad/s)
        g       = 9.81;                //  # gravity
        ecc_2   = pow(ecc,2) ;
        degree  = PI/ 180;
        radian  = 180 / PI;
        dt      = 0.005;

        F.setZero();

        Nedconverter();

        prev_N = cur_N; 
        prev_E = cur_E; 
        prev_D = cur_D;

    }





    // void GPSHandler(const sensor_msgs::NavSatFix::ConstPtr& gps_raw)
    // {
    //   std::lock_guard<std::mutex> lock(mtx);

    //   sensor_msgs::NavSatFix thisGps = gpsConverter(*gps_raw);

    //   Eigen::MatrixXd z(6,1);

    //   double gps_lat = thisGps.latitude;
    //   double gps_lon = thisGps.longitude;
    //   double gps_alt = thisGps.altitude;
      
    //   lat = gps_lat;
    //   lon = gps_lon;
    //   h   = gps_alt;

    //   Nedconverter();

    //   double dt_gps = 0.05;

    //   gps_v_n = (cur_N - prev_N) /dt_gps;
    //   gps_v_e = (cur_E - prev_E) /dt_gps;
    //   gps_v_d = (cur_D - prev_D) /dt_gps; 

    //   prev_N = cur_N; 
    //   prev_E = cur_E; 
    //   prev_D = cur_D;

    //   double gpsTime = ROS_TIME(gps_raw);

    //   if (gps_flag ==0)
    //   {
    //     lastGpsT_opt = gpsTime; 
    //     dt_gps = 0.005;
    //     gps_flag = 1;
    //   } 
    //   else{
        
    //     dt_gps = gpsTime - lastGpsT_opt;

    //     lastGpsT_opt = gpsTime; 
      
    //   }


    //   gps_v_n = (cur_N - prev_N) /dt_gps;
    //   gps_v_e = (cur_E - prev_E) /dt_gps;
    //   gps_v_d = (cur_D - prev_D) /dt_gps; 


    //   if(gpsTime - lastGpsT_opt >= 0.2)
    //   {
    //             lastGpsT_opt = gpsTime; 
    //   prev_N = cur_N; 
    //   prev_E = cur_E; 
    //   prev_D = cur_D;

    //   K = P * (H.transpose()) * ((H * P * H.transpose() + R).inverse()); 

    //   z << X(0) - gps_lat,
    //         X(1) - gps_lon,
    //         X(2) - gps_alt,
    //         X(3) - gps_v_n,
    //         X(4) - gps_v_e,
    //         X(5) - gps_v_d;

    //   dX = dX + (K * (z - H *dX));
    //   P = P - K * H * P;

    //   X(0) = X(0) - dX(0,0);
    //   X(1) = X(1) - dX(1,0);
    //   X(2) = X(2) - dX(2,0);
    //   X(3) = X(3) - dX(3,0);
    //   X(4) = X(4) - dX(4,0);
    //   X(5) = X(5) - dX(5,0);

    //   dX = dX0;

    //   // publish latest odometry
    //   nav_msgs::Odometry INSOdometry;
    //   INSOdometry.pose.pose.position.x = cur_E;
    //   INSOdometry.pose.pose.position.y = cur_N;
    //   INSOdometry.pose.pose.position.z = -cur_D;
    //   INSOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    //   static nav_msgs::Path imuPath;

    //   geometry_msgs::PoseStamped pose_stamped;
    //   pose_stamped.header.stamp = ros::Time::now();
    //   pose_stamped.header.frame_id = odometryFrame;
    //   pose_stamped.pose = INSOdometry.pose.pose;

    //   imuPath.poses.push_back(pose_stamped);
    //   imuPath.header.stamp = ros::Time::now();
    //   imuPath.header.frame_id = odometryFrame;

    //   pubGpsPath.publish(imuPath);

    //   }


    // }






    // void plot()
    // {

    //   // publish latest odometry
    //   nav_msgs::Odometry INSOdometry;
    //   INSOdometry.pose.pose.position.x = cur_E;
    //   INSOdometry.pose.pose.position.y = cur_N;
    //   INSOdometry.pose.pose.position.z = -cur_D;
      
    //   INSOdometry.twist.twist.linear.x = v_n; 
    //   INSOdometry.twist.twist.linear.y = v_e;
    //   INSOdometry.twist.twist.linear.z = v_d;

    //   // INSOdometry.pose.pose.position.x = 0;
    //   // INSOdometry.pose.pose.position.y = 0;
    //   // INSOdometry.pose.pose.position.z = 0;


    //   // double o_x = imu_raw->pose.pose.orientation.x;
    //   // double o_y = imu_raw->pose.pose.orientation.y;
    //   // double o_z = imu_raw->pose.pose.orientation.z;
    //   // double o_w = imu_raw->pose.pose.orientation.w;
    //   // INSOdometry.pose.pose.orientation.x = o_x ;
    //   // INSOdometry.pose.pose.orientation.y = o_y ;
    //   // INSOdometry.pose.pose.orientation.z = o_z ;
    //   // INSOdometry.pose.pose.orientation.w = o_w ;


    //   // INSOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(span_roll, span_pitch, span_yaw);
    //   INSOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * degree, pitch* degree, yaw* degree);
      
    //   pubINS.publish(INSOdometry);

    //   // static tf
    //   static tf::TransformBroadcaster tfMap2Odom;
    //   static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    //   tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, ros::Time::now(), mapFrame, odometryFrame));

    //   // publish tf
    //   static tf::TransformBroadcaster tfOdom2BaseLink;
    //   tf::Transform tCur;
    //   tf::poseMsgToTF(INSOdometry.pose.pose, tCur);

    //   tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, ros::Time::now(), odometryFrame, baselinkFrame);
    //   tfOdom2BaseLink.sendTransform(odom_2_baselink);


    //   static nav_msgs::Path imuPath;

    //   geometry_msgs::PoseStamped pose_stamped;
    //   pose_stamped.header.stamp = ros::Time::now();
    //   pose_stamped.header.frame_id = odometryFrame;
    //   pose_stamped.pose = INSOdometry.pose.pose;

    //   imuPath.poses.push_back(pose_stamped);
    //   imuPath.header.stamp = ros::Time::now();
    //   imuPath.header.frame_id = odometryFrame;

    //   pubImuPath.publish(imuPath);
    // }

void INS_error_model()
    {
      lat     = X(0,0);
      lon     = X(0,1);
      h       = X(0,2); 
      v_n     = X(0,3);
      v_e     = X(0,4); 
      v_d     = X(0,5); 
      roll    = X(0,6);
      pitch   = X(0,7);
      yaw     = X(0,8); 

      earth_model();
    
      gyro_measurement();

      navigation_component();

      earth_rotation();

      cal_Cbn();

      cal_accel_body_2_nav();

      cal_new_Cbn();

      cal_attitude();

      cal_velocity();

      cal_poistion(lat, lon, h);

      X << lat, lon, h , v_n, v_e, v_d, roll, pitch , yaw;

    }




    void Prediction()
    {
      Eigen::MatrixXd tmp(15,15);

      tmp  = dt * F;

      A = tmp.exp();

      dX = A * dX;
      
      P = A * P * A.transpose() + Q;

    }


    void INS_Mechanization()
    { 
      Eigen::MatrixXd F_pp(3,3);
      Eigen::MatrixXd F_pv(3,3);
      Eigen::MatrixXd F_vp(3,3);
      Eigen::MatrixXd F_vv(3,3);
      Eigen::MatrixXd F_vphi(3,3);
      Eigen::MatrixXd F_phip(3,3);
      Eigen::MatrixXd F_phiv(3,3);
      Eigen::MatrixXd F_phiphi(3,3);
      Eigen::MatrixXd Cbn_minus(3,3);
 

      double cos_lat = cos(lat * degree);
      double tan_lat = tan(lat * degree);
      double sin_lat = sin(lat * degree);

      F_pp  << Rmm * rho_e / (Rm + h), 0 ,  rho_e / (Rm + h),
          rho_n * (tan_lat - Rtt / (Rt + h)) / cos_lat,  0,  -rho_n / (cos_lat * (Rt + h)),
          0, 0, 0;

      F_pv << 1 / (Rm + h), 0, 0,
              0, 1 / (cos_lat * (Rt + h)), 0,
                0, 0, -1;
      

      F_vp << Rmm * rho_e * v_d / (Rm + h) - (rho_n / (pow(cos_lat,2))  + 2 * w_ien(0)) * v_e - rho_n * rho_d * Rtt, 0,
          rho_e * v_d / (Rm + h) - rho_n * rho_d,
          (2 * w_ien(0) + rho_n / (pow(cos_lat,2)) + rho_d * Rtt / (Rt + h)) * v_n - (
                      rho_n * Rtt / (Rt + h) - 2 * w_ien(2)) * v_d, 0, rho_d * v_n / (Rt + h) - rho_n * v_d / (Rt + h),
          (pow(rho_n, 2)) * Rtt + (pow(rho_e, 2)) * Rmm - 2 * w_ien(2) * v_e, 0, pow(rho_n, 2) + pow(rho_e, 2);

      

      F_vv <<  v_d / (Rm + h), 2 * rho_d + 2 * w_ien(2), -rho_e,
              -2 * w_ien(2) - rho_d, (v_n * tan_lat + v_d) / (Rt + h), 2 * w_ien(0) + rho_n,
              2 * rho_e, -2 * w_ien(0) - 2 * rho_n, 0;
              

      F_vphi <<
           0, -f_d, f_e,
           f_d, 0, -f_n,
           -f_e, f_n, 0;
      

      F_phip << 
          w_ien(2) - rho_n * Rtt / (Rt + h), 0, -rho_n / (Rt + h),
          -rho_e * Rmm / (Rm + h), 0, -rho_e / (Rm + h),
          -w_ien(0) - rho_n / (pow(cos_lat,2)) - rho_d * Rtt / (Rt + h), 0, -rho_d / (Rt + h);

      F_phiv <<
          0, 1 / (Rt + h), 0,
          -1 / (Rm + h), 0, 0,
          0, -tan_lat / (Rt + h), 0;

      F_phiphi << 
          0, w_ien(2) + rho_d, -rho_e,
          -w_ien(2) - rho_d, 0, w_ien(0) + rho_n,
          rho_e, -w_ien(0) - rho_n, 0;

      Cbn_minus = (-1) * Cbn;


      for (int i = 0; i < 3; i++)
      {
        for(int k = 0; k < 3; k++)
        {
          F(i,k) = F_pp(i,k);
        }
      }

      for (int i = 0; i < 3; i++)
      {
        for(int k = 3; k < 6; k++)
        {
          F(i,k) = F_pv(i,k-3);
        }
      }

    for (int i = 3; i < 6; i++)
      {
        for(int k = 0; k < 3; k++)
        {
          F(i,k) = F_vp(i-3,k);
        }
      }


      for (int i = 3; i < 6; i++)
      {
        for(int k = 3; k <6; k++)
        {
          F(i,k) = F_vv(i-3,k-3);
        }
      }

      for (int i = 3; i < 6; i++)
      {
        for(int k = 6; k <9; k++)
        {
          F(i,k) = F_vphi(i-3,k-6);
        }
      }

      for (int i = 3; i < 6; i++)
      {
        for(int k = 9; k <12 ; k++)
        {
          F(i,k) = Cbn(i-3,k-9);
        }
      }

      for (int i = 6; i < 9; i++)
      {
        for(int k = 0; k <3 ; k++)
        {
          F(i,k) = F_phip(i-6,k);
        }
      }

      for (int i = 6; i < 9; i++)
      {
        for(int k = 3; k <6 ; k++)
        {
          F(i,k) = F_phiv(i-6,k-3);
        }
      }

      for (int i = 6; i < 9; i++)
      {
        for(int k = 6 ; k <9 ; k++)
        {
          F(i,k) = F_phiphi(i-6,k-6);
        }
      }

      for (int i = 6; i < 9; i++)
      {
        for(int k = 12 ; k <15 ; k++)
        {
          F(i,k) = Cbn_minus(i-6,k-12);
        }
      }
    }

    void DCM2eul_bnearth_model()
    {
      Rm = R0 * (1 - ecc_2) / pow(1 - ecc_2 * pow(sin(lat * degree),  2),  1.5);

      Rt = R0 / pow((1 - ecc_2 * pow(sin(lat * degree), 2)),  0.5);

      Rmm = (3 * R0 * (1 - ecc_2) * ecc_2 * sin(lat * degree)) * cos(lat * degree) / pow(1 - ecc_2 * (pow(sin(lat * degree), 2)) , 2.5);

      Rtt = R0 * ecc_2 * sin(lat * degree) * cos(lat * degree) / pow(1 - ecc_2 * pow(sin(lat * degree), 2) , 1.5);

    }

    void gyro_measurement()
    {
      w_ibb(0,0) = cur_gyro_x;
      w_ibb(0,1) = cur_gyro_y;
      w_ibb(0,2) = cur_gyro_z;
    }

    void navigation_component()
    {
      w_enn(0,0) =  v_e / (Rt + h);
      w_enn(0,1) = -v_n / (Rm + h);
      w_enn(0,2) = -v_e * tan(lat * degree) / (Rt + h); 
      
      rho_n = w_enn(0,0);
      rho_e = w_enn(0,1);
      rho_d = w_enn(0,2);
    }

    void earth_rotation()
    {
      w_ien(0,0) = Ome * cos(lat * degree);
      w_ien(0,1) = 0;
      w_ien(0,2) = -Ome * sin(lat * degree);
      
      w_inn(0,0) = w_ien(0,0) + rho_n;
      w_inn(0,1) = w_ien(0,1) + rho_e;
      w_inn(0,2) = w_ien(0,2) + rho_d;

    }

    void Nedconverter()
    {
    
      GpsTran gps_tran(init_longitude, init_latitude, init_altitude);
      GpsDataType gps;
      NedDataType ned; 

      gps.longitude = lat;
      gps.latitude  = lon;
      gps.altitude  = h;
	    gps_tran.fromGpsToNed(ned, gps);

      cur_N = ned.x_north;
      cur_E = ned.y_east;
      cur_D = ned.z_down ;

    }

    Eigen::MatrixXd skew(double x,double y,double z)
    {
      Eigen::MatrixXd M(3,3);
      M << 0, -z,  y,
           z,  0, -x,
          -y,  x,  0;
      
      return M;
    }

    void cal_Cbn()
    {
      Cbn = eul2DCM_bn(roll,pitch, yaw);
    }

    void cal_accel_body_2_nav()
    {
        Eigen::MatrixXd tmp(3,1);
        tmp <<  cur_accel_x, cur_accel_y, cur_accel_z;
        
        f_ned = Cbn * tmp;  // body to ned accel
        
        f_n = f_ned(0,0); 
        f_e = f_ned(1,0);
        f_d = f_ned(2,0);
         
    }

    void cal_new_Cbn()
    {

      Eigen::MatrixXd tmp1(3,3);
      Eigen::MatrixXd tmp2(3,3);

      tmp1 = skew(w_ibb(0,0) - dX(12,0),w_ibb(0,1) - dX(13,0),w_ibb(0,2) - dX(14,0));
      tmp2 = skew(w_inn(0,0),w_inn(0,1),w_inn(0,2));
      Cbn = Cbn + (Cbn * tmp1 - tmp2 * Cbn) * dt;

    }

    void cal_attitude()
    {
      DCM2eul_bn(Cbn, &roll, &pitch, &yaw);
    }

    void cal_velocity()
    {
      Eigen::Vector3d V_ned(3);
      Eigen::MatrixXd tmp1(3,1);
      Eigen::MatrixXd m_7(3,1);
      Eigen::Vector3d tmp2(3);
      Eigen::MatrixXd tmp3(3,1);
      Eigen::MatrixXd tmp4(1,3);

      Eigen::MatrixXd tmp5(3,1);

      Eigen::MatrixXd tmp_reshaped(3,1);
      Eigen::MatrixXd V_ned_matrix(3,1);

      V_ned << v_n, v_e, v_d;
      
      V_ned_matrix<< v_n, v_e, v_d;

      tmp1 << cur_accel_x -dX(9,0),cur_accel_y -dX(10,0),cur_accel_z -dX(11,0);
      tmp2 << w_ien(0,0) * 2 + w_enn(0,0), w_ien(0,1) * 2 + w_enn(0,1), w_ien(0,2) * 2 + w_enn(0,2);
      tmp4 = tmp2.cross(V_ned);
      tmp3 << 0,0, g;

      tmp_reshaped << tmp4(0),tmp4(1),tmp4(2);

      V = V_ned_matrix + (Cbn * tmp1  - tmp4 + tmp3) * dt;

      v_n = V(0,0);
      v_e = V(1,0);
      v_d = V(2,0);

    }

    void cal_poistion(double cur_lat,double cur_lon,double cur_h)
    {
      lat = cur_lat + (180 / PI) * (v_n / (Rm + cur_h)) * dt;
      lon = cur_lon + (180 / PI) * (v_e / ((Rt + cur_h) * cos(cur_lat * degree))) * dt;
      h   = cur_h -  (v_d) * dt;

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");

    ImuEmulator ins;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
