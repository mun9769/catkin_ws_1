#include <fstream>
#include <algorithm>
#include <string>
#include <vector>
#include <sstream>
#include <istream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <INS_Integration_MORAI/GNSSInfo.h>
// #include "plnctrl/totalInfo.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
// morai ¹öÀü
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>

using namespace std;
using namespace Eigen;

INS_Integration_MORAI::GNSSInfo INS;

class SubandPub
{
private:
    ros::NodeHandle nh;
    ros::Publisher gnssPub;
    ros::Subscriber gpsSub;
    ros::Subscriber imuSub;

    MatrixXd latitude;
    MatrixXd V;
    MatrixXd V_ned;
    MatrixXd Vt;
    MatrixXd Vt2;

    MatrixXd c1;
    MatrixXd c2;
    MatrixXd c3;
    MatrixXd c31;
    MatrixXd c32;
    MatrixXd c33;
    MatrixXd c34;
    MatrixXd c35;

    MatrixXd m_7;
    MatrixXd acc;
    VectorXd euler_angles;

    MatrixXd Pt;
    MatrixXd Pt2;

    MatrixXd g;

    MatrixXd Rotation;

    MatrixXd gM;
    MatrixXd av;
    MatrixXd av_2;
    MatrixXd av_exp;
    MatrixXd Bias;
    MatrixXd Accel_Bias;

    MatrixXd delta_Vt;

    MatrixXd X;
    MatrixXd dX;
    MatrixXd dX0;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd H;
    MatrixXd K;
    MatrixXd P;

    MatrixXd w_ibb;
    MatrixXd w_enn;
    MatrixXd w_ien;
    MatrixXd w_inn;
    MatrixXd Cbn;
    MatrixXd minCbn;

    MatrixXd f_ned;
    MatrixXd F;
    MatrixXd A;

    MatrixXd F_pp;
    MatrixXd F_pv;
    MatrixXd F_vp;
    MatrixXd F_vv;
    MatrixXd F_vphi;
    MatrixXd F_phip;
    MatrixXd F_phiv;
    MatrixXd F_phiphi;
    MatrixXd Cbn_minus;

    double sum_lat = 0;
    double sum_lon = 0;
    double sum_alt = 0;
    double sum_v_n = 0;
    double sum_v_e = 0;
    double sum_v_u = 0;
    double sum_roll = 0;
    double sum_pitch = 0;
    double sum_yaw = 0;
    double sum_la_x = 0;
    double sum_la_y = 0;
    double sum_la_z = 0;
    double sum_av_x = 0;
    double sum_av_y = 0;
    double sum_av_z = 0;

    double prev_lat  ;
    double prev_lon  ;
    double prev_alt  ;
    double prev_v_n  ;
    double prev_v_e  ;
    double prev_v_u  ;
    double prev_roll ;
    double prev_pitch;
    double prev_yaw  ;
    double prev_la_x ;
    double prev_la_y ;
    double prev_la_z ;
    double prev_av_x ;
    double prev_av_y ;
    double prev_av_z ;

    double INSTime, lastINST;
    double GPSTime, lastGPST;
    double lat = 0, lon = 0, alt = 0, h = 0;
    double gps_lat = 1, gps_lon = 1, gps_alt = 1;
    double cos_lat, sin_lat, tan_lat;
    double cur_sum1 = 0, cur_sum2 = 0, cur_sum3 = 0, cur_sum4 = 0;
    double cur_E_prev, cur_N_prev, cur_E_aft, cur_N_aft;
    double av_x, av_y, av_z;
    double la_x, la_y, la_z;
    double qu_x, qu_y, qu_z, qu_w;
    double t0, t1, t2, t3, t4, t5;
    double setting_yaw = 0, setting_yaw2 = 10000;
    double roll, pitch, yaw, delta_yaw;
    double q_roll, q_pitch, q_yaw;
    double rho_n, rho_e, rho_d;
    double f_n, f_e, f_d;
    double x, y, z;
    double Rm, Rt, Rmm, Rtt;
    double v_e = 0, v_n = 0, v_u = 0;
    double gps_v_e = 0, gps_v_n = 0, gps_v_u = 0;
    double cur_E, cur_N, cur_U, prev_E = 0, prev_N = 0, prev_U = 0, gps_E, gps_N, gps_U, d_E = 0, d_N = 0, d_U = 0, d_E2 = 0, d_N2 = 0, d_U2 = 0;
    double degree = M_PI / 180.0, radian = 180.0 / M_PI; // degree -> rad / rad -> degree
    bool gps_received = false;
    double dt, dt_gps, d_length;
    // wgs84 earth's eccentricity / wgs84 earth's mean radius (m) / earth's rotational rate (rad/s)
    double ecc = 0.0818192, R0 = 6378137.0, Ome = 7.2921151467e-5, gravity = -9.81, ecc_2 = ecc * ecc; // earth value
    double wgs84_f = 1 / 298.257223563, wgs84_e2 = 2 * wgs84_f - pow(wgs84_f, 2);                      // wgs84 model
    double lat_origin, lon_origin, alt_origin;                                                         // origin LLA(rad)
    double E, N, U;                                                                                    // NED ÁÂÇ¥°è¿¡Œ­ÀÇ º¯È¯ °á°ú
    bool flag1 = 0, flag2 = 0, IMU_flag = 0, GPS_flag = 0, Ref_flag = 0, jamming_flag = 0, tunnel_out_flag = 0, tunnel_out_signal = 0, Straight_signal = 0, Tnl_out_once = 0;
    double Heading = 0;
    float speed = 0, dspeed = 0, prev_speed = 0;
    float wheel_angle = 0, prev_wheel_angle = 0;
    int Tnl_signal = 0;
    int gps_cnt = 0, imu_cnt = 0, yaw_set_cnt = 0, prev_gps_cnt, prev_imu_cnt;
    double roll_mean = 0, pitch_mean = 0, yaw_mean = 0;
    double lat_mean = 0, lon_mean = 0, alt_mean = 0;
    // double delta_roll = 0, delta_pitch = 0, delta_yaw = 0, delta_lat = 0, delta_lon = 0, delta_alt = 0, delta_v_e = 0,delta_v_n, delta_v_u = 0; //delta_av_x = 0, delta_av_y = 0, delta_av_z = 0, delta_la_x = 0,delta_la_y = 0, delta_la_z = 0;
    // double prev_roll, prev_pitch, prev_yaw, prev_lat, prev_lon, prev_alt, prev_v_e, prev_v_n, prev_v_u; // prev_la_x, prev_la_y, prev_la_z,  prev_av_x, prev_av_y, prev_av_z;
    double d_length_var = 0.3, gps_var = 1, prev_cnt_var = 5, aft_cnt_var = prev_cnt_var * 2;

    // int speed;                                                                                                     // ins start or end siganl

public:
    SubandPub()
    {
        initialsetting();
        gpsSub = nh.subscribe<morai_msgs::GPSMessage>("/gps", 1, &SubandPub::gpsCallback, this);
        imuSub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &SubandPub::imuCallback, this);
        gnssPub = nh.advertise<INS_Integration_MORAI::GNSSInfo>("/GNSScom", 1);
    }

    void initialsetting()
    {
        X = MatrixXd(1, 9);    // ok
        R = MatrixXd(15, 15);  // (6,6) -> (15,15)
        dX = MatrixXd(15, 1);  // ok
        dX0 = MatrixXd(15, 1); // ok
        Q = MatrixXd(15, 15);  // ok
        H = MatrixXd(15, 15);  // ok // (6,15) -> (15,15)
        K = MatrixXd(15, 15);  // 이전에는 행렬인 것만 선언하고 공간은 선언 안해줌 15 by 15 해주려고 바꿈 (1,15) -> (15,15)
        P = MatrixXd(15, 15);
        w_ibb = MatrixXd(1, 3);
        w_enn = MatrixXd(1, 3);
        w_ien = MatrixXd(1, 3);
        w_inn = MatrixXd(1, 3);
        Cbn = MatrixXd(3, 3);
        Cbn = MatrixXd(3, 3);
        f_ned = MatrixXd(3, 1);
        V = MatrixXd(3, 1);
        F = MatrixXd(15, 15);
        A = MatrixXd(15, 15);
        V_ned = MatrixXd(3, 1);
        c1 = MatrixXd(1, 3);
        c2 = MatrixXd(1, 3);
        c3 = MatrixXd(3, 1);
        c31 = MatrixXd(1, 3);
        c32 = MatrixXd(1, 3);
        c33 = MatrixXd(1, 3);
        c34 = MatrixXd(1, 3);
        c35 = MatrixXd(1, 3);
        m_7 = MatrixXd(15, 1);
        acc = MatrixXd(3, 1);
        Vt = MatrixXd(3, 1);
        Vt2 = MatrixXd(3, 1);
        Pt = MatrixXd(3, 1);
        Pt2 = MatrixXd(3, 1);
        av = MatrixXd(3, 3);
        g = MatrixXd(3, 1);
        av_exp = MatrixXd(3, 3);
        av_2 = MatrixXd(3, 3);
        Rotation = MatrixXd(3, 3);
        euler_angles = VectorXd(3);
        F_pp = MatrixXd(3, 3);
        F_pv = MatrixXd(3, 3);
        F_vp = MatrixXd(3, 3);
        F_vv = MatrixXd(3, 3);
        F_vphi = MatrixXd(3, 3);
        F_phip = MatrixXd(3, 3);
        F_phiv = MatrixXd(3, 3);
        F_phiphi = MatrixXd(3, 3);
        Cbn_minus = MatrixXd(3, 3);

        dX(0, 0)  = 0.000528385112155;
        dX(1, 0)  = 0.001903292298067;
        dX(2, 0)  = 0.119029140416519;
        dX(3, 0)  = 0.001136779904715;
        dX(4, 0)  = 0.025140765509064;
        dX(5, 0)  = 0.767850705055676;
        dX(6, 0)  = 0.000095216205449;
        dX(7, 0)  = 0.000007047578971;
        dX(8, 0)  = 0.000064377190190;
        dX(9, 0)  = 0.000049586430756;
        dX(10, 0) = 0.000137711460060;
        dX(11, 0) = 0.000341346308273;
        dX(12, 0) = 0.000004785728765;
        dX(13, 0) = 0.000001781730025;
        dX(14, 0) = 0.000000026285999;

        dX0 = dX;

        Q.setIdentity(); // 대각 요소는 1, 대각 제외한 모든 요소=0
        // Q.setZero();     // 행렬 모든 요소=0
        Q(0, 0) = 100000000;
        Q(1, 1) = 100000000;
        Q(2, 2) = 10;
        Q(3, 3) = 100;
        Q(4, 4) = 100;
        Q(5, 5) = 1;
        Q(6, 6) = 1000000;
        Q(7, 7) = 100000000;
        Q(8, 8) = 100000000;
        Q(9, 9) = 100;
        Q(10, 10) = 100;
        Q(11, 11) = 100;
        Q(12, 12) = 100;
        Q(13, 13) = 100;
        Q(14, 14) = 100;

        Q = Q * 10000;

        R.setZero();
        R(0, 0) = 0.0000000000000001; // lla
        R(1, 1) = 0.0000000000000001;
        R(2, 2) = 0.0000000000000001;
        R(3, 3) = 0.01; // v_ned
        R(4, 4) = 0.0000001;
        R(5, 5) = 0.0000000000001;
        R(6, 6) = 0.00000000000001; // euler
        R(7, 7) = 0.00000000000001;
        R(8, 8) = 0.000001;
        R(9, 9) = 1; // l_a
        R(10, 10) = 1;
        R(11, 11) = 1;
        R(12, 12) = 1; // a_v
        R(13, 13) = 1;
        R(14, 14) = 1;

        H.setIdentity(); // H.setZero();
        for (int i = 0; i < H.cols(); i++)
        {
            H(i, i) = 1;
        }
        P.setZero();
        for (int i = 0; i < P.cols(); i++)
        {
            P(i, i) = 0.1;
        }
        V_ned.setZero(); // 이거 안해서 nan 뜸. 으!!
    }
    void IMUSetting()
    {
        cout << "Setting yaw : ";
        cin >> setting_yaw;
        X(0, 6) = q_roll;
        X(0, 7) = q_pitch;
        X(0, 8) = setting_yaw;
        yaw = setting_yaw;
    }
    void AfterTunnelSetting()
    {
        X(0, 6) = q_roll;
        X(0, 7) = q_pitch;
        X(0, 8) = setting_yaw2;
        yaw = setting_yaw2;
        tunnel_out_flag = false;
    }
    void StraightSetting()
    {
        X(0, 8) = setting_yaw2;
        yaw = setting_yaw2;
    }
    void ENUconverter(double &lat, double &lon, double &alt, double &cur_E, double &cur_N, double &cur_U)
    {
        double lat_point, lon_point, alt_point; // find LLA(rad)
        lat_point = lat * degree;
        lon_point = lon * degree;
        alt_point = alt;

        double chi = sqrt(1 - wgs84_e2 * pow(sin(lat_point), 2));
        double q = (R0 / chi + alt_point) * cos(lat_point);

        double find_x = q * cos(lon_point);
        double find_y = q * sin(lon_point);
        double find_z = ((R0 * (1 - wgs84_e2) / chi) + alt_point) * sin(lat_point);

        double ref_chi = sqrt(1 - wgs84_e2 * pow(sin(lat_origin), 2));
        double ref_q = (R0 / ref_chi + alt_origin) * cos(lat_origin);
        double ref_x = ref_q * cos(lon_origin);
        double ref_y = ref_q * sin(lon_origin);
        double ref_z = ((R0 * (1 - wgs84_e2) / ref_chi) + alt_origin) * sin(lat_origin);

        double dx = find_x - ref_x;
        double dy = find_y - ref_y;
        double dz = find_z - ref_z;

        cur_E = -sin(lon_origin) * dx + cos(lon_origin) * dy;
        cur_N = -sin(lat_origin) * cos(lon_origin) * dx - sin(lat_origin) * sin(lon_origin) * dy + cos(lat_origin) * dz;
        cur_U = (cos(lat_origin) * cos(lon_origin) * dx + cos(lat_origin) * sin(lon_origin) * dy + sin(lat_origin) * dz);
    }

    void INS_ERROR_MODEL()
    {
        lat = X(0, 0);
        lon = X(0, 1);
        alt = X(0, 2);
        v_n = X(0, 3);
        v_e = X(0, 4);
        v_u = -X(0, 5);
        roll = X(0, 6);
        pitch = X(0, 7);
        yaw = X(0, 8);

        // earth model
        Rm = R0 * (1 - ecc_2) / pow((1 - ecc_2 * pow((sin(lat * degree)), 2)), 1.5);                                                     // À§µµ(lat)¿¡ µû¶ó ºÏ-³² ¹æÇâÀž·ÎÀÇ ¿ø°æ°Åž®
        Rt = R0 / (1 - ecc_2 * pow(pow((sin(lat * degree)), 2), 0.5));                                                                   // À§µµ(lat)¿¡ µû¶ó µ¿-Œ­ ¹æÇâÀž·ÎÀÇ ¿ø°æ°Åž®
        Rmm = (3 * R0 * (1 - ecc_2) * ecc_2 * sin(lat * degree)) * cos(lat * degree) / pow((1 - ecc_2 * ((sin(lat * degree)), 2)), 2.5); // À§µµ(lat)¿¡ µû¶ó ºÏ-³² ¹æÇâÀÇ °î·ü ¹Ý°æ
        Rtt = R0 * ecc_2 * sin(lat * degree) * cos(lat * degree) / pow((1 - ecc_2 * pow((sin(lat * degree)), 2)), 1.5);                  // À§µµ(lat)¿¡ µû¶ó µ¿-Œ­ ¹æÇâÀÇ °î·ü ¹Ý°æ
        // X = Eigen::Matrix(1,9)

        av.setZero();
        g.setZero();

        // ŒøŒö ÀÚÀÌ·Î °ª °¢ŒÓµµ
        w_ibb << av_x, av_y, av_z;

        // ECEF¿¡ ŽëÇÑ nav frameÀÇ angular rate
        w_enn << v_e / (Rt + h), -v_n / (Rm + h), -v_e * tan(lat * degree) / (Rt + h);
        rho_n = w_enn(0);
        rho_e = w_enn(1);
        rho_d = w_enn(2);

        // Áö±ž ÀÚÀü °¢ŒÓµµ
        w_ien << Ome * cos(lat * degree), 0, -Ome * sin(lat * degree);

        // À§Ä¡¿Í ŒÓ·ÂÀ» »ç¿ëÇÏ¿© ±žÇÑ °¢ŒÓµµ
        w_inn << w_ien(0, 0) + w_enn(0, 0), w_ien(0, 1) + w_enn(0, 1), w_ien(0, 2) + w_enn(0, 2);

        // ÈžÀüÇà·Ä
        Cbn << cos(pitch * degree) * cos(yaw * degree),
            cos(yaw * degree) * sin(pitch * degree) * sin(roll * degree) -
                cos(roll * degree) * sin(yaw * degree),
            cos(roll * degree) * cos(yaw * degree) * sin(pitch * degree) +
                sin(roll * degree) * sin(yaw * degree),
            cos(pitch * degree) * sin(yaw * degree),
            cos(roll * degree) * cos(yaw * degree) +
                sin(pitch * degree) * sin(roll * degree) * sin(yaw * degree),
            cos(roll * degree) * sin(pitch * degree) * sin(yaw * degree) -
                cos(yaw * degree) * sin(roll * degree),
            -sin(pitch * degree),
            cos(pitch * degree) * sin(roll * degree),
            cos(pitch * degree) * cos(roll * degree); // body to ned DCM  eul2DCM_bn

        // Cbn ¿¬»êÀ» À§ÇØ Œ±ŸðÇÑ µÎ Çà·Ä
        c1 = skew(w_ibb(0, 0) - dX(12, 0), w_ibb(0, 1) - dX(13, 0), w_ibb(0, 2) - dX(14, 0)); // w_ibb(0,3) -> w_ibb(0,2)
        c2 = skew(w_inn(0, 0), w_inn(0, 1), w_inn(0, 2));                                     // Çà·Ä ¿¬»ê ÀÇ¹Ì ÆÄŸÇ

        // imu·ÎºÎÅÍ ¹ÞÀº °¡ŒÓµµ
        acc << la_x, la_y, la_z;

        f_ned = Cbn * acc; // Cosine direction matrix @=matrix x matrix body to ned accel
        f_n = f_ned(0, 0); // n accel
        f_e = f_ned(1, 0); // e accel
        f_d = f_ned(2, 0); // d accel

        Cbn = Cbn + (Cbn * c1 - c2 * Cbn) * dt; // mechanization DCM differential equation
        DCM2eul_bn(Cbn);                        // ¿ÀÀÏ·¯ °¢ Ÿ÷µ¥ÀÌÆ®
        
        c31 << w_ien(0, 0) * 2, w_ien(0, 1) * 2, w_ien(0, 2) * 2;
        c32 << w_enn(0, 0), w_enn(0, 1), w_enn(0, 2);
        c33 = c31 + c32;
        c34 << c33(0, 1) * V_ned(2, 0) - c33(0, 2) * V_ned(1, 0),
            c33(0, 2) * V_ned(0, 0) - c33(0, 0) * V_ned(2, 0),
            c33(0, 1) * V_ned(1, 0) - c33(0, 1) * V_ned(0, 0);
        c35 = reshape(c34, 3, 1);
        c3 << la_x - dX(9, 0) - c35(0, 0), la_y - dX(10, 0) - c35(1, 0), la_z - dX(11, 0) - c35(2, 0);

        // ŒÓ·Â Ÿ÷µ¥ÀÌÆ®
        V_ned << v_n, v_e, -v_u;
        g << 0, 0, gravity;
        m_7 = Cbn * c3 - c35;
        V = V_ned + (m_7 + g) * dt; // <- V = reshape(V_ned, 3, 1) + (m_7 + g) * dt;

        v_n = V(0, 0);
        v_e = V(1, 0);
        v_u = -V(2, 0);

        // Position Ÿ÷µ¥ÀÌÆ®

        lat = lat + (radian) * (v_n / (Rm + h)) * dt;
        lon = lon + (radian) * (v_e / ((Rt + h) * cos(lat * degree))) * dt;
        alt = alt + (v_u)*dt;
        h = h + (v_u)*dt;

        X << lat, lon, alt, v_n, v_e, -v_u, roll, pitch, yaw;
    }

    void INS_MECHANIZATION()
    {
        cos_lat = cos(lat * degree);
        sin_lat = sin(lat * degree);
        tan_lat = tan(lat * degree);

        F_pp << Rmm * rho_e / (Rm + h), 0, rho_e / (Rm + h),
            rho_n * (tan_lat - Rtt / (Rt + h)) / cos_lat, 0, -rho_n / (cos_lat * (Rt + h)),
            0, 0, 0;
        F_pv << 1 / (Rm + h), 0, 0,
            0, 1 / (cos_lat * (Rt + h)), 0,
            0, 0, -1;
        F_vp << Rmm * rho_e * (-v_u) / (Rm + h) - (rho_n / (pow(cos_lat, 2)) + 2 * w_ien(0)) * v_e - rho_n * rho_d * Rtt, 0,
            rho_e * (-v_u) / (Rm + h) - rho_n * rho_d,
            (2 * w_ien(0) + rho_n / (pow(cos_lat, 2)) + rho_d * Rtt / (Rt + h)) * v_n - (rho_n * Rtt / (Rt + h) - 2 * w_ien(2)) * (-v_u), 0, rho_d * v_n / (Rt + h) - rho_n * (-v_u) / (Rt + h),
            (pow(rho_n, 2)) * Rtt + (pow(rho_e, 2)) * Rmm - 2 * w_ien(2) * v_e, 0, pow(rho_n, 2) + pow(rho_e, 2);
        F_vv << (-v_u) / (Rm + h), 2 * rho_d + 2 * w_ien(2), -rho_e,
            -2 * w_ien(2) - rho_d, (v_n * tan_lat - v_u) / (Rt + h), 2 * w_ien(0) + rho_n,
            2 * rho_e, -2 * w_ien(0) - 2 * rho_n, 0;
        F_vphi << 0, -f_d, f_e,
            f_d, 0, -f_n,
            -f_e, f_n, 0;
        F_phip << w_ien(2) - rho_n * Rtt / (Rt + h), 0, -rho_n / (Rt + h),
            -rho_e * Rmm / (Rm + h), 0, -rho_e / (Rm + h),
            -w_ien(0) - rho_n / (pow(cos_lat, 2)) - rho_d * Rtt / (Rt + h), 0, -rho_d / (Rt + h);
        F_phiv << 0, 1 / (Rt + h), 0,
            -1 / (Rm + h), 0, 0,
            0, -tan_lat / (Rt + h), 0;
        F_phiphi << 0, w_ien(2) + rho_d, -rho_e,
            -w_ien(2) - rho_d, 0, w_ien(0) + rho_n,
            rho_e, -w_ien(0) - rho_n, 0;
        minCbn = (-1) * Cbn;

        F.setZero();
        for (int i = 0; i < 3; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                F(i, k) = F_pp(i, k);
            }
        }
        for (int i = 0; i < 3; i++)
        {
            for (int k = 3; k < 6; k++)
            {
                F(i, k) = F_pv(i, k - 3);
            }
        }
        for (int i = 3; i < 6; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                F(i, k) = F_vp(i - 3, k);
            }
        }
        for (int i = 3; i < 6; i++)
        {
            for (int k = 3; k < 6; k++)
            {
                F(i, k) = F_vv(i - 3, k - 3);
            }
        }
        for (int i = 3; i < 6; i++)
        {
            for (int k = 6; k < 9; k++)
            {
                F(i, k) = F_vphi(i - 3, k - 6);
            }
        }
        for (int i = 3; i < 6; i++)
        {
            for (int k = 9; k < 12; k++)
            {
                F(i, k) = Cbn(i - 3, k - 9);
            }
        }
        for (int i = 6; i < 9; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                F(i, k) = F_phip(i - 6, k);
            }
        }
        for (int i = 6; i < 9; i++)
        {
            for (int k = 3; k < 6; k++)
            {
                F(i, k) = F_phiv(i - 6, k - 3);
            }
        }
        for (int i = 6; i < 9; i++)
        {
            for (int k = 6; k < 9; k++)
            {
                F(i, k) = F_phiphi(i - 6, k - 6);
            }
        }
        for (int i = 6; i < 9; i++)
        {
            for (int k = 12; k < 15; k++)
            {
                F(i, k) = Cbn_minus(i - 6, k - 12);
            }
        }
    }

    void Prediction()
    {
        Eigen::MatrixXd F2(15, 15);
        F2 = dt * F;                   // continuousÇÑ ¿µ¿ª-> discrete¿µ¿ª¿¡Œ­ ÀûºÐÇÑŽÙŽÂ ŽÀ³Š?
        A = F2.exp();                  // continuouse -> discrete Å×ÀÏ·¯ ±ÞŒö
        dX = A * dX;                   // X2 = AX1
        P = A * P * A.transpose() + Q; //+- »ý°¢ ŸÈÇÏ°í ÀýŽñ°ª
    }

    Eigen::MatrixXd skew(double x, double y, double z)
    {
        Eigen::MatrixXd M(3, 3);
        M << 0, -z, y,
            z, 0, -x,
            -y, x, 0;

        return M;
    }

    void DCM2eul_bn(const Eigen::MatrixXd &matrix)
    {
        euler_angles[0] = atan2(matrix(2, 1), matrix(2, 2)) * 180.0 / M_PI;
        euler_angles[1] = asin(-matrix(2, 0)) * 180.0 / M_PI;
        euler_angles[2] = atan2(matrix(1, 0), matrix(0, 0)) * 180.0 / M_PI;
        roll = euler_angles[0];
        pitch = euler_angles[1];
        yaw = euler_angles[2];
    }

    Eigen::MatrixXd reshape(Eigen::MatrixXd m, int col, int row)
    {
        Eigen::MatrixXd M(col, row);
        M << m(0, 0), m(0, 1), m(0, 2);
        return M;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_imu)
    {
        imu_cnt++;
        INSTime = msg_imu->header.stamp.toSec();

        if (flag1 == 0)
        {
            lastINST = INSTime;
            flag1 = 1;
            dt = 0.02;
        }
        else
        {
            dt = INSTime - lastINST;
            lastINST = INSTime;
        }
        if (GPS_flag == true)
        {
            // if (jamming_flag == true && IMU_flag == true) // Àç¹Ö±ž°£ÀÏ ¶§ Ä®žž ÇÊÅÍ ŸÈ µ¹°í Àç¹Ö ±ž°£ ÀÌÀü µ¥ÀÌÅÍ ŒÛœÅ
            // {
            //     cout << "--------------------Yaw set--------------------" << endl;
            //     cout << "lat: " << lat << "    "
            //          << "lon: " << lon << "    "
            //          << "alt: " << alt << endl;
            //     cout << "yaw : " << yaw << endl;

            //     INS.tunnel_out_signal = SubandPub::tunnel_out_signal; // ÅÍ³ÎÀ» ³ª¿À°í 2¹øÂ° setting_yawžŠ ±žÇÒ µ¿ŸÈžž ŒÛœÅ
            //     INS.jamming_signal = SubandPub::jamming_flag;         // Àç¹Ö œÃ±×³ÎÀ» ¹ÞÀžžé ±×¶§ ¹ÞŽÂ µ¥ÀÌÅÍµéÀº ÆÇŽÜ Ãø¿¡Œ­ ¹«œÃÇÏ°í ¶óÀÌŽÙ·Î ÁÖÇà
            //     INS.heading =    SubandPub::yaw;
            //     INS.latitude =   SubandPub::lat;
            //     INS.longitude =  SubandPub::lon;
            //     INS.altitude =   SubandPub::alt;

            //     cout << "jamming_flag : " << jamming_flag << endl;
            //     cout << "Tnl_signal : " << Tnl_signal << endl;
            //     cout << "tunnel_out_signal : " << tunnel_out_signal << endl;

            //     pub.publish(INS);
            // }
            // else if (jamming_flag == false)
            //{
            av_x = msg_imu->angular_velocity.x;
            av_y = msg_imu->angular_velocity.y;
            av_z = -(msg_imu->angular_velocity.z);

            la_x = msg_imu->linear_acceleration.x;
            la_y = msg_imu->linear_acceleration.y;
            la_z = -(msg_imu->linear_acceleration.z);

            qu_x = msg_imu->orientation.x;
            qu_y = msg_imu->orientation.y;
            qu_z = msg_imu->orientation.z;
            qu_w = msg_imu->orientation.w;

            t0 = 2 * (qu_w * qu_x + qu_y * qu_z);
            t1 = 1 - 2 * (qu_x * qu_x + qu_y * qu_y);
            t2 = sqrt(1 + 2 * (qu_w * qu_y - qu_z * qu_x));
            t3 = sqrt(1 - 2 * (qu_w * qu_y - qu_z * qu_x));
            t4 = 2 * (qu_w * qu_z + qu_x * qu_y);
            t5 = 1 - 2 * (qu_y * qu_y + qu_z * qu_z);

            q_roll = atan2(t0, t1) * radian;
            q_pitch = (2 * atan2(t2, t3) - M_PI / 2) * radian;

            if (IMU_flag == false)
            {
                IMUSetting(); // todo: automatic setting
                IMU_flag = true;
            }
            if (tunnel_out_flag == true)
            {
                AfterTunnelSetting(); // ÅÍ³Î ºüÁ®³ª¿ÔÀ» ¶§žž enu ÁÂÇ¥·Î ÃÊ±â yaw °ª ŽÙœÃ ŒŒÆÃ
            }

            if (Straight_signal == true)
            {
                delta_yaw = abs(setting_yaw2 - yaw);

                cout << "delta_yaw : " << delta_yaw << endl;
                cout << "yaw : " << yaw << endl;

                if (delta_yaw > 7 && delta_yaw < 50) //&& delta_yaw < 30
                {
                    cout << "-------------------straight_set----------------------" << endl;
                    StraightSetting();
                }
                // else if (delta_yaw > 345 && delta_yaw < 355)
                // {
                //     //ŸÆ¹«°Íµµ ŸÈÇÏ°í ³ÑŸî°¡±â ±Ùµ¥ Â÷ÇÇ
                // }
            }

            INS_ERROR_MODEL(); // »çœÇ»ó kalman_filter

            INS_MECHANIZATION(); // F °è»ê (PžŠ °è»êÇÏ±â À§ÇÑ 15*15 Çà·Ä)

            Prediction(); // P°è»ê (¿ÀÂ÷°øºÐ»ê)

            ENUconverter(lat, lon, alt, cur_E, cur_N, cur_U); // ÁÂÇ¥°è ÀüÈ¯

            ENUconverter(gps_lat, gps_lon, gps_alt, gps_E, gps_N, gps_U); // ÁÂÇ¥°è ÀüÈ¯

            cout << "------------------Kalman Filter After--------------------" << endl;
            cout << "lat: " << lat << "    "
                 << "lon: " << lon << "    "
                 << "alt: " << alt << endl;
            // cout << "GPS lat: " << gps_lat << "    "
            //      << "GPS lon: " << gps_lon << "    "
            //      << "GPS alt: " << gps_alt << endl;

            // cout << "roll: " << roll << "    "
            //      << "pitch: " << pitch << "    "
            cout << "kalman yaw: " << yaw << endl;

            cout << "\n"
                 << "\n";

            double d_lat   =  abs(lat   - prev_lat);
                double d_lon   =  abs(lon   - prev_lon);
                double d_alt   =  abs(alt   - prev_alt);
                double d_v_n   =  abs(v_n   - prev_v_n);
                double d_v_e   =  abs(v_e   - prev_v_e);
                double d_v_u   =  abs(v_u   - prev_v_u);
                double d_roll  =  abs(roll  - prev_roll);
                double d_pitch =  abs(pitch - prev_pitch);
                double d_yaw   =  abs(yaw   - prev_yaw);
                double d_la_x  =  abs(la_x  - prev_la_x);
                double d_la_y  =  abs(la_y  - prev_la_y);
                double d_la_z  =  abs(la_z  - prev_la_z);
                double d_av_x  =  abs(av_x  - prev_av_x);
                double d_av_y  =  abs(av_y  - prev_av_y);
                double d_av_z  =  abs(av_z  - prev_av_z);

                sum_lat += d_lat;
                sum_lon += d_lon;
                sum_alt += d_alt;
                sum_v_n += d_v_n;
                sum_v_e += d_v_e;
                sum_v_u += d_v_u;
                sum_roll += d_roll;
                sum_pitch += d_pitch;
                sum_yaw += d_yaw;
                sum_la_x += d_la_x;
                sum_la_y += d_la_y;
                sum_la_z += d_la_z;
                sum_av_x += d_av_x;
                sum_av_y += d_av_y;
                sum_av_z += d_av_z;

                double bias_lat   = sum_lat / imu_cnt;
                double bias_lon   = sum_lon / imu_cnt;
                double bias_alt   = sum_alt / imu_cnt;
                double bias_v_n   = sum_v_n / imu_cnt;
                double bias_v_e   = sum_v_e / imu_cnt;
                double bias_v_u   = sum_v_u / imu_cnt;
                double bias_roll  = sum_roll / imu_cnt;
                double bias_pitch = sum_pitch / imu_cnt;
                double bias_yaw   = sum_yaw / imu_cnt;
                double bias_la_x  = sum_la_x / imu_cnt;
                double bias_la_y  = sum_la_y / imu_cnt;
                double bias_la_z  = sum_la_z / imu_cnt;
                double bias_av_x  = sum_av_x / imu_cnt;
                double bias_av_y  = sum_av_y / imu_cnt;
                double bias_av_z  = sum_av_z / imu_cnt;

                // prev_yaw = yaw;
                // prev_lat = lat;
                // prev_lon = lon;
                // prev_alt = h;

                prev_lat   = lat;
                prev_lon   = lon;
                prev_alt   = alt;
                prev_v_n   = v_n;
                prev_v_e   = v_e;
                prev_v_u   = v_u;
                prev_roll  = roll;
                prev_pitch = pitch;
                prev_yaw   = yaw;
                prev_la_x  = la_x;
                prev_la_y  = la_y;
                prev_la_z  = la_z;
                prev_av_x  = av_x;
                prev_av_y  = av_y;
                prev_av_z  = av_z;

            // prev_yaw = yaw;
            // prev_lat = lat;
            // prev_lon = lon;
            // prev_alt = h;

            INS.tunnel_out_signal = SubandPub::tunnel_out_signal;
            INS.jamming_signal = SubandPub::jamming_flag;
            INS.heading = SubandPub::yaw;
            INS.latitude = SubandPub::lat;
            INS.longitude = SubandPub::lon;
            INS.altitude = SubandPub::alt;

            prev_speed = speed;
            prev_wheel_angle = wheel_angle;

            gnssPub.publish(INS);
            //}
        }
    }
    int cal_sum(int yaw_set_cnt, int prev_cnt_var, int aft_cnt_var)
    {
        cout << "calculate sum" << endl;
        if (yaw_set_cnt < prev_cnt_var)
        {
            cur_sum1 += cur_E;
            cur_sum2 += cur_N;
        }
        else if (prev_cnt_var <= yaw_set_cnt && yaw_set_cnt < aft_cnt_var)
        {
            cur_sum3 += cur_E;
            cur_sum4 += cur_N;
        }
        yaw_set_cnt++;
        return yaw_set_cnt;
    }

    // void yaw_set(int cur_sum1, int cur_sum2, int cur_sum3, int cur_sum4)
    // {
    //     cout << "yaw set ---------------------" << endl;

    //     cur_E_prev = cur_sum1;
    //     cur_N_prev = cur_sum2;
    //     cur_E_aft = cur_sum3;
    //     cur_N_aft = cur_sum4;

    //     d_E2 = cur_E_aft - cur_E_prev;
    //     d_N2 = cur_N_aft - cur_N_prev;

    //     cout << "d_E2 : " << d_E2 << endl;
    //     cout << "d_N2 : " << d_N2 << endl;

    //     setting_yaw2 = atan2(d_E2, d_N2) * radian;

    //     // K.setIdentity();
    //     // initialsetting();

    //     X(0, 0) = gps_lat;
    //     X(0, 1) = gps_lon;
    //     X(0, 2) = gps_alt;
    //     X(0, 3) = 0;
    //     X(0, 4) = 0;
    //     X(0, 5) = 0;

    //     prev_E = cur_E;
    //     prev_N = cur_N;
    //     prev_U = cur_U;

    //     cur_sum1 = 0;
    //     cur_sum2 = 0;
    //     cur_sum3 = 0;
    //     cur_sum4 = 0;

    //     yaw_set_cnt = 0;
    // }

    void gpsCallback(const morai_msgs::GPSMessage::ConstPtr &msg)
    {
        gps_cnt++;
        // cout << "tunnel_out_cnt : " << tunnel_out_cnt << endl;

        GPSTime = msg->header.stamp.toSec();

        if (flag2 == 0)
        {
            lastGPST = GPSTime;
            flag2 = 1;
            dt_gps = 0.1;
        }
        else
        {
            dt_gps = GPSTime - lastGPST;
            lastGPST = GPSTime;
        }
        if (Ref_flag == 0) // Ã³Àœ ¹ÞŸÆ¿Â lla žŠ ¿øÁ¡Àž·Î Àâ°í ENU º¯È¯
        {
            lat_origin = (msg->latitude) * degree;
            lon_origin = (msg->longitude) * degree;
            alt_origin = msg->altitude;
            Ref_flag = 1;
        }
        if (GPS_flag == false || tunnel_out_signal == true) // signal -> flag
        {
            X(0, 0) = msg->latitude;
            X(0, 1) = msg->longitude;
            X(0, 2) = msg->altitude; // ŸøÀžžé nan ¶ä
            X(0, 3) = 0;
            X(0, 4) = 0;
            X(0, 5) = 0;

            gps_lat = msg->latitude;
            gps_lon = msg->longitude;
            gps_alt = msg->altitude;

            lat = gps_lat;
            lon = gps_lon;
            alt = gps_alt;
            h = gps_alt;

            GPS_flag = true;
        }

        if (IMU_flag == true)
        {
            Eigen::MatrixXd z(15, 1);
            gps_lat = msg->latitude;
            gps_lon = msg->longitude;
            gps_alt = msg->altitude;

            lat = gps_lat;
            lon = gps_lon;
            alt = gps_alt;
            h = gps_alt;

            ENUconverter(lat, lon, alt, cur_E, cur_N, cur_U); // ÁÂÇ¥°è ÀüÈ¯

            d_E = cur_E - prev_E;
            d_N = cur_N - prev_N;
            d_U = cur_U - prev_U;

            gps_v_e = d_E / dt_gps;
            gps_v_n = d_N / dt_gps;
            gps_v_u = d_U / dt_gps;

            prev_N = cur_N;
            prev_E = cur_E;
            prev_U = cur_U;

            //
            // if (Tnl_signal == 1 || gps_lat < gps_var || gps_lon < gps_var || gps_alt < gps_var || isnan(gps_lon) || gps_alt > 1000)
            // {
            //     jamming_flag = true; // tnl in

            // }
            // else
            // {
            if (Tnl_signal == 1 || gps_lat < gps_var || gps_lon < gps_var || gps_alt < gps_var || isnan(gps_lon) || gps_alt > 1000)
            {
                jamming_flag = true; // tnl in
            }

            if (Tnl_signal == 2 && Tnl_out_once == false)
            { // À§ÀÇ Á¶°Ç¹®¿¡Œ­ °Éž®Áö ŸÊ°í jamming flag°¡ true¶óžé ÅÍ³ÎÀ» ºüÁ®³ª¿ÔŽÙ°í ÆÇŽÜ.
                if (yaw_set_cnt < aft_cnt_var)
                {
                    yaw_set_cnt = cal_sum(yaw_set_cnt, prev_cnt_var, aft_cnt_var);
                    cout << "yaw_set_cnt: " << yaw_set_cnt << endl;
                    tunnel_out_signal = true;
                    // return;
                }
                else
                {
                    // yaw_set(cur_sum1, cur_sum2, cur_sum3, cur_sum4);
                    cout << "yaw set ---------------------" << endl;

                    cur_E_prev = cur_sum1 / aft_cnt_var;
                    cur_N_prev = cur_sum2 / aft_cnt_var;
                    cur_E_aft = cur_sum3 / aft_cnt_var;
                    cur_N_aft = cur_sum4 / aft_cnt_var;

                    d_E2 = cur_E_aft - cur_E_prev;
                    d_N2 = cur_N_aft - cur_N_prev;

                    setting_yaw2 = atan2(d_E2, d_N2) * radian;

                    X(0, 0) = gps_lat;
                    X(0, 1) = gps_lon;
                    X(0, 2) = gps_alt;
                    X(0, 3) = 0;
                    X(0, 4) = 0;
                    X(0, 5) = 0;

                    prev_E = cur_E;
                    prev_N = cur_N;
                    prev_U = cur_U;

                    cur_sum1 = 0;
                    cur_sum2 = 0;
                    cur_sum3 = 0;
                    cur_sum4 = 0;

                    yaw_set_cnt = 0;

                    // if(prev_N == cur_N)
                    jamming_flag = false;
                    tunnel_out_flag = true; // Âü
                    tunnel_out_signal = false;
                    Tnl_out_once = true;

                    // return;
                }
            }
            else if (Straight_signal == true) // ÅÍ³Î ³ª°¡°íŒ­ŽÂ Á÷Œ± ±ž°£ ŸøÀœ ÃÊ±âÁ÷Œ± ±ž°£ ŸøÀœ
            {
                cout << "striaght" << endl;

                cout << "delta_yaw : " << delta_yaw << endl;
                // cout << "yaw_set_cnt : " << yaw_set_cnt << endl; // À§ÀÇ Á¶°Ç¹®¿¡Œ­ °Éž®Áö ŸÊ°í jamming flag°¡ true¶óžé ÅÍ³ÎÀ» ºüÁ®³ª¿ÔŽÙ°í ÆÇŽÜ.
                if (yaw_set_cnt < aft_cnt_var)
                {
                    yaw_set_cnt = cal_sum(yaw_set_cnt, prev_cnt_var, aft_cnt_var);
                    // return ŸøŸîŒ­ else if¹® ÀÌÈÄ¿¡ ÀÖŽÂ update ºÎºÐ œÇÇàµÊ
                }
                else
                {
                    // yaw_set(cur_sum1, cur_sum2, cur_sum3, cur_sum4);
                    cout << "yaw set ---------------------" << endl;

                    cur_E_prev = cur_sum1;
                    cur_N_prev = cur_sum2;
                    cur_E_aft = cur_sum3;
                    cur_N_aft = cur_sum4;

                    d_E2 = cur_E_aft - cur_E_prev;
                    d_N2 = cur_N_aft - cur_N_prev;

                    setting_yaw2 = atan2(d_E2, d_N2) * radian;
                    // cout << "setting_yaw2 : " << setting_yaw2 << endl;
                    cout << "yaw : " << yaw << endl;

                    prev_E = cur_E;
                    prev_N = cur_N;
                    prev_U = cur_U;

                    cur_sum1 = 0;
                    cur_sum2 = 0;
                    cur_sum3 = 0;
                    cur_sum4 = 0;

                    yaw_set_cnt = 0;
                }
            }
            else if (Straight_signal == false && tunnel_out_signal == false)
            {
                cur_sum1 = 0;
                cur_sum2 = 0;
                cur_sum3 = 0;
                cur_sum4 = 0;

                yaw_set_cnt = 0;

                // return;
            }
            // cout << "setting_yaw2 : " << setting_yaw2 << endl;

            K = P * (H.transpose()) * ((H * P * H.transpose() + R).inverse());

            z.setZero();

            z(0, 0) = X(0, 0) - gps_lat;
            z(1, 0) = X(0, 1) - gps_lon;
            z(2, 0) = X(0, 2) - gps_alt;
            z(3, 0) = X(0, 3) - gps_v_n;
            z(4, 0) = X(0, 4) - gps_v_e;
            z(5, 0) = X(0, 5) - gps_v_u;

            dX = dX + (K * (z - H * dX));
            P = P - K * H * P;

            X(0, 0) = X(0, 0) - dX(0, 0); // lla
            X(0, 1) = X(0, 1) - dX(1, 0);
            X(0, 2) = X(0, 2) - dX(2, 0);
            X(0, 3) = X(0, 3) - dX(3, 0); // V_ned
            X(0, 4) = X(0, 4) - dX(4, 0);
            X(0, 5) = X(0, 5) - dX(5, 0);
            X(0, 6) = X(0, 6) - dX(6, 0); // Roll Pitch Yaw
            X(0, 7) = X(0, 7) - dX(7, 0);
            X(0, 8) = X(0, 8) - dX(8, 0);

            dX = dX0;
            prev_imu_cnt = imu_cnt;
            prev_gps_cnt = gps_cnt;
        }
    }
};

int main(int argc, char **argv)
{
    cout << fixed;
    cout.precision(15);

    ros::init(argc, argv, "GPSIMU");
    SubandPub SaP;
    ros::spin();
}

//Yaw Setting이 이상하다?
//1.straight 신호를 받을 때 이전 N, E 좌표가 SUM에 더해지고 나서 0으로 초기화되지 않은 채 터널을 나온 다음 yaw setting을 진행
//2. count 횟수가 너무 많다..?
//3. 몰라 근데 맨처음 yaw setting은 할때마다 첫 구간에서 29도 정도로 나와서 계산 자체는 맞다고 생각했는데 1 케이스일 확률이 제일 높을듯ㅜㅜ
