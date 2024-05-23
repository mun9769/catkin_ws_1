#include <Lidar_MCduo_2023/Lidar_declare.h>

#define PacketSize 1200 //+6 bytes
#define DataBlockSize 100 //bytes
#define ChannelDataBlockSize 96 //bytes
#define ChannelDataSize 3 //bytes

int UDP_cnt;

struct Point_UDP {
    double x, y, z, dist, azimuth;
    uint8_t reflec;
    int laser_id;
    bool is_ground = false;
    Point_UDP() {}
    Point_UDP(double _x, double _y, double _z, double _dist, uint8_t _reflec, double _azimuth, int _laser_id): x(_x), y(_y), z(_z), dist(_dist), reflec(_reflec), azimuth(_azimuth), laser_id(_laser_id) {}
};

Point_UDP cal_point_UDP(double dist, double azi, uint8_t re, int ld) {
  double omega = LASER_ANGLES[ld] * PI / 180.0;
  double alpha = azi * PI / 180.0;
  double X = dist * cos(omega) * sin(alpha);
  double Y = dist * cos(omega) * cos(alpha);
  double Z = dist * sin(omega);

  return Point_UDP(Y, -X, Z, dist, re, azi, ld);
}

vector<Point_UDP> packet_points;
vector<Point_UDP> raw_points;
vector<Point_UDP> ground_points;
vector<Point_UDP> other_points;

PCXYZI Point_UDP_to_PCXYZI(vector<Point_UDP> points) {
    PCXYZI pcl;
    pcl::PointXYZI tmp;
    for(int i = 0; i < points.size(); i++) {
        tmp.x = points[i].x;
        tmp.y = points[i].y;
        tmp.z = points[i].z;
        tmp.intensity = points[i].reflec;
        pcl.push_back(tmp);
    }

    return pcl;
}

void Publish_UDP_data() {
    UDP_cnt++;

    if(UDP_cnt == 50) {
        printf("\033[38;2;139;232;229mState\033[0m : \033[1;37m%d\033[0m\n", lidar_state);
        if(switch_UDP_communication) RT::end_cal("UDP_communication");
        else printf("\033[38;2;139;232;229mUPD_communication runtime\033[0m : \033[0mOFF\033[0m\n");
        RT::start();

        sensor_msgs::PointCloud2 output;
        PCXYZI pcl_output;
        pcl_output = Point_UDP_to_PCXYZI(raw_points);
        pub_process(pcl_output, output);
        pub_rawdata.publish(output);
        pcl_output = Point_UDP_to_PCXYZI(other_points);
        pub_process(pcl_output, output);
        pub_others.publish(output);
        pcl_output = Point_UDP_to_PCXYZI(ground_points);
        pub_process(pcl_output, output);
        pub_ground.publish(output);

        Lidar_MCduo_2023::ground_msg_arr ground_msg_arr;
        Lidar_MCduo_2023::ground_msg tmp_ground_msg;
        for(int i = 0; i < ground_points.size(); i++) {
            tmp_ground_msg.Gx = ground_points[i].x;
            tmp_ground_msg.Gy = ground_points[i].y;
            tmp_ground_msg.Gz = ground_points[i].z;
            tmp_ground_msg.LID = ground_points[i].laser_id;
            ground_msg_arr.ground_msg_arr.push_back(tmp_ground_msg);
        }
        ground_msg_arr.size = ground_points.size();
        pub_ground_to_cam.publish(ground_msg_arr);

        UDP_cnt = 0;
        raw_points.clear();
        other_points.clear();
        ground_points.clear();
    } 
}

void Parsing_lidar_data(const char* buffer) {
    if(switch_UDP_communication) {
        for(int i = 0; i < PacketSize; i += DataBlockSize) {
            //index 0, 1 = flag
            uint8_t byte1 = buffer[i + 2]; //get value
            uint8_t byte2 = buffer[i + 3]; //
            uint16_t fa_byte = 0;
            fa_byte = fa_byte | byte2; //reverse the bytes
            fa_byte = fa_byte << 8;    //
            fa_byte = fa_byte | byte1; //

            double azimuth = fa_byte * 0.01; //divide by 100 (manual)

            int laser_id = 0;
            for(int j = 0; j < ChannelDataBlockSize; j += ChannelDataSize) {
                //distance
                uint8_t byte1 = buffer[i + 4]; //get value
                uint8_t byte2 = buffer[i + 5]; //
                uint16_t fa_byte = 0;
                fa_byte = fa_byte | byte2; //reverse the bytes
                fa_byte = fa_byte << 8;    //
                fa_byte = fa_byte | byte1; //
                double distance = fa_byte * 0.002; //multiply by 2.0mm (manual)

                //reflectivity
                uint8_t reflectivity = buffer[i + 6]; //get value

                packet_points.push_back(cal_point_UDP(distance, azimuth, reflectivity, laser_id));

                laser_id++;
            }
        }
    }
    Ground_extraction();
    packet_points.clear();
}

void Ground_extraction() {
    for(int i = 0; i < packet_points.size(); i++) {
        if(packet_points[i].dist != 0) {
            raw_points.push_back(packet_points[i]);
        }
    }

    for(int i = 0; i < packet_points.size(); i++) {
        if(i % 32 == 14 || i % 32 == 15) continue; // no next point

        double dist_xy = cal_dist_21(packet_points[i+2].x, packet_points[i].x, packet_points[i+2].y, packet_points[i].y);
        double dist_z = abs(packet_points[i+2].z - packet_points[i].z);
        double slope = atan2(dist_z, dist_xy);

        if(slope < GE_slope && (packet_points[i].z < GE_Z || packet_points[i + 2].z < GE_Z)) {
            packet_points[i].is_ground = true;
            packet_points[i + 2].is_ground = true;
        }
        else if((packet_points[i].laser_id == 0 || packet_points[i].laser_id == 16) && dist_xy < 0.5 && dist_xy > 0.1) {
            packet_points[i].is_ground = true;
        }
    }

    for(int i = 0; i < packet_points.size(); i++) {
        if(packet_points[i].is_ground == true && packet_points[i].dist != 0) ground_points.push_back(packet_points[i]);
        else if(packet_points[i].is_ground == false && packet_points[i].dist != 0) other_points.push_back(packet_points[i]);
    }

    Publish_UDP_data();
}