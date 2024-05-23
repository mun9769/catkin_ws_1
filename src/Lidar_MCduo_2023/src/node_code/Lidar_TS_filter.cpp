#include <Lidar_MCduo_2023/Lidar_declare.h>
#include <functional>

#define LidarZaxisPosition 0.83

inline float retSize(float a, float b) { return abs(a - b);}

void noBongYZsizeFilter (vector<Lidar_MCduo_2023::object_msg>& retMsg){
    for (vector<Lidar_MCduo_2023::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (retSize(it->zMax, it->zMin) > 1.2 ||
            retSize(it->zMax, it->zMin) < 0.2 ||
            retSize(it->yMax, it->yMin) > 0.85 ||
            retSize(it->yMax, it->yMin) < 0.4 ) it = retMsg.erase(it);
        else {
            it->classes = "plate";
            it++;
        }
    }
}

void noBongYZpositionFilter (vector<Lidar_MCduo_2023::object_msg>& retMsg){
    for (vector<Lidar_MCduo_2023::object_msg>::iterator it = retMsg.begin(); it != retMsg.end();){
        if (it->zMin < 0.5 - LidarZaxisPosition||
            it->zMax > 2.0 - LidarZaxisPosition) it = retMsg.erase(it);
        else {
            it++;
        }
    }
}

void delivery_filter_process(const Lidar_MCduo_2023::object_msg_arrConstPtr& objs){
    //RT::start();

    vector<Lidar_MCduo_2023::object_msg> retMsgNoBong = objs->object_msg_arr;

    noBongYZsizeFilter      (retMsgNoBong);
    //noBongYZpositionFilter  (retMsgNoBong);

    Lidar_MCduo_2023::object_msg_arr finMsg;
    finMsg.objc = retMsgNoBong.size();
    finMsg.object_msg_arr = retMsgNoBong;

    //cout << "TS_num : " << retMsgNoBong.size() << endl; 

    Lidar_MCduo_2023::objsInfo msg4;
    Lidar_MCduo_2023::objInfo msgCpnt4;

    vector<Lidar_MCduo_2023::objInfo> msgConvertVector4;
    msg4.objNum = retMsgNoBong.size();
    int i = 1;
    for (Lidar_MCduo_2023::object_msg obj : retMsgNoBong){
        msgCpnt4.classes = i;
        msgCpnt4.posX = obj.x;
        msgCpnt4.posY = obj.y;
        msgCpnt4.posZ = obj.z;
        msgCpnt4.size = (obj.xMax-obj.xMin)*(obj.yMax-obj.yMin);
        msgCpnt4.distance = cal_dist(obj.x, obj.y);
        msgConvertVector4.push_back(msgCpnt4);
        i+=1;
    }

    msg4.data = msgConvertVector4;
    pub_TS.publish(finMsg);

    //RT::end_cal("TS_filter");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "TS filter");         //node name 
    ros::NodeHandle nh;                         //nodehandle

    ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::object_msg_arr> ("/4_Clustering_OMA", 1, delivery_filter_process);
    pub_TS = nh.advertise<Lidar_MCduo_2023::object_msg_arr> ("/TS_to_camera", 1);

    ros::spin();
}