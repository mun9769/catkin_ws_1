#include <Lidar_MCduo_2023/Lidar_declare.h>

using namespace std;

string send_msg_minmax(float xMin,float xMax, float yMin, float yMax){
    string tmp_xMax = (xMax < 0) ? to_string(((int)(xMax * -100) / 2) * 2 + 1) : to_string(((int)(xMax * 100) / 2) * 2);
    string tmp_xMin = (xMin < 0) ? to_string(((int)(xMin * -100) / 2) * 2 + 1) : to_string(((int)(xMin * 100) / 2) * 2);
    string tmp_yMax = (yMax < 0) ? to_string(((int)(yMax * -100) / 2) * 2 + 1) : to_string(((int)(yMax * 100) / 2) * 2);
    string tmp_yMin = (yMin < 0) ? to_string(((int)(yMin * -100) / 2) * 2 + 1) : to_string(((int)(yMin * 100) / 2) * 2);
    string tmp, st;

    for(int i = 4; i > tmp_xMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = tmp + tmp_xMin;
    tmp.clear();
    for(int i = 4; i > tmp_xMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_xMax;
    tmp.clear();
    for(int i = 4; i > tmp_yMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMin;
    tmp.clear();
    for(int i = 4; i > tmp_yMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMax;
    return st;
}

void object_msg_process(const vector<struct objectInfo>& objs){
    Lidar_MCduo_2023::object_msg_arr msg;
    Lidar_MCduo_2023::object_msg msgCpnt;

    vector<Lidar_MCduo_2023::object_msg> msgConvertVector;
    msg.objc = objs.size();
    for (objectInfo obj : objs){
        msgCpnt.x = obj.x;
        msgCpnt.y = obj.y;
        msgCpnt.z = obj.z;
        msgCpnt.xMin = obj.xMin;
        msgCpnt.yMin = obj.yMin;
        msgCpnt.zMin = obj.zMin;
        msgCpnt.xMax = obj.xMax;
        msgCpnt.yMax = obj.yMax;
        msgCpnt.zMax = obj.zMax;
        msgCpnt.classes = obj.classes;
        msgCpnt.idx = obj.idx;
        msgConvertVector.push_back(msgCpnt);
    }
    msg.object_msg_arr = msgConvertVector;

    sort(msg.object_msg_arr.begin(), msg.object_msg_arr.end(),
          [](const Lidar_MCduo_2023::object_msg& obj1, const Lidar_MCduo_2023::object_msg& obj2) {
            float dist_obj1 = cal_dist(obj1.x, obj1.y);
            float dist_obj2 = cal_dist(obj2.x, obj2.y);
              return dist_obj1 < dist_obj2;
          });
    
    pub_obj.publish(msg);

    Lidar_MCduo_2023::objsInfo msg3;
    Lidar_MCduo_2023::objInfo msgCpnt3;
    Lidar_MCduo_2023::coeff coeff_tmp;

    vector<Lidar_MCduo_2023::objInfo> msgConvertVector3;
    msg3.objNum = objs.size();
    int i = 1;
    for (objectInfo obj : objs){
        msgCpnt3.classes = i;
        msgCpnt3.posX = obj.x;
        msgCpnt3.posY = obj.y;
        msgCpnt3.posZ = obj.z;
        msgCpnt3.size = (obj.xMax-obj.xMin)*(obj.yMax-obj.yMin);
        msgCpnt3.distance = cal_dist(obj.x, obj.y);
        msgCpnt3.x_max = obj.xMax;
        msgCpnt3.x_min = obj.xMin;
        msgCpnt3.y_max = obj.yMax;
        msgCpnt3.y_min = obj.yMin;
        msgCpnt3.z_max = obj.zMax;
        msgConvertVector3.push_back(msgCpnt3);
        i+=1;
    }
    for(int i = 0; i < ransac_coeff.size(); i++) {
        msg3.coeff.push_back(ransac_coeff[i]);
    }
    msg3.data = msgConvertVector3;
    
    sort(msg3.data.begin(), msg3.data.end(),
          [](const Lidar_MCduo_2023::objInfo& obj1, const Lidar_MCduo_2023::objInfo& obj2) {
              return obj1.distance < obj2.distance;
          });

    pub_LC.publish(msg3);
}