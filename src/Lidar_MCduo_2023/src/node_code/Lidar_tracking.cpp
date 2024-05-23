#include <Lidar_MCduo_2023/Lidar_declare.h>

PCXYZI final_data;

auto EndTime = system_clock::now();
auto StartTime = system_clock::now();
std::chrono::duration<float> ElapsedSeconds = EndTime - StartTime;

float dt = static_cast<float>(ElapsedSeconds.count());

///////////////////////////////////////////////////////////////////////////////////////////////
void UpdatePreviousPosition(int I)
{
   TrackingObj[I].Prev_X = TrackingObj[I].Update_X;
   TrackingObj[I].Prev_Y = TrackingObj[I].Update_Y;
}

void UpdateTrackingObj(int i, int Update_idx, float Best_X, float Best_Y)
{
  TrackingObj[i].Update_X = Best_X;
  TrackingObj[i].Update_Y = Best_Y;
  TrackingObj[i].UpdateCnt = 1;
  TrackingObj[i].DeadCnt = 0;
  
  ObjCandidate[Update_idx].NewFlag = 0;
   
}

void FindNearObj()
{
  int i, k, Update_flag, Update_idx;
  float dist;
  float Min_dist, Best_X, Best_Y;
  float Best_Prev_X, Best_Prev_Y; 
  for (i = 0; i < TrackingObj.size(); i++)
  {

    float Dist;
    Update_flag = 0;
    Update_idx = 0;

    Min_dist = 9999;
    Best_X = 0;
    Best_Y = 0;

    for(k = 0; k < ObjCandidate.size(); k++)
    {
      if(ObjCandidate[k].NewFlag == 1){

        Dist = Distance2D(TrackingObj[i].Target_X,TrackingObj[i].Target_Y,
                          ObjCandidate[k].curX, ObjCandidate[k].curY);
        if (Dist <= 0.7)
        {
          if(Dist <= Min_dist )
          {
            Min_dist = Dist;
            Best_X = ObjCandidate[k].curX;
            Best_Y = ObjCandidate[k].curY;

            Update_idx = k;
            Update_flag = 1;
          }
        }
      }
    }

    if(Update_flag ==1)
    {
      UpdateTrackingObj(i, Update_idx, Best_X, Best_Y);
      TrackingObj[i].Target_X = Best_X;
      TrackingObj[i].Target_Y = Best_Y;

      for (k = 0; k < ObjCandidate.size(); k++)
      {
        if (ObjCandidate[k].NewFlag == 1)
        {
          Dist = Distance2D(ObjCandidate[k].curX, ObjCandidate[k].curY, Best_X, Best_Y);
          if (Dist <= 1)
            ObjCandidate[k].NewFlag = 0;
        }
      }
    }
    else
    {
      TrackingObj[i].UpdateCnt = 0;
      TrackingObj[i].DeadCnt++;
      if (TrackingObj[i].DeadCnt > 30)
        DeleteTrackingObj(i);
    }
  }

}

void Initialization(int i) 
{
  float Vx, Vy, Update_X, Update_Y;

  TrackingObj[i].EndTime = system_clock::now();
  std::chrono::duration<double> elapsed_seconds = TrackingObj[i].EndTime - TrackingObj[i].StartTime;
  TrackingObj[i].ar_dt = static_cast<float>(elapsed_seconds.count());
  TrackingObj[i].StartTime = system_clock::now();

  TrackingObj[i].idx = cnt;
  cnt++;
  cout << "idx : " << TrackingObj[i].idx << endl;

  TrackingObj[i].UpdateCnt = 0;
  TrackingObj[i].InitFlag = 0;
  TrackingObj[i].F_ = MatrixXd(4, 4);
  TrackingObj[i].X_ = MatrixXd(4, 1);
  TrackingObj[i].Prev_X_ = MatrixXd(4, 1);
  TrackingObj[i].P_ = MatrixXd(4, 4);
  TrackingObj[i].H_ = MatrixXd(4, 4);
  TrackingObj[i].Z_ = MatrixXd(4, 1);
  TrackingObj[i].R_ = MatrixXd(4, 4);
  TrackingObj[i].Q_ = MatrixXd(4, 4);
  TrackingObj[i].K_ = MatrixXd(4, 4);
  TrackingObj[i].F_.setZero();
  TrackingObj[i].X_.setZero();
  TrackingObj[i].Prev_X_.setZero();
  TrackingObj[i].P_.setZero();
  TrackingObj[i].H_.setZero();
  TrackingObj[i].Z_.setZero();
  TrackingObj[i].R_.setZero();
  TrackingObj[i].Q_.setZero();
  TrackingObj[i].K_.setZero();

  TrackingObj[i].P_(0, 0) = 1;
  TrackingObj[i].P_(1, 1) = 1;
  TrackingObj[i].P_(2, 2) = 2;
  TrackingObj[i].P_(3, 3) = 2;
  
  TrackingObj[i].H_(0, 0) = 1;
  TrackingObj[i].H_(1, 1) = 1;
  TrackingObj[i].H_(2, 2) = 1;
  TrackingObj[i].H_(3, 3) = 1;

  TrackingObj[i].Q_(0, 0) = 10000;
  TrackingObj[i].Q_(1, 1) = 10000;
  TrackingObj[i].Q_(2, 2) = 1000;
  TrackingObj[i].Q_(3, 3) = 1000;

  TrackingObj[i].R_(0, 0) = 0.001;
  TrackingObj[i].R_(1, 1) = 0.001;
  TrackingObj[i].R_(2, 2) = 0.01;
  TrackingObj[i].R_(3, 3) = 0.01;

  Update_X = TrackingObj[i].Update_X;
  Update_Y = TrackingObj[i].Update_Y;

  Vx = TrackingObj[i].Init_Vx;
  Vy = TrackingObj[i].Init_Vy;

  TrackingObj[i].X_ << Update_X,
                       Update_Y,
                       Vx,
                       Vy;

  UpdatePreviousPosition(i);
}

void DeleteTrackingObj(int i)
{ 
  TrackingObj.erase(TrackingObj.begin() + i);
  // TrackingObj.ObjValid.erase(TrackingObj.ObjValid.begin() + i);
  // TrackingObj.InitFlag.erase(TrackingObj.InitFlag.begin() + i);
  // TrackingObj.UpdateCnt.erase(TrackingObj.UpdateCnt.begin() + i);
  // TrackingObj.DeadCnt.erase(TrackingObj.DeadCnt.begin() + i);
  // TrackingObj.StartTime.erase(TrackingObj.StartTime.begin() + i);
  // TrackingObj.EndTime.erase(TrackingObj.EndTime.begin() + i);
  // TrackingObj.ar_dt.erase(TrackingObj.ar_dt.begin() + i);
  // TrackingObj.Prev_X.erase(TrackingObj.Prev_X.begin() + i);
  // TrackingObj.Prev_Y.erase(TrackingObj.Prev_Y.begin() + i);
  // TrackingObj.Init_Vx.erase(TrackingObj.Init_Vx.begin() + i);
  // TrackingObj.Init_Vy.erase(TrackingObj.Init_Vy.begin() + i);

}

void FindNew() 
{
  // EndTime = system_clock::now();
  // std::chrono::duration<float> ElapsedSeconds = EndTime - StartTime;

  // float dt = static_cast<float>(ElapsedSeconds.count());

  int i, j, k, Update_flag, Update_idx;
  float Dist, Min_dist, Best_X, Best_Y;
  float Best_Prev_X, Best_Prev_Y;

  for (i = 0; i < ObjCandidate.size(); i++)
  {

    if(ObjCandidate[i].NewFlag == 1) // FindNearObj 중복 검사
    {

      Update_flag = 0;
      Update_idx = 0;
      Min_dist = 999;
      Best_X = 0;
      Best_Y = 0;

      for (k = 0; k < ObjCandidate.size(); k++)
      {
        if (ObjCandidate_prev[k].NewFlag == 1) // FindNearObj 중복 검사
        {
          Dist = Distance2D(ObjCandidate[i].curX, ObjCandidate[i].curY,
                            ObjCandidate_prev[i].curX, ObjCandidate_prev[i].curY);

          if (Dist <= 0.7)
          {
            if (Dist <= Min_dist)
            {
              Min_dist = Dist;
              Best_X = ObjCandidate[i].curX;
              Best_Y = ObjCandidate[i].curY;

              Update_flag = 1;
              Update_idx = i;

              Best_Prev_X = ObjCandidate_prev[k].curX;
              Best_Prev_Y = ObjCandidate_prev[k].curY;
            }
          }
        }
      }

      if (Update_flag == 1)
      {
        // for (j = 0; j < ObjCandidate.size(); j++)
        // {
        //   PushTrackingObj(j, Update_idx);
        //   ObjCandidate[Update_idx].curValid = 0;

        //   TrackingObj[j].Init_Vx = (Best_X - Best_Prev_X) / 0.1;
        //   TrackingObj[j].Init_Vy = (Best_Y - Best_Prev_Y) / 0.1;
        //   TrackingObj[j].Target_X = Best_X;
        //   TrackingObj[j].Target_Y = Best_Y;            
        // }
        TRACKING_OBJ tmp;
        tmp.Update_X = Best_X;
        tmp.Update_Y = Best_Y;
        tmp.Target_X = Best_X;
        tmp.Target_Y = Best_Y;
        tmp.Init_Vx = (Best_X - Best_Prev_X) / 0.1 ;
        tmp.Init_Vy = (Best_Y - Best_Prev_Y) / 0.1 ;
        tmp.InitFlag = 1;
        TrackingObj.push_back(tmp);
      }
    }
  }
  ObjCandidate_prev = ObjCandidate;
  //StartTime = system_clock::now();
}

void Prediction(int i)
{
  TrackingObj[i].F_ << 1, 0, dt, 0,
                       0, 1, 0, dt,
                       0, 0, cos(-dyaw), -sin(-dyaw),
                       0, 0, sin(-dyaw), cos(-dyaw);

  TrackingObj[i].Prev_X_ = TrackingObj[i].X_;
  TrackingObj[i].X_ = TrackingObj[i].F_ * TrackingObj[i].X_;
  TrackingObj[i].P_ = TrackingObj[i].F_ *
                      TrackingObj[i].P_ *
                      TrackingObj[i].F_.transpose() +
                      TrackingObj[i].Q_;
}

void Update(int i) 
{
  float UpdateVx, UpdateVy;

  TrackingObj[i].EndTime = system_clock::now();
  auto st_TempTime = TrackingObj[i].StartTime;

  std::chrono::duration<double> elapsed_seconds = TrackingObj[i].EndTime - TrackingObj[i].StartTime;
  TrackingObj[i].ar_dt = static_cast<float>(elapsed_seconds.count());
  TrackingObj[i].StartTime = system_clock::now();

  UpdateVx = (TrackingObj[i].Update_X - TrackingObj[i].Prev_X) / TrackingObj[i].ar_dt;
  UpdateVy = (TrackingObj[i].Update_Y - TrackingObj[i].Prev_Y) / TrackingObj[i].ar_dt;


  TrackingObj[i].H_ <<  1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;

  TrackingObj[i].Z_ << 
                       TrackingObj[i].Update_X,
                       TrackingObj[i].Update_Y,
                       UpdateVx,
                       UpdateVy;

  TrackingObj[i].K_ =
                      TrackingObj[i].P_ *
                      TrackingObj[i].H_.transpose() *
                      (TrackingObj[i].H_ *
                      TrackingObj[i].P_ *
                      TrackingObj[i].H_.transpose() +
                      TrackingObj[i].R_).inverse();

  TrackingObj[i].P_ =
                      TrackingObj[i].P_ -
                      TrackingObj[i].K_ *
                      TrackingObj[i].H_ *
                      TrackingObj[i].P_;

  TrackingObj[i].X_ =
                      TrackingObj[i].X_ +
                      TrackingObj[i].K_ *
                      (TrackingObj[i].Z_
                      - TrackingObj[i].H_ *
                      TrackingObj[i].X_);


  UpdatePreviousPosition(i);
}
////////////////////////////////////////////////////////////////////////////////////////////

void tracking() 
{
  int i, j, k;
  float Distance, PredictVel;

  // 시간 측정 
  EndTime = system_clock::now();
  std::chrono::duration<float> ElapsedSeconds = EndTime - StartTime;
  StartTime = system_clock::now();
  dt = static_cast<float>(ElapsedSeconds.count());

  yaw_end = yaw_now;
  dyaw = yaw_end - yaw_start;
  yaw_start = yaw_now;

  if (ObjCandidate.size() > 0)
  {

    if (TrackingObj.size() != 0) 
    {

      FindNearObj();

    }

    FindNew();

  }
  else
  {
    for (i = 0; i < TrackingObj.size(); i++)
    {
      // if (TrackingObj[i].ObjValid == 1)
      // {
        TrackingObj[i].UpdateCnt = 0;
        TrackingObj[i].DeadCnt++;

        if (TrackingObj[i].DeadCnt > 30) 
        {
          // 추적 객체 삭제
          DeleteTrackingObj(i);
          i--;
        }

      // }
    }
  }

  for (i = 0; i < TrackingObj.size(); i++)
  {
    if (TrackingObj[i].InitFlag == 1) 
      Initialization(i);
  }

  for (i = 0; i < TrackingObj.size(); i++)
  {
    if (TrackingObj[i].InitFlag == 0)
    {
      Prediction(i);
      Distance = Distance2D((float)TrackingObj[i].Prev_X_(0),
                            (float)TrackingObj[i].Prev_X_(1),
                            (float)TrackingObj[i].X_(0),
                            (float)TrackingObj[i].X_(1));

      PredictVel = Distance2D(0, 0, 
                    (float)TrackingObj[i].X_(2),
                    (float)TrackingObj[i].X_(3));
      
      if (Distance > 6)
        DeleteTrackingObj(i);
      else if (PredictVel * 3.6 > 150)
        DeleteTrackingObj(i);
    }
  }

  for (i = 0; i < TrackingObj.size(); i++)
  {
    if (TrackingObj[i].InitFlag == 0 && TrackingObj[i].UpdateCnt == 1)
    {
      Update(i);
    }
  }
}

void TransDataSet(const Lidar_MCduo_2023::object_msg_arrConstPtr& objects)
{
  for (int i = 0; i < objects->objc; i++)
  {
    CANDIDATE tmp;
    tmp.curX = objects->object_msg_arr[i].x;
    tmp.curY = objects->object_msg_arr[i].y;
    tmp.NewFlag = 1;
    ObjCandidate.push_back(tmp);
  }
  if(ObjCandidate_prev.size() == 0) {
    ObjCandidate_prev = ObjCandidate;
  }
}

void tracking_process(const Lidar_MCduo_2023::object_msg_arrConstPtr& objects) 
{

  TransDataSet(objects);

  tracking();

  // Lidar_MCduo_2023::obj_tracking_msg trkmsg;
  // trkmsg.obj_num = TrackingObj.size();
  // for(int i = 0; i < TrackingObj.size(); i++) {
  //   Lidar_MCduo_2023::obj_tracking tmp;
  //   tmp.trk_idx = TrackingObj[i].idx;
  //   tmp.trk_x = TrackingObj[i].X_(0);
  //   tmp.trk_y = TrackingObj[i].X_(1);
  //   tmp.trk_vx = TrackingObj[i].X_(2);
  //   tmp.trk_vy = TrackingObj[i].X_(3);
  //   trkmsg.data.push_back(tmp);
  // }
  // pub_tracking_msg.publish(trkmsg);

  for(int i = 0; i < TrackingObj.size(); i++) 
  {
    pcl::PointXYZI tmp;
    tmp.x = TrackingObj[i].X_(0);
    tmp.y = TrackingObj[i].X_(1);
    tmp.z = 0;
    tmp.intensity = 0;
    final_data.push_back(tmp);
  }
  sensor_msgs::PointCloud2 output;
  pub_process(final_data, output);
  pub_tracking_pcl.publish(output);
  final_data.clear();

  ObjCandidate.clear();

  vector<objectInfo> objs;
  objectInfo tmp_objInfo;
  for(int i = 0; i < TrackingObj.size(); i++) {
    tmp_objInfo.x = TrackingObj[i].X_(0);
    tmp_objInfo.y = TrackingObj[i].X_(1);
    tmp_objInfo.z = 0;
  }

  Lidar_MCduo_2023::objsInfo msg3;
  Lidar_MCduo_2023::objInfo msgCpnt3;

  vector<Lidar_MCduo_2023::objInfo> msgConvertVector3;
  msg3.objNum = TrackingObj.size();
  int i = 1;
  for (objectInfo obj : objs){
      msgCpnt3.classes = i;
      msgCpnt3.posX = obj.x;
      msgCpnt3.posY = obj.y;
      msgCpnt3.posZ = obj.z;
      msgCpnt3.size = (obj.xMax-obj.xMin)*(obj.yMax-obj.yMin);
      msgCpnt3.distance = cal_dist(obj.x, obj.y);
      msgConvertVector3.push_back(msgCpnt3);
      i+=1;
  }
  msg3.data = msgConvertVector3;
  
  sort(msg3.data.begin(), msg3.data.end(),
        [](const Lidar_MCduo_2023::objInfo& obj1, const Lidar_MCduo_2023::objInfo& obj2) {
            return obj1.distance < obj2.distance;
        });

  pub_tracking_msg.publish(msg3);
}

void imu_process(const Lidar_MCduo_2023::GNSSInfoConstPtr& imu_data) {
  yaw_now = imu_data->heading;
}

int main(int argc, char** argv) {
  TrackingObj.resize(0);
  ObjCandidate_prev.resize(0);

  ros::init(argc, argv, "lidar_tracking_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::object_msg_arr> ("/4_Clustering_OMA", 1, tracking_process);
  ros::Subscriber sub_imu = nh.subscribe<Lidar_MCduo_2023::GNSSInfo> ("/GNSScom", 1, imu_process);
  pub_tracking_pcl = nh.advertise<sensor_msgs::PointCloud2> ("/5_Tracking_PCL2", 1);
  // pub_tracking_msg = nh.advertise<Lidar_MCduo_2023::obj_tracking_msg> ("/LiDAR_tracking", 1);
  pub_tracking_msg = nh.advertise<Lidar_MCduo_2023::objsInfo> ("/Trackcom", 1);

  ros::spin();
}