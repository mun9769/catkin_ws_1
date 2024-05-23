#include <Lidar_MCduo_2023/Lidar_declare.h>

#define SCALE_K_city_main 3.5252762 //convert to real object size(m) to pixel size (0.28366572 pixel = 1m)

bool switch_visual_map;
string path_location;
string path_location1;
string path_location2;
string path_location3;
string path_location4;
string image_location;
cv::Mat image;

vector<float> map_x;
vector<float> map_y;

vector<float> obj_x;
vector<float> obj_y;

float car_x, car_y;

void ENU_to_K_city_main_map(float x, float y) {
    float X = -y * SCALE_K_city_main + 1664;
    float Y = -x * SCALE_K_city_main + 947;
    map_x.push_back(X);
    map_y.push_back(Y);
}

void ENU_to_K_city_main_obj(float x, float y) {
    float X = -y * SCALE_K_city_main + 1664;
    float Y = -x * SCALE_K_city_main + 947;
    obj_x.push_back(X);
    obj_y.push_back(Y);
}

void visual_process(const Lidar_MCduo_2023::obj_tracking_msgConstPtr& trk_msg) {
    car_x = -trk_msg->car_y * SCALE_K_city_main + 1664;
    car_y = -trk_msg->car_x * SCALE_K_city_main + 947;

    // for(int i = 0; i < trk_msg->obj_num; i++) {
    //     ENU_to_K_city_main_obj(trk_msg->data[i].trk_x, trk_msg->data[i].trk_y);
    // }

    cv::Mat output_image;
    image.copyTo(output_image);
    cv::circle(output_image, cv::Point(car_x, car_y), 1, cv::Scalar(255,0,0), -1); //3.5
    for(int i = 0; i < obj_x.size(); i++) {
        cv::circle(output_image, cv::Point(obj_x[i], obj_y[i]), 1, cv::Scalar(0,0,255), -1);
        // cout << "x : " << obj_x[i] << endl;
    }

    double thetaRadian = 0; // + -> CCW
    double scaleFactor = 20;
    double img_car_X = car_x, img_car_Y = car_y;
    cv::Point center = cv::Point(output_image.size().width / 2, output_image.size().height / 2);

    cv::Mat rotationMat = cv::getRotationMatrix2D(center, 0, scaleFactor);

    cv::Mat transformMat = (cv::Mat_<double>(3,3) << 
        1, 0, output_image.size().width / 2 - img_car_X,
        0, 1, output_image.size().height / 2 - img_car_Y,
        0, 0, 1
    );

    cv::Mat resPerspectiveMat = rotationMat * transformMat;

    // cv::warpPerspective(output_image, output_image, resPerspectiveMat, output_image.size());
    cv::warpAffine(output_image, output_image, resPerspectiveMat, output_image.size());

    cv::imshow("Lidar Visual", output_image);
    cv::waitKey(2);
}

void clu_process(const Lidar_MCduo_2023::object_msg_arrConstPtr& clu_msg) {
    vector<float> tmp_x;
    vector<float> tmp_y;

    for(int i = 0; i < clu_msg->object_msg_arr.size(); i++) {
        float X = -clu_msg->object_msg_arr[i].y * SCALE_K_city_main + 1664;
        float Y = -clu_msg->object_msg_arr[i].x * SCALE_K_city_main + 947;
  
        tmp_x.push_back(X);
        tmp_y.push_back(Y);
    }

    obj_x = tmp_x;
    obj_y = tmp_y;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Visual");         //node name 
	ros::NodeHandle nh;                         //nodehandle

    nh.getParam("/visual_map_node/switch_visual_map", switch_visual_map);
    nh.getParam("/visual_map_node/path_location", path_location);
    nh.getParam("/visual_map_node/path_location1", path_location1);
    nh.getParam("/visual_map_node/path_location2", path_location2);
    nh.getParam("/visual_map_node/path_location3", path_location3);
    nh.getParam("/visual_map_node/path_location4", path_location4);
    nh.getParam("/visual_map_node/image_location", image_location);

    ifstream ifs;
    ifstream ifs1;
    ifstream ifs2;
    ifstream ifs3;
    ifstream ifs4;

    ifs.open(path_location);
    for(int i = 0; i < 6701; i++) {
        float tmp_x;
        ifs >> tmp_x;
        float tmp_y;
        ifs >> tmp_y;
        ENU_to_K_city_main_map(tmp_x, tmp_y);
    }

    image = cv::imread(image_location, cv::IMREAD_COLOR);

    if(image.empty()) {
        cerr << "No image" << endl;
        return -1;
    }

    for(int i = 0; i < map_x.size() - 1; i++) {
        cv::line(image, cv::Point(map_x[i], map_y[i]), cv::Point(map_x[i+1], map_y[i+1]), cv::Scalar::all(255), 1, 8, 0);
    }

    map_x.clear();
    map_y.clear();

    ifs1.open(path_location1);
    for(int i = 0; i < 322; i++) {
        float tmp_x;
        ifs1 >> tmp_x;
        float tmp_y;
        ifs1 >> tmp_y;
        ENU_to_K_city_main_map(tmp_x, tmp_y);
    }

    for(int i = 0; i < map_x.size() - 1; i++) {
        cv::line(image, cv::Point(map_x[i], map_y[i]), cv::Point(map_x[i+1], map_y[i+1]), cv::Scalar(0,0,255), 1, 8, 0);
    }
    
    map_x.clear();
    map_y.clear();

    ifs2.open(path_location2);
    for(int i = 0; i < 335; i++) {
        float tmp_x;
        ifs2 >> tmp_x;
        float tmp_y;
        ifs2 >> tmp_y;
        ENU_to_K_city_main_map(tmp_x, tmp_y);
    }
    
    for(int i = 0; i < map_x.size() - 1; i++) {
        cv::line(image, cv::Point(map_x[i], map_y[i]), cv::Point(map_x[i+1], map_y[i+1]), cv::Scalar(255,0,0), 1, 8, 0);
    }

    map_x.clear();
    map_y.clear();

    ifs3.open(path_location3);
    for(int i = 0; i < 374; i++) {
        float tmp_x;
        ifs3 >> tmp_x;
        float tmp_y;
        ifs3 >> tmp_y;
        ENU_to_K_city_main_map(tmp_x, tmp_y);
    }
    
    for(int i = 0; i < map_x.size() - 1; i++) {
        cv::line(image, cv::Point(map_x[i], map_y[i]), cv::Point(map_x[i+1], map_y[i+1]), cv::Scalar(0,255,0), 1, 8, 0);
    }

    map_x.clear();
    map_y.clear();

    ifs4.open(path_location4);
    for(int i = 0; i < 374; i++) {
        float tmp_x;
        ifs4 >> tmp_x;
        float tmp_y;
        ifs4 >> tmp_y;
        ENU_to_K_city_main_map(tmp_x, tmp_y);
    }
    
    for(int i = 0; i < map_x.size(); i++) {
        cv::circle(image, cv::Point(map_x[i], map_y[i]), 1, cv::Scalar(0,100,100), -1);
    }

    // while(ros::ok()) {
    //     cv::imshow("Lidar Visual", image);
    //     cv::waitKey(2);
    // }

    ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::obj_tracking_msg> ("/LiDAR_tracking_global", 1, visual_process);
    ros::Subscriber sub_clu = nh.subscribe<Lidar_MCduo_2023::object_msg_arr> ("/LiDAR_tracking_global_map", 1, clu_process);

    ros::spin();
}

// #include <Lidar_MCduo_2023/Lidar_declare.h>

// #define SCALE_K_city_main 3.5252762 //convert to real object size(m) to pixel size (0.28366572 pixel = 1m)

// int main(int argc, char** argv){
// 	ros::init(argc, argv, "Visual");         //node name 
// 	ros::NodeHandle nh;                         //nodehandle

//     string input;
//     ifstream ifs;
//     double tmp_x;
//     vector<double> map_x;
//     vector<double> map_y;

//     ifs.open("/home/min/catkin_ws/src/Lidar_MCduo_2023/include/Lidar_MCduo_2023/Map_data/kcity Main Path.txt");
//         for(int i = 0; i < 6701; i++) {
//             double tmp_x;
//             ifs >> tmp_x;
//             map_x.push_back(tmp_x);
//             double tmp_y;
//             ifs >> tmp_y;
//             map_y.push_back(tmp_y);
//         }

//     string path = "/home/min/catkin_ws/src/Lidar_MCduo_2023/include/Lidar_MCduo_2023/Map_data/K_city_main(height 597.4)(downsized 2).png";
//     cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);

//     if(image.empty()) {
//         cerr << "No image" << endl;
//         return -1;
//     }

//     for(int i = 0; i < map_x.size() - 1; i++) {
//         cv::line(image, cv::Point(-map_y[i] * SCALE_K_city_main + 1664, -map_x[i] * SCALE_K_city_main + 947), cv::Point(-map_y[i+1] * SCALE_K_city_main + 1664, -map_x[i+1] * SCALE_K_city_main + 947), cv::Scalar::all(255), 1, 8, 0);
//     }

//     while(ros::ok()) {
//         // for(int i = 0; i < map_x.size() - 1; i++) {
//         //     cv::line(image, cv::Point(-map_y[i] * SCALE + 1664, -map_x[i] * SCALE + 947), cv::Point(-map_y[i+1] * SCALE + 1664, -map_x[i+1] * SCALE + 947), cv::Scalar::all(255), 1, 8, 0);
//         // }
//         cv::imshow("Lidar Visual", image);
//         cv::waitKey(2);
//     }

//     ros::spin();
// }