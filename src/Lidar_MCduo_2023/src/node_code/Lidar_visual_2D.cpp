#include <Lidar_MCduo_2023/Lidar_declare.h>

string img;

#define METRETOPIXEL 30.5 //convert to real object size(m) to pixel size (30.5 pixel = 1m)

void flat_visual_process(const Lidar_MCduo_2023::object_msg_arrConstPtr& objs){
    //RT::start();

    cv::Mat src = cv::imread(img, cv::IMREAD_COLOR);

    //car
    cv::rectangle(src, cv::Rect(236,535,20,50), cv::Scalar(150,100,0), 20, 8, 0); //236 = 246 - 20/2

    //object
    for (const Lidar_MCduo_2023::object_msg& msgobj : objs->object_msg_arr){
        float uCor = 246 - msgobj.y * METRETOPIXEL; //246 is origin 'u' point
        float vCor = 552 - msgobj.x * METRETOPIXEL; //552 is origin 'v' point
        float uSiz = (msgobj.yMax - msgobj.yMin) * METRETOPIXEL;
        float vSiz = (msgobj.xMax - msgobj.xMin) * METRETOPIXEL;
        
        cv::rectangle(src, cv::Rect(uCor - uSiz/2, vCor - vSiz/2, uSiz, vSiz), cv::Scalar(0,255,205), -1, 8, 0); //obj size
        cv::circle   (src, cv::Point(uCor,vCor), 3.5, cv::Scalar(0,0,255), -1); //center point
    }

    cv::imshow("Lidar Detected objs", src);
    cv::waitKey(2);

    //RT::end_cal("visual");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Visual_2D");         //node name 
	ros::NodeHandle nh;                         //nodehandle

    nh.getParam("/visual_2D_node/switch_visual_2D", switch_visual_2D);
    nh.getParam("/visual_2D_node/location", img);
    if (!switch_visual_2D) exit(0); //exit node if switch is 0

	ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::object_msg_arr> ("/4_Clustering_OMA", 1, flat_visual_process);

    ros::spin();
}