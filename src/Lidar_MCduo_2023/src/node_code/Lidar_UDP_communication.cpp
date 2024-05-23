#include <Lidar_MCduo_2023/Lidar_declare.h>

#define BUFFER_SIZE 1206

ros::Publisher pub_ref;

void udp_process(const Lidar_MCduo_2023::totalInfoConstPtr& signal) {
    lidar_state = signal->State;
    switch_UDP_communication = (signal->State == 1 || signal->State == 2 || signal->State == 3 || signal->State == 4 || signal->State == 8 || signal->State == 9 || signal->State == -1) ? true : false;
    
    if(signal->State == 4) {
        GE_Z = GE_Z_4;
    }
    else {
        GE_Z = GE_Z_re;
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "UDP_communication");
	ros::NodeHandle nh;

    nh.getParam("/UDP_node/UDP_IP", UDP_IP);
    nh.getParam("/UDP_node/Port_Num", Port_Num);
    nh.getParam("/UDP_node/GE_Z", GE_Z);
    nh.getParam("/UDP_node/GE_Z_4", GE_Z_4);
    nh.getParam("/UDP_node/GE_Z_re", GE_Z_re);
    nh.getParam("/UDP_node/GE_slope", GE_slope);

    int socket_fd;
    struct sockaddr_in server_address, client_address;
    char buffer[BUFFER_SIZE];
    const char* IP = UDP_IP.c_str();

    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd == -1) {
        cerr << "Failed to create socket." << endl;
        return 1;
    }

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = inet_addr(IP);
    server_address.sin_port = htons(Port_Num);

    if (bind(socket_fd, (struct sockaddr*)&server_address, sizeof(server_address)) == -1) {
        cerr << "Failed to bind socket." << endl;
        return 1;
    }

    RT::start();
    ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::totalInfo> ("/Totalcom", 1, udp_process);
    pub_rawdata = nh.advertise<sensor_msgs::PointCloud2> ("/0_UDP_rawdata", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2> ("/0_UDP_ground", 1);
    pub_others = nh.advertise<sensor_msgs::PointCloud2> ("/0_UDP_others", 1);
    pub_ground_to_cam = nh.advertise<Lidar_MCduo_2023::ground_msg_arr> ("/Ground_Data", 1);

    while (ros::ok() && !shouldExit) {
        ros::spinOnce();
        socklen_t addr_len = sizeof(client_address);
        ssize_t num_bytes = recvfrom(socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_address, &addr_len);

        if (num_bytes == BUFFER_SIZE) {
            Parsing_lidar_data(buffer);
        }
    }
    close(socket_fd);

    return 0;
}