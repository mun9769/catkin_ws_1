#include <Lidar_MCduo_2023/Lidar_declare.h>

ros::Publisher pub;
ros::Publisher pub_rviz;

struct point {
	float X;
	float Y;
};

struct tetragon {
	point p1;
	point p2;
	point p3;
	point p4;
};

void signal_process(const Lidar_MCduo_2023::totalInfoConstPtr& signal) {
	switch_interior_points = (signal->State == 4) ? true : false;
}

float cal_x(float s, float c, float y) {
	float x = y / s - c / s;
	
	return x;
}

float cal_y(float s, float c, float x) {
	float y = s * x + c;

	return y;
}

bool is_inner_point(float rx, float ry, const vector<point>& p){
    //cnt는 점q와 오른쪽 반직선과 다각형과의 교점의 개수
    int cnt = 0;
    for(int i = 0 ; i < p.size() ; i++) {
        int j = (i + 1) % p.size();
        //점 B가 선분 (p[i], p[j])의 y좌표 사이에 있음
        if((p[i].Y > ry) != (p[j].Y > ry) ) {
            //atX는 점 B를 지나는 수평선과 선분 (p[i], p[j])의 교점
            float at_X = (p[j].X - p[i].X) * (ry-p[i].Y)/(p[j].Y-p[i].Y) + p[i].X;
            //atX가 오른쪽 반직선과의 교점이 맞으면 교점의 개수를 증가시킨다.
            if(rx < at_X) cnt++;
        }
    }
    return cnt % 2 > 0;
}

void interior_points_process(const Lidar_MCduo_2023::RANSAC_points_arrConstPtr& msg) {
	RT::start();
	vector<point> tetragon(4);

	if(msg->coeff.size() == 0) switch_interior_points = false;
	
	if(switch_interior_points) {
		if(abs(msg->coeff[0].Slope) < 1) {
			tetragon[0].X = 12;
			tetragon[0].Y = cal_y(msg->coeff[0].Slope, msg->coeff[0].Yintercept, 12);
			tetragon[1].X = -2;
			tetragon[1].Y = cal_y(msg->coeff[0].Slope, msg->coeff[0].Yintercept, 0.5);
		}
		else {
			tetragon[0].X = cal_x(msg->coeff[0].Slope, msg->coeff[0].Yintercept, -20);
			tetragon[0].Y = -20;
			tetragon[1].X = cal_x(msg->coeff[0].Slope, msg->coeff[0].Yintercept, 20);
			tetragon[1].Y = 20;
		}

		if(abs(msg->coeff[1].Slope) < 1) {
			tetragon[2].X = -2;
			tetragon[2].Y = cal_y(msg->coeff[1].Slope, msg->coeff[1].Yintercept, 0.5);
			tetragon[3].X = 12;
			tetragon[3].Y = cal_y(msg->coeff[1].Slope, msg->coeff[1].Yintercept, 12);
		}
		else {
			tetragon[2].X = cal_x(msg->coeff[1].Slope, msg->coeff[1].Yintercept, 20);
			tetragon[2].Y = 20;
			tetragon[3].X = cal_x(msg->coeff[1].Slope, msg->coeff[1].Yintercept, -20);
			tetragon[3].Y = -20;
		}
	}

	Lidar_MCduo_2023::RANSAC_points_arr to_clustering;
	if(switch_interior_points) to_clustering.coeff = msg->coeff;
	for(int i = 0; i < msg->data.size(); i++) {
		Lidar_MCduo_2023::RANSAC_points tmp;
		if(!switch_interior_points) {
			tmp.Rx = msg->data[i].Rx;
			tmp.Ry = msg->data[i].Ry;
			tmp.Rz = msg->data[i].Rz;
			tmp.Ri = msg->data[i].Ri;
			to_clustering.data.push_back(tmp);
		}
		else if(is_inner_point(msg->data[i].Rx, msg->data[i].Ry, tetragon)) {
			tmp.Rx = msg->data[i].Rx;
			tmp.Ry = msg->data[i].Ry;
			tmp.Rz = msg->data[i].Rz;
			tmp.Ri = msg->data[i].Ri;
			to_clustering.data.push_back(tmp);
		}
	}

	pub.publish(to_clustering);

	PCXYZI to_rviz;
	pcl::PointXYZI tmp;
	for(int i = 0; i < to_clustering.data.size(); i++) {
		tmp.x = to_clustering.data[i].Rx;
		tmp.y = to_clustering.data[i].Ry;
		tmp.z = to_clustering.data[i].Rz;
		tmp.intensity = to_clustering.data[i].Ri;
		to_rviz.push_back(tmp);
	}
	for(int i = 0; i < to_clustering.coeff.size(); i++) {
		tmp.x = tetragon[i].X;
		tmp.y = tetragon[i].Y;
		tmp.z = 0;
		to_rviz.push_back(tmp);
		tmp.x = tetragon[i + 2].X;
		tmp.y = tetragon[i + 2].Y;
		tmp.z = 0;
		to_rviz.push_back(tmp);
	}
	sensor_msgs::PointCloud2 output;
	pub_process(to_rviz, output);
	pub_rviz.publish(output);

	RT::end_cal("Interior_points");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "interior_points");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<Lidar_MCduo_2023::RANSAC_points_arr> ("/2_3_RANSAC_points_RPA", 1, interior_points_process);
	ros::Subscriber sub_totalInfo = nh.subscribe<Lidar_MCduo_2023::totalInfo> ("/Totalcom", 1, signal_process);
	pub = nh.advertise<Lidar_MCduo_2023::RANSAC_points_arr> ("/3_1_Interior_points_RPA", 1);
	pub_rviz = nh.advertise<sensor_msgs::PointCloud2> ("/3_2_Interior_points_PCL2", 1);

	ros::spin();
}
