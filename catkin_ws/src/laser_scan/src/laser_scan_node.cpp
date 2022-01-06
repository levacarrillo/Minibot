#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16MultiArray.h>


using namespace std;


double sharp_distance[3];

void sharp_callback(const std_msgs::Int16MultiArray::ConstPtr& msg) {

	//left_sharp_distance  = 9.609  * pow(msg->data[0], -0.832);
	//right_sharp_distance = 19.327 * pow(msg->data[1], -0.958); 
    //cout << "left_sharp_distance: " << left_sharp_distance << "\tright_sharp_distance: " << right_sharp_distance << endl;

	for(int i=2; i<5; i++) {

		sharp_distance[i-2] = 9.609  * pow(msg->data[i], -0.832) + 0.08;
		if(sharp_distance[i-2] > 0.7) sharp_distance[i-2] = 0.7;		
	}

	//for(int i=0; i<8; i++)  sharp_distance[i] = 9.609  * pow(500, -0.832);
}

int main(int argc, char** argv){

	cout << "Starting laser_scan_node by Luis Näva..." << endl;

	ros::init(argc, argv, "laser_scan_node");
	ros::NodeHandle n;

	ros::Publisher  scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 50);
	ros::Subscriber laser_sub = n.subscribe("/sharp_sensors", 100, sharp_callback); 

	unsigned int num_readings = 3;
	double laser_frequency = 40;
	double ranges[num_readings];
	double intensities[num_readings];
	double theta = 0.5235;

	int count = 0;
	ros::Rate loop(20);

	tf::TransformListener listener;


	while(n.ok()){

		tf::StampedTransform transform;
		try {

			listener.waitForTransform("base_link", "laser_link", ros::Time(0), ros::Duration(10.0));
			listener.lookupTransform("base_link", "laser_link", ros::Time(0), transform);
		}

		catch (tf::TransformException ex){

			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		for(int i=2; i<5; i++) {
			ranges[i-2] = sharp_distance[i-2];
			intensities[i-2] = 100;
		}

		ros::Time scan_time = ros::Time::now();

		sensor_msgs::LaserScan scan;
		scan.header.stamp = scan_time;
		scan.header.frame_id = "laser_link";
		//scan.angle_min = - 2.35619;
		scan.angle_min = - 0.7853;
		//scan.angle_max =    3.1416;
		scan.angle_max = 3.1416;

		scan.angle_increment = 0.785398;
		scan.time_increment = (1 / laser_frequency) / (num_readings);
		scan.range_min = 0.0;
		scan.range_max = 100.0;

		scan.ranges.resize(num_readings);
		scan.intensities.resize(num_readings);

		for(unsigned int i = 0; i < num_readings; ++i) {

			scan.ranges[i] = ranges[i];
			scan.intensities[i] = intensities[i];
		}

		scan_pub.publish(scan);
		++count;

		ros::spinOnce();
		loop.sleep();
	}
}
