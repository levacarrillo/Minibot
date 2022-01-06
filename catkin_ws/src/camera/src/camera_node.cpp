#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <raspicam/raspicam_cv.h>
#include <std_msgs/UInt8MultiArray.h>



using namespace std;


int main(int argc, char** argv) {
    cout << "Starting camera_node by Luis Näva..." << endl;
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

    ros::Publisher pubImage = nh.advertise<sensor_msgs::Image>("/image", 1);
    ros::Publisher pubJpeg  = nh.advertise<std_msgs::UInt8MultiArray>("/img_compressed", 1);
    ros::Rate loop(60);
 
    raspicam::RaspiCam_Cv camera;

    camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    camera.set(CV_CAP_PROP_ROLL, 180);

    if(!camera.open()) {
	cerr << "Error opening the raspicam... ;(" << endl;
	return -1;
    }    

    sensor_msgs::Image msgImage;
    std_msgs::UInt8MultiArray msgCompressed;

    int img_width  = 320;
    int img_height = 240;

    msgImage.header.frame_id = "camera_link";
    msgImage.data.resize(img_width*img_height*3);
    msgImage.height = img_height;
    msgImage.width  = img_width;
    msgImage.encoding = "bgr8";
    msgImage.step = img_width*3;
    
    vector<int> compressionParams(2);
    vector<uchar> compressedBuff;
    
    compressionParams[0] = cv::IMWRITE_JPEG_QUALITY;
    compressionParams[1] = 95;

    while(ros::ok() && cv::waitKey(15)!=27) {
	
	camera.grab();
    	cv::Mat image;

	camera.retrieve(image);

	cv::flip(image, image, 0);
	cv::flip(image, image, 1);

	msgImage.header.stamp = ros::Time::now();
	memcpy(msgImage.data.data(), image.data, img_width*img_height*3);
	cv::imencode(".jpg", image, msgCompressed.data, compressionParams); 

	pubImage.publish(msgImage);
	pubJpeg.publish(msgCompressed);

	ros::spinOnce();
	loop.sleep();
    }

    cout << "Stop camera..." << endl;
    camera.release();

    return 0;
}

