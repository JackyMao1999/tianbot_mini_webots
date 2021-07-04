/************************************************* 
Copyright:Tianbot_Mini Robot 
Author: 锡城筱凯
Date:2021-07-01
Blog：https://blog.csdn.net/xiaokai1999
Description:TianbotMini 机器人寻线代码
**************************************************/  
#include <signal.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include "ros/ros.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;                         // opencv

#define TIME_STEP 32                        // 时钟
#define ROBOT_NAME "tianbot_mini/"          // ROBOT名称

ros::NodeHandle *n;

void ImageProcessFun(Mat img_rgb){
    Mat img_gray, img;
    // createTrackbar("阈值", "img", &value, 255);
    img_rgb(Range(40, 64),Range(0, 64)).copyTo(img_gray); //选择感兴趣区域
    cvtColor(img_gray, img_gray, COLOR_BGR2GRAY);// 将图像转换为灰度图像
    threshold(img_gray, img, 100, 255, THRESH_BINARY);// 对感兴趣区域进行二值化操作
    Canny(img, img, 20, 40);// 由于仿真没有额外复杂环境，使用Canny边缘检测
    line(img, Point(32,0), Point(32,24), Scalar(255));// 在感兴趣区域中绘制中心线
    imshow("img",img);
    waitKey(5);
}


void CameraCallback(const sensor_msgs::Image::ConstPtr &value){
    // 将webots提供的uchar格式数据转换成Mat
    Mat img_rgb = Mat(value->data).clone().reshape(4, 64);
    ImageProcessFun(img_rgb);
}

int main(int argc, char **argv) {
    // 在ROS网络中创建一个名为tianbot_mini_init的节点
    ros::init(argc, argv, "tianbot_mini_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;

    // 使能摄像头
    ros::ServiceClient set_camera_client;
    webots_ros::set_int camera_srv;
    ros::Subscriber sub_camera_img;

    set_camera_client = n->serviceClient<webots_ros::set_int>(string(ROBOT_NAME)+string("camera/enable"));
    camera_srv.request.value = TIME_STEP;
    if (set_camera_client.call(camera_srv) && camera_srv.response.success) {
        sub_camera_img = n->subscribe(string(ROBOT_NAME)+string("camera/image"), 1, CameraCallback);
        ROS_INFO("Camera enabled.");
    } else {
        if (!camera_srv.response.success)
        ROS_ERROR("Failed to enable Camera.");
        return 1;
    }


    while (ros::ok()) {   
        ros::spinOnce();
    } 
    return 0;

}
