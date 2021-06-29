/************************************************* 
Copyright:tianbot_mini Robot 
Author: 锡城筱凯
Date:2021-05-23
Blog：https://blog.csdn.net/xiaokai1999
Description:跑gmapping建图时的传感器启动文件
**************************************************/  
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

#include <rosgraph_msgs/Clock.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Float64Stamped.h>

using namespace std;

#define TIME_STEP 32                            // Webots时钟
ros::NodeHandle *n;                             

static int controllerCount;                     // 设置控制器数量
static std::vector<std::string> controllerList; // 控制器列表

ros::ServiceClient timeStepClient;              // 时钟通讯客户端
webots_ros::set_int timeStepSrv;                // 时钟服务数据

double GPSvalues[2];                            // GPS数据列表
double Inertialvalues[4];                       // IMU数据列表

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());

}

/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/tianbot_mini_init' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}
/*******************************************************
* Function name ：broadcastTransform
* Description   ：TF坐标转换函数
* Parameter     ：
* Return        ：无
**********************************************************/
void broadcastTransform()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(GPSvalues[0],GPSvalues[1],0));
    tf::Quaternion q(Inertialvalues[0],Inertialvalues[2],Inertialvalues[1],-Inertialvalues[3]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link"));
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "tianbot_mini/LDS_01"));
}

/*******************************************************
* Function name ：InertialUnitCallback
* Description   ：IMU数据回调函数
* Parameter     ：
* Return        ：无
**********************************************************/
void InertialUnitCallback(const sensor_msgs::Imu::ConstPtr &value)
{
    
    Inertialvalues[0] = value->orientation.x;
    Inertialvalues[1] = value->orientation.y;
    Inertialvalues[2] = value->orientation.z;
    Inertialvalues[3] = value->orientation.w;
    broadcastTransform();
}
/*******************************************************
* Function name ：GPSCallback
* Description   ：GPS数据回调函数
* Parameter     ：
* Return        ：无
**********************************************************/
void GPSCallback(const geometry_msgs::PointStamped::ConstPtr &value)
{
    GPSvalues[0] = value->point.x;
    GPSvalues[1] = value->point.z;
    broadcastTransform();  
}

int main(int argc,char **argv)
{
    std::string controllerName;
    // 在ROS网络中创建一个名为’tianbot_mini_init‘的节点
    ros::init(argc, argv, "tianbot_mini_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 程序退出信号
    signal(SIGINT, quit);

    // 订阅model_name消息来获得可用的控制器
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    ros::spinOnce();

    timeStepClient = n->serviceClient<webots_ros::set_int>("tianbot_mini/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // 如果存在一个以上的控制器，让用户选择其中一个
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    nameSub.shutdown();

    // 使能激光雷达传感器
    ros::ServiceClient set_lidar_client;
    webots_ros::set_int lidar_srv;
    ros::Subscriber sub_lidar_scan;

    set_lidar_client = n->serviceClient<webots_ros::set_int>("tianbot_mini/LDS_01/enable");
    lidar_srv.request.value = TIME_STEP;
    if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) {
        ROS_INFO("Lidar enabled.");
    } else {
        if (!lidar_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable lidar.");
        return 1;
    }

    // 使能GPS传感器
    ros::ServiceClient set_GPS_client;
    webots_ros::set_int GPS_srv;
    ros::Subscriber sub_GPS;
    ros::Subscriber sub_GPS_speed;
    set_GPS_client = n->serviceClient<webots_ros::set_int>("tianbot_mini/gps/enable");
    GPS_srv.request.value = TIME_STEP;
    if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) {
        sub_GPS = n->subscribe("tianbot_mini/gps/values", 1, GPSCallback);
        // sub_GPS_speed = n->subscribe("volcano/gps/speed", 1, GPSspeedCallback);
        while (sub_GPS.getNumPublishers() == 0) {
        }
        ROS_INFO("GPS enabled.");
    } else {
        if (!GPS_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable GPS.");
        return 1;
    }

    // enable inertial unit
    ros::ServiceClient set_inertial_unit_client;
    webots_ros::set_int inertial_unit_srv;
    ros::Subscriber sub_inertial_unit;
    set_inertial_unit_client = n->serviceClient<webots_ros::set_int>("tianbot_mini/inertial_unit/enable");
    inertial_unit_srv.request.value = TIME_STEP;
    if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
        sub_inertial_unit = n->subscribe("tianbot_mini/inertial_unit/quaternion", 1, InertialUnitCallback);
        while (sub_inertial_unit.getNumPublishers() == 0) {
        }
        ROS_INFO("Inertial unit enabled.");
    } else {
        if (!inertial_unit_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable inertial unit.");
        return 1;
    }

    ROS_INFO("You can now start the creation of the map using 'rosrun gmapping slam_gmapping "
            "scan:=/tianbot_mini/LDS_01/laser_scan/layer0 _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30 _delta:=0.2'.");
    ROS_INFO("You can now visualize the sensors output in rqt using 'rqt'.");

    // main loop
    while (ros::ok()) {
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
        ROS_ERROR("Failed to call service time_step for next step.");
        break;
        }
        ros::spinOnce();
    }
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);

    ros::shutdown();
    return 0;

}