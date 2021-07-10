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
using namespace cv;                             

ros::NodeHandle *n;

ros::ServiceClient set_velocity_client;         // 电机速度通讯service客户端
webots_ros::set_float set_velocity_srv;         // 电机速度服务数据

#define TIME_STEP   32                          // 时钟
#define ROBOT_NAME  "tianbot_mini/"             // ROBOT名称
#define NMOTORS     2                           // 电机数量
double speeds[NMOTORS]={0.0,0.0};               // 两电机速度值 0～10
static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"}; // 匹配电机名


struct pid
{
    float pid_setValue;     // 设置值
    float pid_actualValue;  // 当前值
    float Kp;
    float Ki;
    float Kd;
    float err;              // 偏差
    float last_err;         // 上一次的偏差
    float speed;
    float T;                // 更新周期
    float integral;         // 累积误差
}_pid;
void pid_init(float Kp,float Ki,float Kd){
    _pid.pid_setValue = 0.0;
    _pid.pid_actualValue = 0.0;
    _pid.Kp = Kp;
    _pid.Ki = Ki;
    _pid.Kd = Kd;
    _pid.err = 0.0;
    _pid.last_err = 0.0;
    _pid.speed = 0.0;
    _pid.T = .0 ;
    _pid.integral = 0.0;
}

float pid_run(float value){
    _pid.pid_setValue = value;
    _pid.err = _pid.pid_setValue - _pid.pid_actualValue;
    _pid.integral = _pid.integral + _pid.err;
    _pid.speed = _pid.err * _pid.Kp + _pid.integral *_pid.T * _pid.Ki + (_pid.err - _pid.last_err) * _pid.Kd;
    _pid.last_err = _pid.err;
    _pid.pid_actualValue = _pid.speed;

    return _pid.pid_actualValue;
}


/*******************************************************
* Function name ：updateSpeed
* Description   ：将速度请求以set_float的形式发送给set_velocity_srv
* Parameter     ：无
* Return        ：无
**********************************************************/
void updateSpeed(Point middle_point) {  
    double speed_diff=0.0;
    double cspeed=0.0;
    speed_diff = pid_run((float)(middle_point.x - 32))/2.0;
    cout<<"speed_diff="<<speed_diff<<endl;
    speeds[0] = speed_diff;
    speeds[1] = -speed_diff;
    
    // 确定基准速度
    if (abs(middle_point.x - 32) <= 2){
        cspeed=9.0;
    }else if (abs(middle_point.x - 32) > 2){
        cspeed=7.0;
    }

    cout<<"leftspeed="<<speeds[0]<<endl;
    cout<<"rightspeed="<<speeds[1]<<endl; 
    for (int i = 0; i < NMOTORS; ++i) {
        // 发送给webots更新机器人速度
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string("/tianbot_mini/") + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = speeds[i]+cspeed;
        set_velocity_client.call(set_velocity_srv);
    }
}


/*******************************************************
* Function name ：FindPoints
* Description   ：找到算法中的关键点用函数
* Parameter     ：
        @img canny算法后的图像
* Return        ：无
**********************************************************/
void FindPoints(Mat img){
    Point left_point, right_point, middle_point;// 左上一点，右下一点，求出中点
    int flag = 1;// 点数
    // 找出左边线
    while (flag <= 4){
        for (int i = 0; i < 64; i++){
            // 在前块中查找左边线的点
            if (flag == 1){
                if((int)img.at<uchar>(0, i) == 255){
                    left_point.x = i;
                    left_point.y = 0;
                    cout<<"left_point="<<left_point<<endl;
                    flag = 4;
                    break;
                }
            } 
            // 在中块中查找左边线的点
            else if (flag == 2){
                if((int)img.at<uchar>(9, i) == 255){
                    left_point.x = i;
                    left_point.y = 9;
                    cout<<"left_point="<<left_point<<endl;
                    flag = 4;
                    break;
                }
            }
            else{
                cout<<"已超出范围"<<endl;
                flag = 6;
                break;
            }        
        }
        flag++;
    }  
    // 找到左边线后求解右边线
    for (int i = 63; i >= 0; i--){
        if((int)img.at<uchar>(23, i) == 255){
            right_point.x = i;
            right_point.y = 23;
            cout<<"right_point="<<right_point<<endl;
            break;
        }
    }
        
    //计算出中间点
    middle_point.x = left_point.x + (right_point.x - left_point.x)/2;
    middle_point.y = 12;//固定中点的高
    cout<<"middle_point="<<middle_point<<endl;
    line(img, Point(32,0), Point(32,24), Scalar(255));// 在感兴趣区域中绘制中心线
    line(img, middle_point, middle_point, Scalar(255),5);// 画出中心点

    updateSpeed(middle_point);// 更新速度

    imshow("img",img);  //显示检测效果图像
    waitKey(5);
}
/*******************************************************
* Function name ：ImageProcessFun
* Description   ：图像处理算法函数
* Parameter     ：
        @img_rgb 原图
* Return        ：
        @img canny处理过的图像
**********************************************************/
Mat ImageProcessFun(Mat img_rgb){
    Mat img_gray, img;
    img_rgb(Range(40, 64),Range(0, 64)).copyTo(img_gray); //选择感兴趣区域
    cvtColor(img_gray, img_gray, COLOR_BGR2GRAY);// 将图像转换为灰度图像
    threshold(img_gray, img, 100, 255, THRESH_BINARY);// 对感兴趣区域进行二值化操作
    Canny(img, img, 20, 40);// 由于仿真没有额外复杂环境，使用Canny边缘检测
    return img;
}

/*******************************************************
* Function name ：CameraCallback
* Description   ：webots摄像头数据回调函数
* Parameter     ：
* Return        ：无
**********************************************************/
void CameraCallback(const sensor_msgs::Image::ConstPtr &value){
    // 将webots提供的uchar格式数据转换成Mat
    Mat img_rgb = Mat(value->data).clone().reshape(4, 64);
    Mat img;
    img = ImageProcessFun(img_rgb);
    FindPoints(img);
}

int main(int argc, char **argv) {
    // 在ROS网络中创建一个名为tianbot_mini_init的节点
    ros::init(argc, argv, "tianbot_mini_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    pid_init(0.5,0.2,0.05);
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
