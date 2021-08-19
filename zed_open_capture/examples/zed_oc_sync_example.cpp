///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

// ----> Includes
#include "videocapture.hpp"
#include "sensorcapture.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// <---- Includes

// ----> Functions
// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap);
// <---- Functions

// ----> Global variables
std::mutex imuMutex;
std::string imuTsStr;
std::string imuAccelStr;
std::string imuGyroStr;

bool sensThreadStop=false;
uint64_t mcu_sync_ts=0;
// <---- Global variables




std::string encoding_ = "bgr8";

void publishImage(const cv::Mat& img, image_transport::Publisher& img_pub, const std::string& img_frame_id,
                ros::Time t)
{
cv_bridge::CvImage cv_image;
// TODO(dizeng) maybe we can save a copy here?
// or it seems like CV mat is passing by reference?
cv_image.image = img;
// TODO(dizeng)
// by default the cv::mat from zed is bgr8, here just chaing encoding seems
// doesn't work, need to implement conversion function specificly
cv_image.encoding = encoding_;
cv_image.header.frame_id = img_frame_id;
cv_image.header.stamp = t;
img_pub.publish(cv_image.toImageMsg());
}



// The main function
int main(int argc, char **argv)
{
    //sl_oc::sensors::SensorCapture::resetSensorModule();
    //sl_oc::sensors::SensorCapture::resetVideoModule();


    ros::init(argc, argv, "zed_open_capture");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
    image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

    // Set the verbose level
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::ERROR;

    // ----> Set the video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD1080;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = verbose;
    // <---- Video parameters

    // ----> Create a Video Capture object
    sl_oc::video::VideoCapture videoCap(params);
    if( !videoCap.initializeVideo(-1) )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "Try to enable verbose to get more info" << std::endl;

        return EXIT_FAILURE;
    }

    // Serial number of the connected camera
    int camSn = videoCap.getSerialNumber();

    std::cout << "Video Capture connected to camera sn: " << camSn << std::endl;
    // <---- Create a Video Capture object

    // ----> Create a Sensors Capture object
    sl_oc::sensors::SensorCapture sensCap(verbose);
    if( !sensCap.initializeSensors(camSn) ) // Note: we use the serial number acquired by the VideoCapture object
    {
        std::cerr << "Cannot open sensors capture" << std::endl;
        std::cerr << "Try to enable verbose to get more info" << std::endl;

        return EXIT_FAILURE;
    }
    std::cout << "Sensors Capture connected to camera sn: " << sensCap.getSerialNumber() << std::endl;

    // Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
    // minor (max 100Hz), we use a separated thread for sensors.
    std::thread sensThread(getSensorThreadFunc,&sensCap);
    // <---- Create Sensors Capture

    // ----> Enable video/sensors synchronization
    videoCap.enableSensorSync(&sensCap);
    // <---- Enable video/sensors synchronization

    // ----> Init OpenCV RGB frame
    int w,h;
    videoCap.getFrameSize(w,h);

    cv::Size display_resolution(1024, 576);


    ROS_ERROR("height: %d,   width: %d", h, w);

    switch(params.res)
    {
    default:
    case sl_oc::video::RESOLUTION::VGA:
        display_resolution.width = w;
        display_resolution.height = h;
        break;
    case sl_oc::video::RESOLUTION::HD720:
        display_resolution.width = w*0.6;
        display_resolution.height = h*0.6;
        break;
    case sl_oc::video::RESOLUTION::HD1080:
    case sl_oc::video::RESOLUTION::HD2K:
        display_resolution.width = w*0.4;
        display_resolution.height = h*0.4;
        break;
    }

    cv::Mat frameBGR(h, w, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat left_image;
    cv::Mat right_image;

    cv::Rect left_rect(0, 0, w / 2, h);
    cv::Rect right_rect(w / 2, 0, w / 2, h);

    // <---- Init OpenCV RGB frame

    uint64_t last_timestamp = 0;

    float frame_fps=0;

    ros::Rate r(30);

    // Infinite grabbing loop
    while (ros::ok())
    {


        ros::Time now = ros::Time::now();

        const sl_oc::video::Frame frame = videoCap.getLastFrame(1);

        // If the frame is valid we can update it
        if(frame.data!=nullptr )
        {
            cv::Mat frameYUV( frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV,frameBGR, cv::COLOR_YUV2BGR_YUYV);
        }

        left_image = frameBGR(left_rect);
        right_image = frameBGR(right_rect);

        publishImage(left_image, left_image_pub, "left_frame", now);

        publishImage(right_image, right_image_pub, "right_frame", now);

        r.sleep();
    }

    ros::spin();

    return EXIT_SUCCESS;
}

// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap)
{

    static ros::NodeHandle nh;
    static ros::Publisher Imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 400);

    // Previous IMU timestamp to calculate frequency

    // Infinite data grabbing loop

    double pre_angle_vel[3];
    Eigen::Quaterniond quaternion_sum(1.0, 0.0, 0.0, 0.0);
    bool is_first = true;

    ros::Rate r(100);



    int cnt = 0;
    std::cout << "start removing zero-move" << std::endl;
    std::cout << "make the imu static, do not move!!!!!!" << std::endl;
    double zero_move[3];
    for(int i = 0; i < 3; i++)
        zero_move[i] = 0.0;
    int times = 1000;   
    while(cnt != times){
        ros::spinOnce();
        const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);
        zero_move[0] += imuData.gX*3.14159/180;
        zero_move[1] += imuData.gY*3.14159/180;
        zero_move[2] += imuData.gZ*3.14159/180;
        cnt++;
        r.sleep();
    }
    for(int i = 0; i < 3; i++)
        zero_move[i] /= times;    
    std::cout << "removing zero-move finished" << std::endl;




    while(ros::ok())
    {
        // ----> Get IMU data
        const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);

        // Process data only if valid
        if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
        {

            ros::spinOnce();

            sensor_msgs::Imu imu_msg;
            if(is_first){
                is_first = false;

                imu_msg.orientation.x = 0;
                imu_msg.orientation.y = 0;
                imu_msg.orientation.z = 0;
                imu_msg.orientation.w = 1;

            }
            else{
                Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(((imuData.gX*3.14159/180) + pre_angle_vel[0] - zero_move[0]) / 2 / 100, Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(((imuData.gY*3.14159/180) + pre_angle_vel[1] - zero_move[1]) / 2 / 100, Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(((imuData.gZ*3.14159/180) + pre_angle_vel[2] - zero_move[2]) / 2 / 100, Eigen::Vector3d::UnitZ()));
                Eigen::Quaterniond quaternion_tmp = yawAngle * pitchAngle * rollAngle;
                quaternion_sum = quaternion_sum * quaternion_tmp;
                imu_msg.orientation.x = quaternion_sum.x();
                imu_msg.orientation.y = quaternion_sum.y();
                imu_msg.orientation.z = quaternion_sum.z();
                imu_msg.orientation.w = quaternion_sum.w();
            }

            pre_angle_vel[0] = imuData.gX*3.14159/180 - zero_move[0];
            pre_angle_vel[1] = imuData.gY*3.14159/180 - zero_move[1];
            pre_angle_vel[2] = imuData.gZ*3.14159/180 - zero_move[2];
            imu_msg.angular_velocity.x = imuData.gX*3.14159/180 - zero_move[0];
            imu_msg.angular_velocity.y = imuData.gY*3.14159/180 - zero_move[1];
            imu_msg.angular_velocity.z = imuData.gZ*3.14159/180 - zero_move[2];
            imu_msg.header.stamp = ros::Time::now();
            //imu_msg.header.stamp.nsec = ros::Time::now().toNSec();
            imu_msg.header.frame_id = "imu";
            imu_msg.linear_acceleration.x = imuData.aX;
            imu_msg.linear_acceleration.y = imuData.aY;
            imu_msg.linear_acceleration.z = imuData.aZ;
            Imu_pub.publish(imu_msg);
                // <---- Data info to be displayed

        }

        r.sleep();
        // <---- Get IMU data
    }
}
