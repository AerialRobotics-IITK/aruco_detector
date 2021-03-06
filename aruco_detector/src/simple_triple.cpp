#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <mav_utils_msgs/signal.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_detector/ArucoThresholdConfig.h>

#define exit execFlag == -1
#define run execFlag == 1
#define echo(X) std::cout << X << std::endl

int execFlag = 0;

using namespace cv;
using namespace aruco;

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
MarkerDetector mDetector;
vector<Marker> markers;
ros::Subscriber cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub, debug_pub;
ros::Publisher pose_pub1, pose_pub2, pose_pub3, flag_pub;
std::string child_name1, parent_name, child_name2, child_name3;

double marker_size;
int marker_id1, marker_id2, marker_id3;
int flags[3] = {0,0,0};
std_msgs::Int16MultiArray objects;


void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if(run)
    {
        double ticksBefore = cv::getTickCount();
        static tf::TransformBroadcaster br;
        if(cam_info_received)
        {
            ros::Time curr_stamp(ros::Time::now());
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                inImage = cv_ptr->image;

                if(normalizeImageIllumination)
                {
                    ROS_WARN("normalizeImageIllumination is unimplemented!");
                    //cv::Mat inImageNorm;
                    //pal_vision_util::dctNormalization(inImage, inImageNorm, dctComponentsToRemove);
                    //inImage = inImageNorm;
                }

                //detection results will go into "markers"
                markers.clear(); objects.data.clear();
                objects.layout.data_offset = 0;
                flags[0] = flags[1] = flags[2] = 0;
                //Ok, let's detect
                mDetector.detect(inImage, markers, camParam, marker_size);
                //for each marker, draw info and its boundaries in the image
                for(unsigned int i=0; i<markers.size(); ++i)
                {
                    // only publishing the selected marker
                    if ( markers[i].id == marker_id1 )
                    {
                        tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
                        br.sendTransform(tf::StampedTransform(transform, curr_stamp,
                                                        parent_name, child_name1));
                        geometry_msgs::Pose poseMsg;
                        tf::poseTFToMsg(transform, poseMsg);
                        pose_pub1.publish(poseMsg);
                        flags[0] = 1;
                    }
                    else if ( markers[i].id == marker_id2 )
                    {
                        tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
                        br.sendTransform(tf::StampedTransform(transform, curr_stamp,
                                                        parent_name, child_name2));
                        geometry_msgs::Pose poseMsg;
                        tf::poseTFToMsg(transform, poseMsg);
                        pose_pub2.publish(poseMsg);
                        flags[1] = 1;
                    }
                    else if ( markers[i].id == marker_id3 )
                    {
                        tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
                        br.sendTransform(tf::StampedTransform(transform, curr_stamp,
                                                        parent_name, child_name2));
                        geometry_msgs::Pose poseMsg;
                        tf::poseTFToMsg(transform, poseMsg);
                        pose_pub3.publish(poseMsg);
                        flags[2] = 1;
                    }
                    // but drawing all the detected markers
                    markers[i].draw(inImage,Scalar(0,0,255),2);
                }
                
                for(int i=0; i<3; i++){ objects.data.push_back(flags[i]); }
                objects.layout.data_offset = cv_ptr->header.seq;
                flag_pub.publish(objects);

                if(image_pub.getNumSubscribers() > 0)
                {
                //show input with augmented information
                    cv_bridge::CvImage out_msg;
                    out_msg.header.stamp = curr_stamp;
                    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                    out_msg.image = inImage;
                    image_pub.publish(out_msg.toImageMsg());
                }

                if(debug_pub.getNumSubscribers() > 0)
                {
                    //show also the internal image resulting from the threshold operation
                    cv_bridge::CvImage debug_msg;
                    debug_msg.header.stamp = curr_stamp;
                    debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
                    debug_msg.image = mDetector.getThresholdedImage();
                    debug_pub.publish(debug_msg.toImageMsg());
                }

                ROS_DEBUG("runtime: %f ms",
                    1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
                
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
    }
}

void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
    cam_info_received = true;
    cam_info_sub.shutdown();
}

void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
{
    mDetector.setThresholdParams(config.param1,config.param2);
    normalizeImageIllumination = config.normalizeImage;
    dctComponentsToRemove      = config.dctComponentsToRemove;
}

bool serviceCall(mav_utils_msgs::signal::Request &req, mav_utils_msgs::signal::Response &res)
{
    switch(req.signal){
        case -1:
        case 0:
        case 1: execFlag = req.signal;
                switch(execFlag){
                    case 0: ROS_INFO("Paused"); break;
                    case 1: ROS_INFO("Starting"); break;
                    case -1: ROS_INFO("Exiting"); break;
                }
                res.success = true;
                break;
        default: res.success = false;
                break;
    }
    return true;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "detector");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    ros::ServiceServer exec_server = nh.advertiseService("terminate", serviceCall);

    dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
    dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
    f_ = boost::bind(&reconf_callback, _1, _2);
    server.setCallback(f_);

    normalizeImageIllumination = false;

    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
    ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

    image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
    cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

    cam_info_received = false;
    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub1 = nh.advertise<geometry_msgs::Pose>("pose1", 100);
    pose_pub2 = nh.advertise<geometry_msgs::Pose>("pose2", 100);
    pose_pub3 = nh.advertise<geometry_msgs::Pose>("pose3", 100);
    flag_pub = nh.advertise<std_msgs::Int16MultiArray>("flags",100);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id1", marker_id1, 582);
    nh.param<int>("marker_id2", marker_id2, 26);
    nh.param<int>("marker_id3", marker_id3, 80);
    nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
    nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
    
    if(dctComponentsToRemove == 0) normalizeImageIllumination = false;
    
    nh.param<std::string>("parent_name", parent_name, "");
    nh.param<std::string>("child_name1", child_name1, "");
    nh.param<std::string>("child_name2", child_name2, "");
    nh.param<std::string>("child_name3", child_name3, "");

    if(parent_name == "" || child_name1 == "" || child_name2 == "" || child_name3 == "")
    {
        ROS_ERROR("parent_name and/or child_name was not set!");
        return -1;
    }

    ROS_INFO("Aruco node started with marker size of %f meters and marker ids to track: %d, %d, %d",
           marker_size, marker_id1, marker_id2, marker_id3);
    ROS_INFO("Aruco node will publish pose to TF with (%s, %s), (%s, %s) and (%s, %s) as (parent,child).",
           parent_name.c_str(), child_name1.c_str(), parent_name.c_str(), child_name2.c_str(), parent_name.c_str(), child_name3.c_str());

    ros::Rate loopRate(10);
    while(ros::ok() && !(exit))
    {
        ros::spinOnce();
        loopRate.sleep();
    }
}
