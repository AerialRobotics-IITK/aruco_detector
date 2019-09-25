#include <ros/ros.h>
#include <mav_utils_msgs/BBPose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <mav_utils_msgs/BBPoses.h>
#include <std_msgs/Int16MultiArray.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#define sq(X) (X)*(X)
#define echo(X) std::cout << X << std::endl

geometry_msgs::Pose pose[3];
nav_msgs::Odometry odom;
std::vector<int16_t> flags;
Eigen::Matrix3f camToQuad, quadToCam;
Eigen::Vector3f tCam;
int imageID; int types[3] = {0,0,0};
double maxCentreError;
int height, width;

void pose1Callback(const geometry_msgs::Pose& msg){pose[0] = msg;}
void pose2Callback(const geometry_msgs::Pose& msg){pose[1] = msg;}
void pose3Callback(const geometry_msgs::Pose& msg){pose[2] = msg;}
void odomCallback(const nav_msgs::Odometry& msg){odom = msg;}

void flagCallback(const std_msgs::Int16MultiArray& msg)
{
    flags = msg.data;
    imageID = msg.layout.data_offset;
}

mav_utils_msgs::BBPose convertPose(geometry_msgs::Pose msg, int i)
{
    mav_utils_msgs::BBPose temp;
    Eigen::Matrix3f quadToGlob;

    tf::Quaternion q1(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    Eigen::Quaternionf quat = Eigen::Quaternionf(q1.w(), q1.x(), q1.y(), q1.z());
    quadToGlob = quat.toRotationMatrix();
    
    Eigen::Vector3f camCoord(msg.position.x, msg.position.y, msg.position.z - odom.pose.pose.position.z);
    Eigen::Vector3f quadCoord = camToQuad*camCoord + tCam;
    Eigen::Vector3f globCoord = quadToGlob*quadCoord;

    temp.position.x = globCoord(0) + odom.pose.pose.position.x;
    temp.position.y = globCoord(1) + odom.pose.pose.position.y;
    temp.position.z = globCoord(2) + odom.pose.pose.position.z;

    temp.boxID = i;
    temp.type = types[i];
    temp.area = -1;
    
    temp.store = (sq(msg.position.x - height/2) + sq(msg.position.y - width/2) < maxCentreError) ? true: false;
    if(temp.type == -40) temp.store = true;

    return temp;   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_pose");
    ros::NodeHandle nh, ph("~");

    ph.getParam("marker1", types[0]);
    ph.getParam("marker2", types[1]);
    ph.getParam("marker3", types[2]);

    ph.getParam("maxCentreError", maxCentreError);
    ph.getParam("width", width);
    ph.getParam("height", height);

    std::vector<double> tempList;
    ph.getParam("camera/translation", tempList);
    for (int i = 0; i < 3; i++)
    {
        tCam(i) = tempList[i];
    }

    int tempIdx = 0; tempList.clear();
    ph.getParam("camera/rotation", tempList);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            quadToCam(i,j) = tempList[tempIdx++];
        }
    }
    camToQuad = quadToCam.inverse();

    ros::Subscriber pose_sub1 = nh.subscribe("pose1", 1, pose1Callback);
    ros::Subscriber pose_sub2 = nh.subscribe("pose2", 1, pose2Callback);
    ros::Subscriber pose_sub3 = nh.subscribe("pose3", 1, pose3Callback);
    ros::Subscriber flag_sub = nh.subscribe("flags", 1, flagCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);

    ros::Publisher obj_pub = nh.advertise<mav_utils_msgs::BBPoses>("object_poses", 1);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        mav_utils_msgs::BBPoses msg;
        ros::spinOnce();
        if(flags.size() == 3)
        {
            for(int i=0; i<3; i++)
            {
                mav_utils_msgs::BBPose obj;
                if(flags.at(i) == 1)
                {
                    obj = convertPose(pose[i], i);
                    msg.object_poses.push_back(obj);
                }
            }
            msg.stamp = ros::Time::now();
            msg.imageID = imageID;
            obj_pub.publish(msg);
        }
       loopRate.sleep();
    }

    return 0;
}
