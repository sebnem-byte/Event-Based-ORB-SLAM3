#include "compiler_options.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <dvs_msgs/EventArray.h>


#include "System.h"
#include "EventLoader.h"

using namespace std;

ORB_SLAM3::System* SLAM;

void eventCallback(const dvs_msgs::EventArray::ConstPtr& event_msg) {
    vector<EORB_SLAM::EventData> vEvBuff;

    for (const auto& ev : event_msg->events) {
        vEvBuff.emplace_back(EORB_SLAM::EventData{ev.x, ev.y, ev.ts.toSec(), ev.polarity});
    }

    // Process events using SLAM
    SLAM->TrackEvent(vEvBuff);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    double timestamp = msg->header.stamp.toSec();
    SLAM->TrackMonocular(cv_ptr->image, timestamp);
}

void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    vImuMeas.emplace_back(ORB_SLAM3::IMU::Point(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z,
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z,
        imu_msg->header.stamp.toSec()
    ));

    SLAM->TrackEvent({}, vImuMeas); // Example for event + IMU
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "eorbslam_node");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun eorb_slam_node eorb_slam_node path_to_settings.yaml");
        return 1;
    }

    const string settingsFile = argv[1];
    EORB_SLAM::EvEthzLoader evEthzLoader(settingsFile);

    if (!evEthzLoader.isGood()) {
        ROS_ERROR("Cannot load settings file.");
        return 1;
    }

    const EORB_SLAM::SensorConfigPtr dsConf = evEthzLoader.getConfigStat();
    const EORB_SLAM::EvParamsPtr evParams = evEthzLoader.getEventParams();

    // Initialize SLAM system
    SLAM = new ORB_SLAM3::System(evEthzLoader.getPathOrbVoc(), dsConf, evEthzLoader.getCamParams(),
                                  evEthzLoader.getPairCamCalibrator(), evEthzLoader.getORBParams(),
                                  evEthzLoader.getIMUParams(), evEthzLoader.getViewerParams(), evParams);

    // ROS subscribers
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 10, imageCallback);
    ros::Subscriber event_sub = nh.subscribe("/dvs/events", 10, eventCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 10, imuCallback);

    ros::spin();

    SLAM->Shutdown();
    delete SLAM;

    return 0;
}
