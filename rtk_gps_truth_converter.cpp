#include "ros/ros.h"

#include <sensor_msgs/NavSatFix.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>

class RTKGpsTruthConverter
{
public:
    RTKGpsTruthConverter()
    {
        nh_.param<std::string>("rtkTopic", rtkTopic_, "/raw_gnss");
        nh_.param<std::string>("rtk_topic", rtkTopic_, rtkTopic_);
        nh_.param<std::string>("rtk_save_path", rtk_save_path_, "/home/sax/NDT_Radar/txt/rtk_lla_file.txt");
        nh_.param<std::string>("rtk_mode", rtk_mode_, "enu");

        std::filesystem::path output_path(rtk_save_path_);
        if (output_path.has_parent_path())
        {
            std::filesystem::create_directories(output_path.parent_path());
        }

        rtk_file_.open(rtk_save_path_.c_str());
        if (!rtk_file_.is_open())
        {
            ROS_ERROR("无法打开 RTK 数据保存文件: %s", rtk_save_path_.c_str());
            ros::shutdown();
            return;
        }

        if (rtk_mode_ == "utm")
        {
            rtk_sub_ = nh_.subscribe(rtkTopic_, 1, &RTKGpsTruthConverter::rtk_utmCallback, this);
            ROS_INFO("RTK/GPS truth converter mode = UTM, topic = %s", rtkTopic_.c_str());
        }
        else
        {
            rtk_sub_ = nh_.subscribe(rtkTopic_, 1, &RTKGpsTruthConverter::rtk_enuCallback, this);
            ROS_INFO("RTK/GPS truth converter mode = ENU, topic = %s", rtkTopic_.c_str());
        }

        ROS_INFO("RTK truth output file = %s", rtk_save_path_.c_str());
    }

    ~RTKGpsTruthConverter()
    {
        if (rtk_file_.is_open())
        {
            rtk_file_.close();
        }
    }

    void rtk_enuCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        double e, n, u;

        if (!rtk_initialized_)
        {
            initial_rtk_ = *msg;
            rtk_initialized_ = true;
            local_cartesian_.Reset(initial_rtk_.latitude, initial_rtk_.longitude, initial_rtk_.altitude);
            e = 0.0;
            n = 0.0;
            u = 0.0;
        }
        else
        {
            local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, e, n, u);
        }

        writeTumLine(msg->header.stamp, e, n, u);
    }

    void rtk_utmCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        double utm_x, utm_y;
        int zone;
        bool northp;

        GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, utm_x, utm_y);

        double z = msg->altitude;

        if (!rtk_initialized_)
        {
            init_utm_x_ = utm_x;
            init_utm_y_ = utm_y;
            init_alt_ = z;
            rtk_initialized_ = true;

            utm_x = 0.0;
            utm_y = 0.0;
            z = 0.0;
        }
        else
        {
            utm_x = utm_x - init_utm_x_;
            utm_y = utm_y - init_utm_y_;
            z = z - init_alt_;
        }

        writeTumLine(msg->header.stamp, utm_x, utm_y, z);
    }

private:
    void writeTumLine(const ros::Time &stamp, double x, double y, double z)
    {
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;

        rtk_file_ << std::fixed << std::setprecision(3)
                  << stamp.sec << "."
                  << std::setw(9) << std::setfill('0') << stamp.nsec << " "
                  << x << " " << y << " " << z << " "
                  << qx << " " << qy << " " << qz << " " << qw << std::endl;
    }

    ros::NodeHandle nh_;
    ros::Subscriber rtk_sub_;

    GeographicLib::LocalCartesian local_cartesian_;

    bool rtk_initialized_ = false;
    sensor_msgs::NavSatFix initial_rtk_;

    double init_utm_x_ = 0.0;
    double init_utm_y_ = 0.0;
    double init_alt_ = 0.0;

    std::ofstream rtk_file_;
    std::mutex data_mutex_;

    std::string rtkTopic_;
    std::string rtk_save_path_;
    std::string rtk_mode_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtk_gps_truth_converter");
    RTKGpsTruthConverter node;
    ros::spin();
    return 0;
}
