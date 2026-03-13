#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>

#include "groundtruth_tum_extractor/tum_export_utils.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

constexpr const char *kSupportedType = "geometry_msgs/PoseStamped";

struct Options
{
    fs::path input_path;
    fs::path output_path;
    std::string topic = "/leica/pose/relative";
    std::string type = kSupportedType;
};

struct ConvertStats
{
    std::size_t total_messages = 0;
    std::size_t written_messages = 0;
    std::size_t skipped_invalid_pose = 0;
};

struct Summary
{
    std::size_t bag_count = 0;
    std::size_t converted_count = 0;
    std::size_t skipped_count = 0;
};

bool isFinite(double value)
{
    return std::isfinite(value);
}

bool isValidPose(const geometry_msgs::PoseStamped &pose)
{
    const geometry_msgs::Point &position = pose.pose.position;
    const geometry_msgs::Quaternion &orientation = pose.pose.orientation;

    return isFinite(position.x) &&
           isFinite(position.y) &&
           isFinite(position.z) &&
           isFinite(orientation.x) &&
           isFinite(orientation.y) &&
           isFinite(orientation.z) &&
           isFinite(orientation.w);
}

ros::Time resolveStamp(const geometry_msgs::PoseStamped &pose, const rosbag::MessageInstance &message)
{
    return pose.header.stamp.isZero() ? message.getTime() : pose.header.stamp;
}

Options loadRosParams(Options options)
{
    ros::NodeHandle private_nh("~");

    std::string input_path = options.input_path.string();
    std::string output_path = options.output_path.string();

    private_nh.param<std::string>("input_path", input_path, input_path);
    private_nh.param<std::string>("output_path", output_path, output_path);
    private_nh.param<std::string>("topic", options.topic, options.topic);
    private_nh.param<std::string>("type", options.type, options.type);

    options.input_path = input_path;
    options.output_path = output_path;
    return options;
}

bool convertBag(const fs::path &bag_path, const Options &options, const fs::path &output_file, ConvertStats &stats)
{
    rosbag::Bag bag;
    bag.open(bag_path.string(), rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{options.topic}));

    std::string actual_type;
    if (!tum_export_utils::hasExpectedType(view, options.type, actual_type))
    {
        if (actual_type.empty())
        {
            ROS_WARN_STREAM("Skip bag without topic " << options.topic << ": " << bag_path);
        }
        else
        {
            ROS_WARN_STREAM("Skip bag due to type mismatch on " << options.topic << ": expected "
                            << options.type << ", got " << actual_type << " in " << bag_path);
        }

        bag.close();
        return false;
    }

    fs::create_directories(output_file.parent_path());
    std::ofstream output_stream(output_file);
    if (!output_stream.is_open())
    {
        bag.close();
        throw std::runtime_error("failed to open output file: " + output_file.string());
    }

    tum_export_utils::writeTumHeader(output_stream);

    for (const rosbag::MessageInstance &message : view)
    {
        ++stats.total_messages;

        const geometry_msgs::PoseStamped::ConstPtr pose = message.instantiate<geometry_msgs::PoseStamped>();
        if (!pose)
        {
            continue;
        }

        if (!isValidPose(*pose))
        {
            ++stats.skipped_invalid_pose;
            continue;
        }

        const ros::Time stamp = resolveStamp(*pose, message);
        if (stamp.isZero())
        {
            ++stats.skipped_invalid_pose;
            continue;
        }

        tum_export_utils::writeTumLine(
            output_stream,
            stamp,
            pose->pose.position.x,
            pose->pose.position.y,
            pose->pose.position.z,
            pose->pose.orientation.x,
            pose->pose.orientation.y,
            pose->pose.orientation.z,
            pose->pose.orientation.w);
        ++stats.written_messages;
    }

    output_stream.close();
    bag.close();

    if (stats.written_messages == 0)
    {
        fs::remove(output_file);
        ROS_WARN_STREAM("Skip bag without valid PoseStamped samples: " << bag_path);
        return false;
    }

    ROS_INFO_STREAM("Wrote " << stats.written_messages << " TUM poses to " << output_file
                    << " (skipped_invalid=" << stats.skipped_invalid_pose << ")");
    return true;
}

void printUsage(const char *program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " --input <bag_or_directory> [--topic /leica/pose/relative]\n"
        << "             [--type geometry_msgs/PoseStamped] [--output <output_file_or_output_dir>]\n\n"
        << "Notes:\n"
        << "  - If --input is a directory, the tool scans all .bag files recursively.\n"
        << "  - Only bags containing the requested topic with type geometry_msgs/PoseStamped are converted.\n"
        << "  - Output is standard TUM format with a header comment line.\n";
}

Options parseArguments(int argc, char **argv, Options options)
{
    const std::map<std::string, std::string *> option_targets = {
        {"--topic", &options.topic},
        {"--type", &options.type},
    };

    for (int index = 1; index < argc; ++index)
    {
        const std::string argument(argv[index]);
        if (argument == "--help" || argument == "-h")
        {
            printUsage(argv[0]);
            std::exit(0);
        }

        if (argument == "--input" || argument == "-i")
        {
            if (index + 1 >= argc)
            {
                throw std::runtime_error("--input requires a value");
            }
            options.input_path = argv[++index];
            continue;
        }

        if (argument == "--output" || argument == "-o")
        {
            if (index + 1 >= argc)
            {
                throw std::runtime_error("--output requires a value");
            }
            options.output_path = argv[++index];
            continue;
        }

        const auto it = option_targets.find(argument);
        if (it != option_targets.end())
        {
            if (index + 1 >= argc)
            {
                throw std::runtime_error(argument + " requires a value");
            }

            *(it->second) = argv[++index];
            continue;
        }

        throw std::runtime_error("unknown argument: " + argument);
    }

    if (options.input_path.empty())
    {
        throw std::runtime_error("--input is required");
    }

    if (options.type != kSupportedType)
    {
        throw std::runtime_error("only geometry_msgs/PoseStamped is supported, got: " + options.type);
    }

    return options;
}

}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_stamped_to_tum");

    try
    {
        Options options = loadRosParams(Options{});
        options = parseArguments(argc, argv, options);
        const std::vector<fs::path> bag_files = tum_export_utils::collectBagFiles(options.input_path);

        if (bag_files.empty())
        {
            ROS_ERROR_STREAM("No .bag files found under " << options.input_path);
            return 1;
        }

        Summary summary;
        summary.bag_count = bag_files.size();

        for (const fs::path &bag_path : bag_files)
        {
            ConvertStats stats;
            const fs::path output_file = tum_export_utils::defaultFileOutputPath(
                bag_path,
                options.input_path,
                options.output_path,
                tum_export_utils::sanitizeTopic(options.topic));

            try
            {
                if (convertBag(bag_path, options, output_file, stats))
                {
                    ++summary.converted_count;
                }
                else
                {
                    ++summary.skipped_count;
                }
            }
            catch (const std::exception &error)
            {
                ++summary.skipped_count;
                ROS_ERROR_STREAM("Failed to convert " << bag_path << ": " << error.what());
            }
        }

        ROS_INFO_STREAM("Finished. scanned=" << summary.bag_count
                        << ", converted=" << summary.converted_count
                        << ", skipped=" << summary.skipped_count);

        return summary.converted_count > 0 ? 0 : 1;
    }
    catch (const std::exception &error)
    {
        ROS_ERROR_STREAM(error.what());
        printUsage(argv[0]);
        return 1;
    }
}
