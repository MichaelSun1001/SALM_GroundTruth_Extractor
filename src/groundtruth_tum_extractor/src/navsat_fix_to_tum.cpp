#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "groundtruth_tum_extractor/tum_export_utils.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

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

constexpr const char *kSupportedType = "sensor_msgs/NavSatFix";

struct Options
{
    fs::path input_path;
    fs::path output_path;
    std::string topic = "/vn200/GPS";
    std::string type = kSupportedType;
    std::string mode = "enu";
    bool skip_leading_duplicates = true;
};

struct ConvertStats
{
    std::size_t total_messages = 0;
    std::size_t written_messages = 0;
    std::size_t skipped_invalid_fix = 0;
    std::size_t skipped_leading_duplicates = 0;
};

struct Summary
{
    std::size_t bag_count = 0;
    std::size_t converted_count = 0;
    std::size_t skipped_count = 0;
};

bool isValidFix(const sensor_msgs::NavSatFix &fix)
{
    if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) || !std::isfinite(fix.altitude))
    {
        return false;
    }

    return fix.status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX;
}

bool sameFix(const sensor_msgs::NavSatFix &lhs, const sensor_msgs::NavSatFix &rhs)
{
    return lhs.latitude == rhs.latitude &&
           lhs.longitude == rhs.longitude &&
           lhs.altitude == rhs.altitude;
}

ros::Time resolveStamp(const sensor_msgs::NavSatFix &fix, const rosbag::MessageInstance &message)
{
    return fix.header.stamp.isZero() ? message.getTime() : fix.header.stamp;
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
    private_nh.param<std::string>("mode", options.mode, options.mode);
    private_nh.param("skip_leading_duplicates", options.skip_leading_duplicates, options.skip_leading_duplicates);

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

    GeographicLib::LocalCartesian local_cartesian;
    bool initialized = false;
    bool pending_fix_set = false;
    sensor_msgs::NavSatFix pending_fix;
    ros::Time pending_stamp;
    std::size_t leading_duplicate_count = 0;

    double init_utm_x = 0.0;
    double init_utm_y = 0.0;
    double init_altitude = 0.0;
    int init_zone = 0;
    bool init_north = true;

    auto initializeOrigin = [&](const sensor_msgs::NavSatFix &origin_fix)
    {
        if (options.mode == "enu")
        {
            local_cartesian.Reset(origin_fix.latitude, origin_fix.longitude, origin_fix.altitude);
        }
        else
        {
            GeographicLib::UTMUPS::Forward(
                origin_fix.latitude, origin_fix.longitude, init_zone, init_north, init_utm_x, init_utm_y);
            init_altitude = origin_fix.altitude;
        }
        initialized = true;
    };

    auto projectFix = [&](const sensor_msgs::NavSatFix &sample_fix, double &x, double &y, double &z)
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;

        if (options.mode == "enu")
        {
            local_cartesian.Forward(sample_fix.latitude, sample_fix.longitude, sample_fix.altitude, x, y, z);
        }
        else
        {
            double utm_x = 0.0;
            double utm_y = 0.0;
            int zone = 0;
            bool northp = true;
            GeographicLib::UTMUPS::Forward(sample_fix.latitude, sample_fix.longitude, zone, northp, utm_x, utm_y);

            if (zone != init_zone || northp != init_north)
            {
                ROS_WARN_STREAM_THROTTLE(
                    1.0,
                    "UTM zone changed inside bag " << bag_path << ", output remains relative to first UTM zone");
            }

            x = utm_x - init_utm_x;
            y = utm_y - init_utm_y;
            z = sample_fix.altitude - init_altitude;
        }
    };

    for (const rosbag::MessageInstance &message : view)
    {
        ++stats.total_messages;

        const sensor_msgs::NavSatFix::ConstPtr fix = message.instantiate<sensor_msgs::NavSatFix>();
        if (!fix)
        {
            continue;
        }

        if (!isValidFix(*fix))
        {
            ++stats.skipped_invalid_fix;
            continue;
        }

        const ros::Time stamp = resolveStamp(*fix, message);

        if (!initialized && options.skip_leading_duplicates)
        {
            if (!pending_fix_set)
            {
                pending_fix = *fix;
                pending_stamp = stamp;
                pending_fix_set = true;
                leading_duplicate_count = 1;
                continue;
            }

            if (sameFix(*fix, pending_fix))
            {
                ++leading_duplicate_count;
                continue;
            }

            if (leading_duplicate_count == 1)
            {
                initializeOrigin(pending_fix);
                tum_export_utils::writeTumLine(output_stream, pending_stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
                ++stats.written_messages;
            }
            else
            {
                stats.skipped_leading_duplicates += leading_duplicate_count;
                initializeOrigin(*fix);
                tum_export_utils::writeTumLine(output_stream, stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
                ++stats.written_messages;
                continue;
            }
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        if (!initialized)
        {
            initializeOrigin(*fix);
        }

        projectFix(*fix, x, y, z);
        tum_export_utils::writeTumLine(output_stream, stamp, x, y, z, 0.0, 0.0, 0.0, 1.0);
        ++stats.written_messages;
    }

    if (!initialized && pending_fix_set)
    {
        initializeOrigin(pending_fix);
        tum_export_utils::writeTumLine(output_stream, pending_stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        ++stats.written_messages;
    }

    output_stream.close();
    bag.close();

    if (!initialized)
    {
        fs::remove(output_file);
        ROS_WARN_STREAM("Skip bag without valid NavSatFix samples: " << bag_path);
        return false;
    }

    ROS_INFO_STREAM("Wrote " << stats.written_messages << " TUM poses to " << output_file
                    << " (skipped_invalid=" << stats.skipped_invalid_fix
                    << ", skipped_leading_duplicates=" << stats.skipped_leading_duplicates << ")");
    return true;
}

void printUsage(const char *program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " --input <bag_or_directory> [--topic /vn200/GPS]\n"
        << "             [--type sensor_msgs/NavSatFix] [--mode enu|utm]\n"
        << "             [--output <output_file_or_output_dir>] [--keep-leading-duplicates]\n\n"
        << "Notes:\n"
        << "  - If --input is a directory, the tool scans all .bag files recursively.\n"
        << "  - Only bags containing the requested topic with type sensor_msgs/NavSatFix are converted.\n"
        << "  - By default, identical samples at the beginning of a bag are skipped as GPS warmup.\n"
        << "  - Output is standard TUM format with a header comment line.\n";
}

Options parseArguments(int argc, char **argv, Options options)
{
    const std::map<std::string, std::string *> option_targets = {
        {"--topic", &options.topic},
        {"--type", &options.type},
        {"--mode", &options.mode},
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

        if (argument == "--keep-leading-duplicates")
        {
            options.skip_leading_duplicates = false;
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
        throw std::runtime_error("only sensor_msgs/NavSatFix is supported, got: " + options.type);
    }

    if (options.mode != "enu" && options.mode != "utm")
    {
        throw std::runtime_error("--mode must be either 'enu' or 'utm'");
    }

    return options;
}

}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navsat_fix_to_tum");

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
                tum_export_utils::sanitizeTopic(options.topic) + "_" + options.mode);

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
