#ifndef GROUNDTRUTH_TUM_EXTRACTOR_TUM_EXPORT_UTILS_HPP_
#define GROUNDTRUTH_TUM_EXTRACTOR_TUM_EXPORT_UTILS_HPP_

#include <ros/time.h>
#include <rosbag/view.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

namespace tum_export_utils
{

namespace fs = std::filesystem;

constexpr const char *kTumHeader = "# timestamp(s) x(m) y(m) z(m) qx qy qz qw";

inline std::string sanitizeTopic(const std::string &topic)
{
    std::string clean = topic;
    while (!clean.empty() && clean.front() == '/')
    {
        clean.erase(clean.begin());
    }

    for (char &ch : clean)
    {
        if (ch == '/')
        {
            ch = '_';
        }
    }

    return clean.empty() ? "topic" : clean;
}

inline std::string makeTumFileName(const fs::path &bag_path, const std::string &suffix)
{
    std::string file_name = bag_path.stem().string();
    if (!suffix.empty())
    {
        file_name += "_" + suffix;
    }

    file_name += ".tum.txt";
    return file_name;
}

inline fs::path defaultFileOutputPath(
    const fs::path &bag_path,
    const fs::path &input_path,
    const fs::path &output_path,
    const std::string &suffix)
{
    const std::string file_name = makeTumFileName(bag_path, suffix);

    if (fs::is_regular_file(input_path))
    {
        if (!output_path.empty())
        {
            return output_path;
        }

        return bag_path.parent_path() / file_name;
    }

    const fs::path output_root = output_path.empty() ? (input_path / "tum_truth") : output_path;
    const fs::path relative_parent = fs::relative(bag_path.parent_path(), input_path);
    return output_root / relative_parent / file_name;
}

inline std::vector<fs::path> collectBagFiles(const fs::path &input_path)
{
    std::vector<fs::path> bag_files;

    if (fs::is_regular_file(input_path))
    {
        if (input_path.extension() != ".bag")
        {
            throw std::runtime_error("input file is not a .bag file: " + input_path.string());
        }

        bag_files.push_back(input_path);
    }
    else if (fs::is_directory(input_path))
    {
        for (const auto &entry : fs::recursive_directory_iterator(input_path))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".bag")
            {
                bag_files.push_back(entry.path());
            }
        }

        std::sort(bag_files.begin(), bag_files.end());
    }
    else
    {
        throw std::runtime_error("input path does not exist: " + input_path.string());
    }

    return bag_files;
}

inline bool hasExpectedType(rosbag::View &view, const std::string &expected_type, std::string &actual_type)
{
    const auto connections = view.getConnections();
    if (connections.empty())
    {
        actual_type.clear();
        return false;
    }

    actual_type = connections.front()->datatype;
    return std::all_of(
        connections.begin(),
        connections.end(),
        [&expected_type](const rosbag::ConnectionInfo *connection)
        {
            return connection->datatype == expected_type;
        });
}

inline void writeTumHeader(std::ofstream &stream)
{
    stream << kTumHeader << '\n';
}

inline void writeTumLine(
    std::ofstream &stream,
    const ros::Time &stamp,
    double x,
    double y,
    double z,
    double qx,
    double qy,
    double qz,
    double qw)
{
    stream << std::fixed << std::setprecision(9)
           << stamp.toSec() << " "
           << x << " " << y << " " << z << " "
           << qx << " " << qy << " " << qz << " " << qw << '\n';
}

}  // namespace tum_export_utils

#endif  // GROUNDTRUTH_TUM_EXTRACTOR_TUM_EXPORT_UTILS_HPP_
