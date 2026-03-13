# SALM GroundTruth Extractor

`groundtruth_tum_extractor` is a ROS 1 / catkin package for converting ground-truth topics in a `rosbag` into TUM trajectory text files.

It is designed for offline export and works with:

- `sensor_msgs/NavSatFix`
- `geometry_msgs/PoseStamped`

The tool is not tied to a specific dataset. If your bags use different topic names, folder layouts, or output locations, you usually only need to change the input path, topic, and output path.

## Output Format

Each exported file uses standard TUM trajectory layout:

```text
# timestamp(s) x(m) y(m) z(m) qx qy qz qw
timestamp x y z qx qy qz qw
```

## Requirements

- ROS 1 with `catkin`
- `rosbag`
- `geometry_msgs`
- `sensor_msgs`
- `GeographicLib` for GPS projection

## Build

From the workspace root:

```bash
catkin_make
source devel/setup.bash
```

## Quick Start

### 1. Inspect your bag

Find the topic name and message type first:

```bash
rosbag info /path/to/your.bag
```

Choose the converter based on the topic type:

- Use `pose_stamped_to_tum` for `geometry_msgs/PoseStamped`
- Use `navsat_fix_to_tum` for `sensor_msgs/NavSatFix`

### 2. Export a single bag

For `PoseStamped`:

```bash
rosrun groundtruth_tum_extractor pose_stamped_to_tum \
  --input /path/to/your.bag \
  --topic /your/pose_topic \
  --output /path/to/output.tum.txt
```

For `NavSatFix`:

```bash
rosrun groundtruth_tum_extractor navsat_fix_to_tum \
  --input /path/to/your.bag \
  --topic /your/gps_topic \
  --mode enu \
  --output /path/to/output.tum.txt
```

### 3. Export a whole dataset directory

If `--input` is a directory, the tool scans all `.bag` files recursively.

For `PoseStamped`:

```bash
rosrun groundtruth_tum_extractor pose_stamped_to_tum \
  --input /path/to/dataset_root \
  --topic /your/pose_topic \
  --output /path/to/export_root
```

For `NavSatFix`:

```bash
rosrun groundtruth_tum_extractor navsat_fix_to_tum \
  --input /path/to/dataset_root \
  --topic /your/gps_topic \
  --mode enu \
  --output /path/to/export_root
```

## Converter Behavior

### `pose_stamped_to_tum`

- Supports only `geometry_msgs/PoseStamped`
- Writes position and quaternion directly from the message
- Uses `header.stamp` when available, otherwise falls back to bag time
- Skips samples with invalid numeric values

### `navsat_fix_to_tum`

- Supports only `sensor_msgs/NavSatFix`
- Writes a relative trajectory derived from latitude, longitude, and altitude
- Uses identity orientation: `0 0 0 1`
- Rejects invalid fixes and `STATUS_NO_FIX`
- By default, skips identical samples at the beginning of the bag to avoid GPS warm-up artifacts

Projection modes:

- `enu`: local ENU frame with the first valid GPS sample as origin
- `utm`: relative UTM coordinates using the first valid GPS sample as origin

If you want to keep repeated leading GPS samples, add:

```bash
--keep-leading-duplicates
```

## Output Rules

- For a single bag, the default output is written next to the bag unless `--output` is provided
- For a directory input, the default output root is `<input>/tum_truth`
- For a directory input, the original subdirectory structure is preserved
- Bags are converted only if the requested topic exists and its type matches the selected converter

Typical output filenames:

- `my_bag_your_pose_topic.tum.txt`
- `my_bag_your_gps_topic_enu.tum.txt`

## Launch File Usage

You can also run the converters through ROS launch files:

```bash
roslaunch groundtruth_tum_extractor pose_stamped_to_tum.launch
roslaunch groundtruth_tum_extractor navsat_fix_to_tum.launch
```

The default parameter files are:

- `src/groundtruth_tum_extractor/config/viral_pose_stamped.yaml`
- `src/groundtruth_tum_extractor/config/mcd_navsat_fix.yaml`

For a new dataset, copy one of these files and edit the fields you need:

```yaml
input_path: /path/to/dataset_or_bag
output_path: /path/to/output_root
topic: /your/topic
type: geometry_msgs/PoseStamped
```

or:

```yaml
input_path: /path/to/dataset_or_bag
output_path: /path/to/output_root
topic: /your/topic
type: sensor_msgs/NavSatFix
mode: enu
skip_leading_duplicates: true
```

Then run:

```bash
roslaunch groundtruth_tum_extractor pose_stamped_to_tum.launch \
  config_file:=/path/to/your_pose_config.yaml
```

or:

```bash
roslaunch groundtruth_tum_extractor navsat_fix_to_tum.launch \
  config_file:=/path/to/your_gps_config.yaml
```

## Limitations

- Only two message types are currently supported: `PoseStamped` and `NavSatFix`
- `NavSatFix` export does not estimate orientation
- If your dataset stores ground truth in another message type, you will need an additional converter

## Package Path

The ROS package lives in:

```text
src/groundtruth_tum_extractor
```
