# groundtruth_tum_extractor

离线读取 `rosbag` 中的 `sensor_msgs/NavSatFix` 或 `geometry_msgs/PoseStamped` 话题，并输出 TUM 轨迹文本：

```text
# timestamp(s) x(m) y(m) z(m) qx qy qz qw
timestamp x y z qx qy qz qw
```

当前提供两个工具：

- `navsat_fix_to_tum`：处理 `sensor_msgs/NavSatFix`
- `pose_stamped_to_tum`：处理 `geometry_msgs/PoseStamped`

## 构建

```bash
cd /home/sax/SALM_GroundTruth_Extractor
catkin_make
source devel/setup.bash
```

## 运行

批量处理你的 `MCD` 数据目录：

```bash
rosrun groundtruth_tum_extractor navsat_fix_to_tum \
  --input /home/sax/rosbags/MCD \
  --topic /vn200/GPS \
  --type sensor_msgs/NavSatFix \
  --mode enu \
  --output /home/sax/rosbags/MCD/tum_truth
```

单独处理一个 bag：

```bash
rosrun groundtruth_tum_extractor navsat_fix_to_tum \
  --input /home/sax/rosbags/MCD/ntu_day_02/ntu_day_02_vn200.bag \
  --topic /vn200/GPS \
  --mode enu
```

批量处理 `VIRAL` 数据目录中的 Leica ground truth：

```bash
rosrun groundtruth_tum_extractor pose_stamped_to_tum \
  --input /home/sax/rosbags/VIRAL \
  --topic /leica/pose/relative \
  --type geometry_msgs/PoseStamped \
  --output /home/sax/rosbags/VIRAL/tum_truth
```

单独处理一个 VIRAL bag：

```bash
rosrun groundtruth_tum_extractor pose_stamped_to_tum \
  --input /home/sax/rosbags/VIRAL/eee_01/eee_01.bag \
  --topic /leica/pose/relative
```

## 输出规则

- 输入是目录时：递归扫描所有 `.bag` 文件，只转换同时满足 `topic` 和 `type` 的 bag。
- 输出目录会保留原始场景子目录结构。
- 每个输出文件第 1 行会写入 `# timestamp(s) x(m) y(m) z(m) qx qy qz qw` 作为列说明注释，真实轨迹数据从第 2 行开始。
- 默认会跳过 bag 开头“完全重复”的 GPS 前缀样本，避免把设备预热阶段的假值当成原点；如需保留，可加 `--keep-leading-duplicates`。
- `enu` 模式：以第一帧有效 GPS 为局部原点。
- `utm` 模式：先转 UTM，再减去第一帧 UTM 坐标，得到相对位移。
- `PoseStamped` 工具会直接写出消息中的位置与四元数，不做原点重置或额外坐标补偿。

## Launch

MCD 的 `NavSatFix` 导出：

```bash
roslaunch groundtruth_tum_extractor navsat_fix_to_tum.launch
```

VIRAL 的 `PoseStamped` 导出：

```bash
roslaunch groundtruth_tum_extractor pose_stamped_to_tum.launch
```

对应默认参数放在：

- `config/mcd_navsat_fix.yaml`
- `config/viral_pose_stamped.yaml`
