# Rosbag Play Live Test

This folder contains one end-to-end live-play script:

- `nav_from_bag_pipeline.py`

## What it tests

Runs navigation in live ROS mode from bag topics:

1. Starts `auv_navigation` with `use_sim_time:=true`
2. Remaps generated outputs to isolated topics (default: `/navigation/generated/odometry`, `/navigation/generated/state`)
3. Records only those generated topics
4. Plays an input run from `../runs/<run_name>/` with `--clock`
5. Saves generated navigation path plot + summary
6. If input bag has onboard nav odom, overlays and compares against generated path

This validates the "bag play -> nav node -> nav topics" flow.

## How to run

```bash
cd /home/aatmaj/AUV
source /opt/ros/humble/setup.bash
source /home/aatmaj/AUV/code_ws/install/setup.bash
python3 kf_tests/rosbag_play_test/nav_from_bag_pipeline.py run3 --record-state
```

Optional reference topic override (if onboard nav topic is different):

```bash
python3 kf_tests/rosbag_play_test/nav_from_bag_pipeline.py run5 --record-state --input-nav-topic /navigation/odometry
```

Optional generated topic overrides:

```bash
python3 kf_tests/rosbag_play_test/nav_from_bag_pipeline.py run1 --record-state \
  --generated-nav-topic /navigation/generated/odometry \
  --generated-state-topic /navigation/generated/state
```

## Output location

For run `run3`, outputs are written to:

- `kf_tests/rosbag_play_test/run3_result/run3_result_0.db3` (recorded nav bag DB)
- `kf_tests/rosbag_play_test/run3_result/metadata.yaml`
- `kf_tests/rosbag_play_test/run3_result/nav_xy.png`
- `kf_tests/rosbag_play_test/run3_result/summary.txt` (includes generated vs input comparison when input nav topic exists)

