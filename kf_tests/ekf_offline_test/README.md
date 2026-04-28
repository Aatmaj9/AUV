# EKF Offline Test

This folder contains the offline EKF replay/evaluation script:

- `test_ekf.py`

## What it tests

- Replays a rosbag from `../runs/<run_name>/` using Python `rosbags`.
- Runs the same `NavEKF` core imported from `auv_navigation`.
- Executes:
  - EKF with DVL DR updates
  - EKF without DVL DR updates
  - heading drift diagnostic
- Produces trajectory/diagnostic plots and a text summary.

## How to run

From repo root:

```bash
cd /home/aatmaj/AUV
python3 kf_tests/ekf_offline_test/test_ekf.py run3
```

## Output location

For run `run3`, outputs are always written to:

- `kf_tests/ekf_offline_test/run3_result/results.png`
- `kf_tests/ekf_offline_test/run3_result/heading.png`
- `kf_tests/ekf_offline_test/run3_result/summary.txt`

