"""Load vessel YAML: env AUV_VESSEL_DATA, --ros-args -p vessel_data_file:=..., or package share default."""

import os
from typing import Any, Dict, Optional

import yaml


def read_vessel_data(file_path: Optional[str] = None) -> Dict[str, Any]:
    if file_path is None:
        file_path = os.environ.get("AUV_VESSEL_DATA")

    if file_path is None or not os.path.isfile(file_path):
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("auv_navigation")
            candidate = os.path.join(share, "config", "vessel_data.example.yml")
            if os.path.isfile(candidate):
                file_path = candidate
        except Exception:
            pass

    if file_path is None or not os.path.isfile(file_path):
        raise FileNotFoundError(
            "Set AUV_VESSEL_DATA to vessel_data.yml, pass vessel_data_file param, or install "
            "auv_navigation share config."
        )

    with open(file_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if "gps_datum" not in data:
        data["gps_datum"] = [0.0, 0.0, 0.0]
    return data
