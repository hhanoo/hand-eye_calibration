import os

config = {
    # Data Path
    "data_path": os.path.join(os.path.dirname(__file__).joinpath(".."), "data"),
    "calibration_path": os.path.join(os.path.dirname(__file__).joinpath(".."), "data", "calibration_result.txt"),
    "calibration_ransac_path": os.path.join(os.path.dirname(__file__).joinpath(".."), "data", "calibration_result_ransac.txt"),

    # Marker
    "marker_grid_size": 6,
    "num_marker_ids": 250,
}
