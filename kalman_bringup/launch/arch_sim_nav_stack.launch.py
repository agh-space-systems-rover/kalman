from kalman_bringup import *

RGBD_IDS = "d455_front d455_back d455_left d455_right"


def generate_launch_description():
    return gen_launch(
        {
            "description": {
                "layout": "autonomy",
            },
            "clouds": {
                "rgbd_ids": RGBD_IDS,
            },
            "slam": {"rgbd_ids": RGBD_IDS, "use_mag": "false"},
            # "nav2": {
            #     "rgbd_ids": RGBD_IDS,
            # },
            # "supervisor": {},
        }
    )
