import numpy as np
import mujoco

from config import (
    LIDAR_NUM_RAYS,
    LIDAR_MAX_RANGE
)


class Lidar2D:

    def __init__(
        self,
        model,
        site_name="lidar_site"
    ):

        self.model = model

        self.site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            site_name
        )

        if self.site_id == -1:
            raise ValueError(
                f"LiDAR site '{site_name}' not found."
            )

        self.body_id = model.site_bodyid[
            self.site_id
        ]

        self.angles = np.linspace(
            -np.pi,
            np.pi,
            LIDAR_NUM_RAYS,
            endpoint=False
        )

        self.geomgroup = np.ones(
            6,
            dtype=np.uint8
        )

        self.geomgroup[1] = 0
  


    def scan(self, data):

        origin = data.site_xpos[
            self.site_id
        ].copy()

        rotation = data.site_xmat[
            self.site_id
        ].reshape(3, 3)

        ranges = []

        for angle in self.angles:

            # Ray direction in robot frame
            local_direction = np.array(
                [
                    np.cos(angle),
                    np.sin(angle),
                    0.0
                ],
                dtype=np.float64
            )

            # Convert to world frame
            world_direction = (
                rotation @ local_direction
            )

            geom_id = np.array(
                [-1],
                dtype=np.int32
            )

            distance = mujoco.mj_ray(
                self.model,
                data,
                origin,
                world_direction,
                self.geomgroup,
                1,
                int(self.body_id),
                geom_id,
                None
            )

            if (
                distance < 0.0
                or
                distance > LIDAR_MAX_RANGE
            ):
                distance = LIDAR_MAX_RANGE

            ranges.append(
                distance
            )

        return (
            self.angles.copy(),
            np.array(
                ranges,
                dtype=float
            )
        )