"""
https://github.com/Abstract-Horizon/vl53l5cx_python/issues/1

FIXME: Rewrite driver cuz when there is no object inthe FoV, the depth
       values stays at the last moment when there is an object.
       Expected behaviour is return NaN or minus values.

I2C rdwr reg addr format on the datasheet p.12
https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library/blob/main/src/SparkFun_VL53L5CX_IO.cpp#L58-L60
https://stackoverflow.com/questions/70753284/how-can-i-read-two-bytes-from-two-registers-in-a-device-within-i2c
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from vl53l5cx.vl53l5cx import VL53L5CX, VL53L5CX_RESOLUTION_8X8


class VL53L5CXPublisher(Node):
    def __init__(self):
        super().__init__("vl53l5cx")

        self.vl53l5cx = VL53L5CX(bus_id=0)
        self.vl53l5cx.init()
        self.vl53l5cx.set_resolution(VL53L5CX_RESOLUTION_8X8)
        self.vl53l5cx.start_ranging()

        self.publisher = self.create_publisher(PointCloud2, "/vl53l5cx", 10)
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        if not self.vl53l5cx.check_data_ready():
            return
        data = self.vl53l5cx.get_ranging_data()
        distance_mm = np.array(data.distance_mm).reshape(8, 8)
        print(distance_mm)

        points2 = PointCloud2(
            header = Header(
                stamp = self.get_clock().now().to_msg(),
                frame_id = "vl53l5cx_link"),
            height = 8,
            width = 8,
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)],
            is_bigendian = False,
            is_dense = False,
        )
        points2.point_step = 4 * len(points2.fields)
        points2.row_step = points2.point_step * 8

        # Calc x,y,z coordinates for VL53L5CX data
        # https://forum.dronebotworkshop.com/sensors-modules/time-of-flight-tof-vl53l5cx-8x8-pixel-sensor/
        # https://community.st.com/s/question/0D53W000015XpcBSAS/vl53l5cx-multizone-sensor-get-xyz-of-points-relative-to-origin
        # https://community.st.com/s/question/0D53W00001NTlTzSAL/for-vl53l5cx-what-does-resultsdatadistancemmzonenum-exactly-mean
        # FIXME: The algo is not correct
        buf = np.empty((8, 8, len(points2.fields)), dtype=np.float32)
        it = np.nditer(distance_mm, flags=["multi_index"])
        per_px = np.deg2rad(45) / 8
        for e in it:
            w, h = it.multi_index

            x = e * np.cos(w * per_px - np.deg2rad(45)/2 - np.deg2rad(90)) / 1_000
            y = e * np.sin(h * per_px - np.deg2rad(45)/2) / 1_000
            z = e / 1_000

            buf[w][h] = [x, y, z]
        points2.data = buf.tobytes()

        self.publisher.publish(points2)


def process(args=None):
    rclpy.init(args=args)
    publisher = VL53L5CXPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    process()
