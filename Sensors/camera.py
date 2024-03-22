import omni
import numpy as np
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils

from Sensors.sensor_base import Sensor
from dynamics import DynamicObject

class ROSCamera(Sensor, DynamicObject):
    def __init__(self, name, resolution, fps):
        self._name = name
        self._resolution = resolution
        self._fps = fps

        self._camera = Camera(
            name=name,
            prim_path="/World/"+name,
            position=np.array([0.0, -3.0, 0.0]),
            frequency=fps,
            resolution=resolution,
            orientation=rot_utils.euler_angles_to_quats(np.array([90, 0, 90]), degrees=True),
        )

        DynamicObject.__init__(self, self._camera.prim_path)

    def _start_publisher(self, freq, topic, pub_type):
        # The following code will link the camera's render product and publish the data to the specified topic name.
        render_product = self._camera._render_product_path
        step_size = int(60/freq)
        topic_name = self._camera.name+"/"+topic
        queue_size = 1
        node_namespace = ""
        frame_id = self._camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

        writer = rep.writers.get(pub_type)
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", render_product
        )

        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    def _start_camera_info_publisher(self, freq):
        pub_type = "ROS2PublishCameraInfo"
        self._start_publisher(freq, "info", pub_type)

    def _start_rgb_publisher(self, freq):
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        pub_type = rv + "ROS2PublishImage"
        self._start_publisher(freq, "rgb", pub_type)

    def _start_depth_publisher(self, freq):
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        pub_type = rv + "ROS2PublishImage"
        self._start_publisher(freq, "depth", pub_type)

    def start_sensor(self):

        self._camera.initialize()

        # OpenCV camera matrix and width and height of the camera sensor, from the calibration file
        width, height = self._resolution
        camera_matrix = [[958.8, 0.0, 957.8], [0.0, 956.7, 589.5], [0.0, 0.0, 1.0]]

        # Pixel size in microns, aperture and focus distance from the camera sensor specification
        # Note: to disable the depth of field effect, set the f_stop to 0.0. This is useful for debugging.
        pixel_size = 3 * 1e-3   # in mm, 3 microns is a common pixel size for high resolution cameras
        f_stop = 1.8            # f-number, the ratio of the lens focal length to the diameter of the entrance pupil
        focus_distance = 3      # in meters, the distance from the camera to the object plane

        # Calculate the focal length and aperture size from the camera matrix
        ((fx,_,cx),(_,fy,cy),(_,_,_)) = camera_matrix
        horizontal_aperture =  pixel_size * width                   # The aperture size in mm
        vertical_aperture =  pixel_size * height
        focal_length_x  = fx * pixel_size
        focal_length_y  = fy * pixel_size
        focal_length = (focal_length_x + focal_length_y) / 2         # The focal length in mm

        # Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
        self._camera.set_focal_length(focal_length / 10.0)                # Convert from mm to cm (or 1/10th of a world unit)
        self._camera.set_focus_distance(focus_distance)                   # The focus distance in meters
        self._camera.set_lens_aperture(f_stop * 100.0)                    # Convert the f-stop to Isaac Sim units
        self._camera.set_horizontal_aperture(horizontal_aperture / 10.0)  # Convert from mm to cm (or 1/10th of a world unit)
        self._camera.set_vertical_aperture(vertical_aperture / 10.0)

        self._camera.set_clipping_range(0.1, 1.0e5)

        self._start_camera_info_publisher(10)
        self._start_rgb_publisher(30)
        self._start_depth_publisher(30)
        self.start_tf_publisher()

    def tick_sensor(self):
        pass