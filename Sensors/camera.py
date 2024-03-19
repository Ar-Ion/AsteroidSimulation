from Sensors.sensor_base import Sensor

import omni
import numpy as np
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage, extensions, nucleus
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims

class ROSCamera(Sensor):
    def __init__(self, name, resolution, fps):

        self._camera = Camera(
            name=name,
            prim_path="/World/"+name,
            position=np.array([0.0, -10.0, 0.0]),
            frequency=fps,
            resolution=resolution,
            orientation=rot_utils.euler_angles_to_quats(np.array([90, 0, 90]), degrees=True),
        )

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
        self._start_publisher(freq, "camera_info", pub_type)

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
        self._start_camera_info_publisher(10)
        self._start_rgb_publisher(30)
        self._start_depth_publisher(10)

    def tick_sensor(self):
        pass