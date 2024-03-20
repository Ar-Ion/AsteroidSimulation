import omni.graph.core as og
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims

class DynamicObject:

    def __init__(self, prim_path):
        self._prim_path = prim_path

    def start_tf_publisher(self):
        if not is_prim_path_valid(self._prim_path):
            raise ValueError(f"Prim path '{self._prim_path}' is invalid.")

        try:
            # Generate the frame_id. OmniActionGraph will use the last part of
            # the full camera prim path as the frame name, so we will extract it here
            # and use it for the pointcloud frame_id.
            frame_id=self._prim_path.split("/")[-1]

            # Generate an action graph associated with camera TF publishing.
            ros_tf_graph_path = "/TFActionGraph"

            # If a camera graph is not found, create a new one.
            if not is_prim_path_valid(ros_tf_graph_path):
                (ros_camera_graph, _, _, _) = og.Controller.edit(
                    {
                        "graph_path": ros_tf_graph_path,
                        "evaluator_name": "execution",
                        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                    },
                    {
                        og.Controller.Keys.CREATE_NODES: [
                            ("OnTick", "omni.graph.action.OnTick"),
                            ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                            ("RosPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                        ],
                        og.Controller.Keys.CONNECT: [
                            ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                            ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                        ]
                    }
                )

            # Generate 2 nodes associated with each camera: TF from world to ROS camera convention, and world frame.
            og.Controller.edit(
                ros_tf_graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("PublishTF_"+frame_id, "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                        ("PublishRawTF_"+frame_id+"_world", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF_"+frame_id+".inputs:topicName", "/tf"),
                        # Note if topic_name is changed to something else besides "/tf",
                        # it will not be captured by the ROS tf broadcaster.
                        ("PublishRawTF_"+frame_id+"_world.inputs:topicName", "/tf"),
                        ("PublishRawTF_"+frame_id+"_world.inputs:parentFrameId", frame_id),
                        ("PublishRawTF_"+frame_id+"_world.inputs:childFrameId", frame_id+"_world"),
                        # Static transform from ROS camera convention to world (+Z up, +X forward) convention:
                        ("PublishRawTF_"+frame_id+"_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (ros_tf_graph_path+"/OnTick.outputs:tick",
                            "PublishTF_"+frame_id+".inputs:execIn"),
                        (ros_tf_graph_path+"/OnTick.outputs:tick",
                            "PublishRawTF_"+frame_id+"_world.inputs:execIn"),
                        (ros_tf_graph_path+"/IsaacClock.outputs:simulationTime",
                            "PublishTF_"+frame_id+".inputs:timeStamp"),
                        (ros_tf_graph_path+"/IsaacClock.outputs:simulationTime",
                            "PublishRawTF_"+frame_id+"_world.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Add target prims for the USD pose. All other frames are static.
        set_target_prims(
            primPath=ros_tf_graph_path+"/PublishTF_"+frame_id,
            inputName="inputs:targetPrims",
            targetPrimPaths=[self._prim_path],
        )