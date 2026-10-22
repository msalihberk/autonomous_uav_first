import os
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from uav.topic_names import TopicNames
from custom_interfaces.msg import CamData
from custom_interfaces.srv import Coordinate, MovePoint
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from custom_interfaces.srv import Float64

SERVICE_TIMEOUT_SEC = 2.0

class SetYawService(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("SetYawService")

        self.blackboard = self.attach_blackboard_client("SetYawService Blackboard")
        self.blackboard.register_key(key="angle", access=py_trees.common.Access.READ)
        self.node = Node("TMP")
        self.future = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.client = self.node.create_client(Float64, TopicNames.SET_YAW_TOPIC)

    def update(self):
        if not self.blackboard.exists('angle'):
            return py_trees.common.Status.FAILURE

        if self.future is None:
            if not self.client.wait_for_service(timeout_sec=0.1):
                return py_trees.common.Status.RUNNING

            data = self.blackboard.get('angle')

            msg = Float64.Request()
            msg.data = data

            self.future = self.client.call_async(msg)
            self.start_time = self.node.get_clock().now()
            return py_trees.common.Status.RUNNING
        
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > SERVICE_TIMEOUT_SEC:
            return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.node.get_logger().error(f"SetYaw Service Error: {e}")
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.future = None

class AngleCalculationServer(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("AngleCalculationServer")

        self.blackboard = self.attach_blackboard_client("AngleCalculationServer Blackboard")
        self.blackboard.register_key(key="cam_data", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="angle", access=py_trees.common.Access.WRITE)
        self.node = Node("TMP")
        self.future = None
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.client = self.node.create_client(Coordinate, TopicNames.ANGLE_CAL_TOPIC)

    def update(self):
        if not self.blackboard.exists('cam_data'):
            self.node.get_logger().info("No camera data available.")
            return py_trees.common.Status.FAILURE

        if self.future is None:
            if not self.client.wait_for_service(timeout_sec=0.1):
                return py_trees.common.Status.RUNNING

            data = self.blackboard.get('cam_data')

            msg = Coordinate.Request()
            msg.img_width = data.img_width
            msg.img_height = data.img_height
            msg.center_x = data.center_x
            msg.center_y = data.center_y

            self.future = self.client.call_async(msg)
            self.start_time = self.node.get_clock().now()
            return py_trees.common.Status.RUNNING
        
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > SERVICE_TIMEOUT_SEC:
            self.node.get_logger().error(f"Angle Calculation Service Timeout after {elapsed:.2f} seconds.")
            return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.node.get_logger().error(f"Angle Calculation Service Error: {e}")
                return py_trees.common.Status.FAILURE
            else:
                self.blackboard.set('angle', response.angle) 
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.future = None

class MovementServer(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("MovementServer")

        self.blackboard = self.attach_blackboard_client("MovementServer Blackboard")
        self.blackboard.register_key(key="target_coordinates", access=py_trees.common.Access.READ)
        self.node = Node("TMP")
        self.future = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.client = self.node.create_client(MovePoint, TopicNames.MOVEMENT_TOPIC)

    def update(self):
        if not self.blackboard.exists('target_coordinates'):
            return py_trees.common.Status.FAILURE

        if self.future is None:
            if not self.client.wait_for_service(timeout_sec=0.1):
                return py_trees.common.Status.RUNNING

            data = self.blackboard.get('target_coordinates')

            msg = MovePoint.Request()
            msg.point = data

            self.future = self.client.call_async(msg)
            self.start_time = self.node.get_clock().now()
            return py_trees.common.Status.RUNNING
        
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > SERVICE_TIMEOUT_SEC:
            return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.node.get_logger().error(f"Coordinate Service Error: {e}")
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.future = None

class CoordinateServer(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("CoordinateServer")

        self.blackboard = self.attach_blackboard_client("CoordinateServer Blackboard")
        self.blackboard.register_key(key="cam_data", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_coordinates", access=py_trees.common.Access.WRITE)
        self.node = Node("TMP")
        self.future = None
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.client = self.node.create_client(Coordinate, TopicNames.COORDINATE_CAL_TOPIC)

    def update(self):
        if not self.blackboard.exists('cam_data'):
            self.node.get_logger().info("No camera data available.")
            return py_trees.common.Status.FAILURE

        if self.future is None:
            if not self.client.wait_for_service(timeout_sec=0.1):
                return py_trees.common.Status.RUNNING

            data = self.blackboard.get('cam_data')

            msg = Coordinate.Request()
            msg.img_width = data.img_width
            msg.img_height = data.img_height
            msg.center_x = data.center_x
            msg.center_y = data.center_y

            self.future = self.client.call_async(msg)
            self.start_time = self.node.get_clock().now()
            return py_trees.common.Status.RUNNING
        
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > SERVICE_TIMEOUT_SEC:
            self.node.get_logger().error(f"Coordinate Service Timeout after {elapsed:.2f} seconds.")
            return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.node.get_logger().error(f"Coordinate Service Error: {e}")
                return py_trees.common.Status.FAILURE
            else:
                self.blackboard.set('target_coordinates', response.target_point) 
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.future = None
        

class DataTimeOutController(py_trees.behaviour.Behaviour):
    def __init__(self, timeout_sec=2.5):
        super().__init__("DataTimeOutController")
        self.timeout_sec = timeout_sec
        self.blackboard = self.attach_blackboard_client("Timeout Blackboard")
        self.blackboard.register_key(key="cam_data", access=py_trees.common.Access.WRITE)
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs['node']

    def update(self):
        if not self.blackboard.exists('cam_data'):
            self.node.get_logger().info("No camera data available.")
            return py_trees.common.Status.FAILURE

        data = self.blackboard.get('cam_data')

        msg_time = Time.from_msg(data.header.stamp)
        current_time = self.node.get_clock().now()

        elapsed_sec = (current_time - msg_time).nanoseconds / 1e9
        
        if elapsed_sec > self.timeout_sec:
            self.node.get_logger().info(f"Camera data timeout: {elapsed_sec:.2f} seconds elapsed.")
            self.blackboard.unset('cam_data') 
            return py_trees.common.Status.FAILURE
            
        return py_trees.common.Status.SUCCESS

def create_tree():
    root = py_trees.composites.Selector(
        name="Root Selector",
        memory=False
    )

    cam_data = py_trees_ros.subscribers.ToBlackboard(
        name="CamData Listener",
        topic_name=TopicNames.CAM_DATA_TOPIC,
        topic_type=CamData,
        blackboard_variables="cam_data",
        qos_profile=qos_profile_sensor_data,
        clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
    )

    movement_sequence = py_trees.composites.Sequence(
        name="Movement Sequence",
        memory=False
    )
    movement_sequence.add_children([cam_data, DataTimeOutController(), CoordinateServer(), AngleCalculationServer(), SetYawService(), MovementServer()])

    root.add_children([movement_sequence])

    return root



def live_tree_callback(tree):
    os.system('clear')
    print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

def main(args=None):
    rclpy.init(args=args)
    root = create_tree()
    
    executor = MultiThreadedExecutor()
    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.add_post_tick_handler(live_tree_callback)
    
    tree.setup(timeout=15.0)
    
    try:
        tree.tick_tock(period_ms=500)
        executor.add_node(tree.node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

