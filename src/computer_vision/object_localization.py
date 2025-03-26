import rospy
import cv2
import torch
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Load YOLO model (adjust for desired version)
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
bridge = CvBridge()

def transform_camera_to_base(camera_point, tf_buffer):
    """
    Transforms a 3D point from the camera frame to the robot base frame using TF2.

    Parameters:
    - camera_point (x, y, z): Point in the camera frame.
    - tf_buffer: tf2_ros.Buffer() object for lookup

    Returns:
    - tuple: (x, y, z) in base frame, or None if transform fails
    """
    point_msg = PointStamped()
    point_msg.header.stamp = rospy.Time.now()
    point_msg.header.frame_id = "camera_link"  # Use your actual camera frame
    point_msg.point.x = float(camera_point[0])
    point_msg.point.y = float(camera_point[1])
    point_msg.point.z = float(camera_point[2])

    try:
        transformed_point = tf_buffer.transform(point_msg, "base_link", timeout=rospy.Duration(1.0))
        return (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
        rospy.logwarn(f"TF transform failed: {e}")
        return None

class ObjectLocalizationNode:
    def __init__(self):
        rospy.init_node('object_localization_node')
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth', Image, self.depth_callback)
        self.target_obj_sub = rospy.Subscriber('/target_object', String, self.target_callback)
        self.coord_pub = rospy.Publisher('/object_coordinates', String, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.latest_image = None
        self.latest_depth = None
        self.target_object = None

    def target_callback(self, msg):
        self.target_object = msg.data.lower()
        rospy.loginfo(f'Target object set to: {self.target_object}')

    def image_callback(self, msg):
        self.latest_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.latest_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def process_image(self):
        if self.latest_image is None or self.latest_depth is None or self.target_object is None:
            return

        results = model(self.latest_image)
        detected_objects = results.pandas().xyxy[0]  # Get detection results as Pandas DataFrame

        for _, row in detected_objects.iterrows():
            if row['name'].lower() == self.target_object:
                x_center = int((row['xmin'] + row['xmax']) / 2)
                y_center = int((row['ymin'] + row['ymax']) / 2)

                depth_value = self.latest_depth[y_center, x_center]
                if np.isnan(depth_value) or depth_value == 0:
                    rospy.logwarn(f'Invalid depth value for {self.target_object}')
                    continue

                object_cam_coords = (x_center, y_center, depth_value)
                object_base_coords = transform_camera_to_base(object_cam_coords, self.tf_buffer)
                if object_base_coords is None:
                    continue

                rospy.loginfo(f'Object {self.target_object} found at: {object_base_coords}')
                self.coord_pub.publish(f'{object_base_coords[0]},{object_base_coords[1]},{object_base_coords[2]}')
                break

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_image()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ObjectLocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
