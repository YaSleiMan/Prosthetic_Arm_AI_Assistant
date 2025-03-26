import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from spatialmath import SE3
import roboticstoolbox as rtb
from interfaces import FrankaStateInterface

# Load YOLO model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
bridge = CvBridge()

class ObjectLocalizationNode:
    def __init__(self):
        rospy.init_node('object_localization_node')

        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth', Image, self.depth_callback)
        self.target_obj_sub = rospy.Subscriber('/target_object', String, self.target_callback)
        self.coord_pub = rospy.Publisher('/object_coordinates', String, queue_size=10)

        self.latest_image = None
        self.latest_depth = None
        self.target_object = None

        self.panda = rtb.models.Panda()
        self.tool_offset = SE3(0.0, 0.0, 0.145)  # camera offset from end-effector
        self.fsi = FrankaStateInterface()  # assumes this uses latest joint state from ROS

    def target_callback(self, msg):
        self.target_object = msg.data.lower()
        rospy.loginfo(f'Target object set to: {self.target_object}')

    def image_callback(self, msg):
        self.latest_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.latest_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_to_base(self):
        joint_positions = np.array(self.fsi.joint_positions)
        base_to_ee = self.panda.fkine(joint_positions)
        return base_to_ee * self.tool_offset

    def process_image(self):
        if self.latest_image is None or self.latest_depth is None or self.target_object is None:
            return

        results = model(self.latest_image)
        detected_objects = results.pandas().xyxy[0]

        for _, row in detected_objects.iterrows():
            if row['name'].lower() == self.target_object:
                x_center = int((row['xmin'] + row['xmax']) / 2)
                y_center = int((row['ymin'] + row['ymax']) / 2)

                depth_value = self.latest_depth[y_center, x_center]
                if np.isnan(depth_value) or depth_value == 0:
                    rospy.logwarn(f'Invalid depth value for {self.target_object}')
                    continue

                # Convert pixel to 3D camera coordinates (simplified pinhole)
                # fx, fy = 600.0, 600.0  # Focal lengths (example values)
                # cx, cy = self.latest_image.shape[1] // 2, self.latest_image.shape[0] // 2
                # x = (x_center - cx) * depth_value / fx
                # y = (y_center - cy) * depth_value / fy
                # z = depth_value
                # point_cam = np.array([x, y, z])
                point_cam = np.array([x_center, y_center, depth_value])

                # Transform to base frame using FK
                T_cam = self.camera_to_base()
                point_base = T_cam.A @ np.append(point_cam, 1)
                coords = tuple(point_base[:3])

                rospy.loginfo(f'Object {self.target_object} found at: {coords}')
                self.coord_pub.publish(f'{coords[0]},{coords[1]},{coords[2]}')
                break

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.process_image()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ObjectLocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
