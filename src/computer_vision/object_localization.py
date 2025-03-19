import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Load YOLO model (adjust for desired version)
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
bridge = CvBridge()

# Define camera to base transformation matrix (Update these values based on actual setup)
translation = np.array([0.0, 0.0, 0.0])  # (x, y, z) offset in meters

rotation_matrix = np.array([[1, 0, 0],  # Identity matrix (assuming no rotation)
                            [0, 1, 0],
                            [0, 0, 1]])

# Construct 4x4 transformation matrix
transformation_matrix = np.eye(4)
transformation_matrix[:3, :3] = rotation_matrix
transformation_matrix[:3, 3] = translation

def transform_camera_to_base(object_cam_coords, transformation_matrix):
    """
    Transforms object coordinates from the camera frame to the robot base frame.
    Parameters:
    - object_cam_coords (tuple): (x, y, z) coordinates of the object in the camera frame.
    - transformation_matrix (numpy array): 4x4 transformation matrix from camera to base frame.
    Returns:
    - tuple: (x, y, z) coordinates in the robot base frame.
    """
    object_cam_homogeneous = np.array([object_cam_coords[0], object_cam_coords[1], object_cam_coords[2], 1])
    object_base_homogeneous = np.dot(transformation_matrix, object_cam_homogeneous)
    return tuple(object_base_homogeneous[:3])

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
                object_base_coords = transform_camera_to_base(object_cam_coords, transformation_matrix)
                
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
