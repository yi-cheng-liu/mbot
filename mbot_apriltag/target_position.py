import numpy as np
import cv2
import time
import lcm
from view_apriltags import get_camera_params, get_angle_and_dist_of_tag
from dt_apriltags import Detector
from gst_cam import camera

from mbot_lcm_msgs.twist2D_t import twist2D_t
from mbot_lcm_msgs.pose2D_t import pose2D_t
from mbot_lcm_msgs.path2D_t import path2D_t

SMALL_TAG_IDS = [10, 20, 30, 40]
SMALL_TAG_SIZE_MM = 10.8
LARGE_TAG_IDS = [1, 2, 3, 4]
LARGE_TAG_SIZE_MM = 54

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

H_FOV_DEG = 62.2

PREFERED_ID = 6
SECONDARY_ID = 60

# Initialize poses as pose2D_t
cur_pose = pose2D_t() # robot cur pose

tag_pose = pose2D_t()
block_pose = pose2D_t()
target_pose_close = pose2D_t()
target_pose_far = pose2D_t()

""" Tags """
def get_tags():
    camera_matrix, camera_params, distortion_coefficients = get_camera_params()
    cap = cv2.VideoCapture(camera(0, CAMERA_WIDTH, CAMERA_HEIGHT))

    time.sleep(3)

    # Initialize AprilTag detector
    detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                            families='tagCustom48h12',
                            nthreads=4,
                            quad_decimate=2,
                            quad_sigma=0.4,
                            refine_edges=1,
                            decode_sharpening=1,
                            max_hamming=1,
                            debug=0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)

        # Detect AprilTags
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray, True, camera_params, LARGE_TAG_SIZE_MM * (1/1000))
        return tags
    
    return None

def prioritize_tags(tags_data, preferred_id=PREFERED_ID, secondary_id=SECONDARY_ID):
    for tag in tags_data:
        if tag.tag_id == preferred_id:
            return tag
        elif tag.tag_id == secondary_id:
            return tag
        
        # TODO: Make this another algo
        return tag

""" LCM message """
def odometry_handler(channel, data):
    global cur_pose
    msg = pose2D_t.decode(data)
    cur_pose.x = msg.x
    cur_pose.y = msg.y
    cur_pose.theta = msg.theta
    print("Received message on channel \"%s\"" % channel)
    print("   x   = %s" % str(msg.x))
    print("   y   = %s" % str(msg.y))
    print(" theta = %s" % str(msg.theta))
    print("")

""" Block Info"""
def get_tag_pose(tag_distance, tag_yaw, center_h, tag_to_block_offset=0.06):
    global cur_pose
    
    # Find the degree of the tag (h is horizontal)
    hFOV = np.deg2rad(H_FOV_DEG)
    angle_per_pixel_h = hFOV / CAMERA_WIDTH
    offset_h = -(center_h - CAMERA_WIDTH / 2) # in pixel
    angle_offset_h = offset_h * angle_per_pixel_h + cur_pose.theta # rad
    
    tag_pose.x = tag_distance * np.cos(angle_offset_h)
    tag_pose.y = tag_distance * np.sin(angle_offset_h)
    tag_pose.theta = angle_offset_h
       
def get_block_pose(tag_yaw, tag_to_block_dist=0.06):
    global block_pose
    block_pose.theta = tag_pose.theta + np.deg2rad(tag_yaw)
    block_pose.x = tag_pose.x + tag_to_block_dist * np.cos(block_pose.theta)
    block_pose.y = tag_pose.y + tag_to_block_dist * np.sin(block_pose.theta)

def get_target_pose(far_coef=5):
    tag_block_center_dist_x = block_pose.x - tag_pose.x
    tag_block_center_dist_y = block_pose.y - tag_pose.y
    
    # The closer pose
    target_pose_close.x = block_pose.x - far_coef * tag_block_center_dist_x
    target_pose_close.y = block_pose.y + far_coef * tag_block_center_dist_y
    target_pose_close.theta = block_pose.theta
    # The far pose
    target_pose_far.x = block_pose.x + far_coef * tag_block_center_dist_x
    target_pose_far.y = block_pose.y - far_coef * tag_block_center_dist_y
    target_pose_close.theta = block_pose.theta
    
def main():
    print("==================    Start    ==================")
    # Get the current position
    print("==================     LCM     ==================")
    lc = lcm.LCM()
    subscription = lc.subscribe("MBOT_ODOMETRY", odometry_handler)
    
    # Detect Apriltags
    print("==================  Apriltags  ==================")
    tags = get_tags() # get all tags
    # print(tags)
    
    # Perform calculation
    try:
        lc.handle_timeout(1000)
        if tags:
            tag = prioritize_tags(tags)
            if tag.tag_id:
                # Tag
                tag_distance, tag_yaw = get_angle_and_dist_of_tag(tag.tag_id, tag.pose_R, tag.pose_t)
                center_h, center_v = tag.center[0], tag.center[1]
                get_tag_pose(tag_distance, tag_yaw, center_h)
                
                # Block
                get_block_pose(tag_yaw)
                
                
                print("==================    Pose    ==================")
                print(f"Tag {tag.__doc__tag_id} detected(5 is large, 50 is small)")
                print(f"  Tag distance: {tag_distance} Tag yaw: {tag_yaw}")
                print(f"Current Pose: {cur_pose.x:.4f}, {cur_pose.y:.4f}, {cur_pose.theta:.4f}")
                print(f"Block Pose:   {block_pose.x:.4f}, {block_pose.y:.4f}, {block_pose.theta:.4f}")
                
                # find target pose according to block pose
                get_target_pose()
                print(f"Target Pose:  {target_pose_close.x:.4f}, {target_pose_close.y:.4f}, {target_pose_close.theta:.4f}")
                
                # Publish
                path = path2D_t()
                path.path.append(cur_pose)
                path.path.append(target_pose_close)
                path.path_length = 2
                                
                lc.publish("CONTROLLER_PATH", path.encode())

                
        else:
            print("No tags detected. ")
                
    except KeyboardInterrupt:
        print("doesn't receive LCM message")
        pass
    print("=====================  End  =====================")

if __name__ == '__main__':
    main()