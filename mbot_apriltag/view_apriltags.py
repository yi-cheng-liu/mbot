import cv2
import time
import numpy as np
import yaml
from dt_apriltags import Detector
from gst_cam import camera
from mbot_lcm_msgs.pose2D_t import pose2D_t

#Coordinate system: 
# Camera: Z = away from camera, X = to the right of image of camera, Y = towards the ground of image of camera
# Tag: Z = into tag, X = to the right of tag, Y = towards the ground of tag


SMALL_TAG_IDS = [10, 20, 30, 40]
SMALL_TAG_SIZE_MM = 10.8
LARGE_TAG_IDS = [1, 2, 3, 4]
LARGE_TAG_SIZE_MM = 54

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

H_FOV_DEG = 62.2

cur_pose = pose2D_t()
block_pose = pose2D_t()

# Load camera parameters from yaml
def get_camera_params():
    with open("camera_params.yaml", 'r') as f:
        params = yaml.safe_load(f)

    camera_matrix = np.array(params['camera_matrix'], dtype=np.float32)
    K = camera_matrix
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    camera_params = [fx, fy, cx, cy]
    distortion_coefficients = np.array(params['distortion_coefficients'], dtype=np.float32)
    return camera_matrix, camera_params, distortion_coefficients

#Using pose R and t of detected apriltags, return the angle and distance of the tag from the camera
# Angle = angle between camera's Z axis and tag's Z axis, in degrees
# Distance = XZ plane distance between camera and tag, in meters
def get_angle_and_dist_of_tag(tag_id, pose_R_mat, pose_t_mat):
    distance = np.sqrt(float(pose_t_mat[0]**2 + pose_t_mat[2]**2))
    if tag_id in SMALL_TAG_IDS:
        distance *= (SMALL_TAG_SIZE_MM / LARGE_TAG_SIZE_MM) #Detector assumes everything is large tag size. Scale down distance for small tags
    yaw = np.arctan2(pose_R_mat[0][2], pose_R_mat[2][2]) * 180 / np.pi
    return distance, yaw
    
def main():
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
        # print(tags)

        for det in tags:
            if (det.tag_id == 6):
                # Draw bounding box
                for i in range(4):
                    start_point = tuple(det.corners[i-1].astype(int))
                    end_point = tuple(det.corners[i].astype(int))
                    cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

                # Draw tag family and ID on the image
                tag_info = "ID:{}".format(det.tag_id)
                dist, angle = get_angle_and_dist_of_tag(det.tag_id, det.pose_R, det.pose_t)

                cv2.putText(frame, tag_info, (int(det.center[0]), int(det.center[1])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                
                hFOV = np.deg2rad(H_FOV_DEG)
                center_h, center_v = det.center[0], det.center[1]
                distance_to_block = dist

                angle_per_pixel_h = hFOV / CAMERA_WIDTH
                offset_h = -(center_h - CAMERA_WIDTH / 2)

                # TODO: add robot theta
                robot_theta = 0
                angle_offset_h = offset_h * angle_per_pixel_h + robot_theta

                tag_pose_x = distance_to_block * np.cos(angle_offset_h)
                tag_pose_y = distance_to_block * np.sin(angle_offset_h)
                print(f"Tag {det.tag_id}: angle_offset: {angle_offset_h:.4f}, distance: {dist:.4f}, pose: ({tag_pose_x:.4f}, {tag_pose_y:.4f})")
                
                # calculate block center
                block_size = 0.05
                block_pose_x = tag_pose_x + block_size * np.cos(angle_offset_h + np.deg2rad(angle))
                block_pose_y = tag_pose_y + block_size * np.sin(angle_offset_h + np.deg2rad(angle))
                print(f"Tag {det.tag_id}: Angle: {angle:.4f} pose: ({block_pose_x:.4f}, {block_pose_y:.4f})")
                
                # target pose
                tag_block_center_dist_x = block_pose_x - tag_pose_x
                tag_block_center_dist_y = block_pose_y - tag_pose_y
                far_coef = 5
                target_pose_x_1 = block_pose_x - far_coef * tag_block_center_dist_x
                target_pose_y_1 = block_pose_y + far_coef * tag_block_center_dist_y
                target_pose_x_2 = block_pose_x + far_coef * tag_block_center_dist_x
                target_pose_y_2 = block_pose_y - far_coef * tag_block_center_dist_y
                print(f"                      tar1: ({target_pose_x_1:.4f}, {target_pose_y_1:.4f})")
                print(f"                      tar2: ({target_pose_x_2:.4f}, {target_pose_y_2:.4f})")

        # cv2.imshow("Camera", frame)

        key = cv2.waitKey(10)
        if key == ord('q'):
            break

    # cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    main()