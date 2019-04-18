# Import the necessary Python modules
import rospy # rospy - ROS Python API
import intera_interface # intera_interface - Sawyer Python API
# for x,y,z interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from collections import deque
import numpy as np
import argparse
import imutils
import cv2 # display for camera

''' ################################################# '''
'''   Global Initialization of Sawyer Arm Functions   '''
''' ################################################# '''

# initialize our ROS node, registering it with the Master
rospy.init_node('Move_Sawyer')
# create an instance of intera_interface's Limb class
limb = intera_interface.Limb('right')
# create instance for gripper class
gripper = intera_interface.Gripper('right_gripper')

# Enable robot
robot_state = intera_interface.RobotEnable()
robot_state.enable() # enable with INFO output

# joint angles are in randians, standard arm positions (central, nuetral)
default_view_pose = {'right_j6': 3.140002173613553,
					'right_j5': 0.5699964613771344,
					'right_j4': -1.6103727593197448e-05,
					'right_j3': 2.1800525678571123,
					'right_j2': 8.969925566759684e-06,
					'right_j1': -1.1799162122493065,
					'right_j0': -1.4938956081067545e-06}

# move robot arm to the neutral position, to the side to clear arm obstruction
camera_view_pose = {'right_j6': 3.140002173613553,
					'right_j5': 0.5699964613771344,
					'right_j4': -1.6103727593197448e-05,
					'right_j3': 2.1800525678571123,
					'right_j2': 8.969925566759684e-06,
					'right_j1': -1.1799162122493065,
					'right_j0': -1.6}

# Default values for object location and color
object_color = "NULL"
object_location = (0.5,0.0)
sawyer_z_plane = -0.15 # default at .15 below table, need to hardcode top of table


''' #################### '''
'''   Camera Functions   '''
''' #################### '''
def camera_function(obj_loc, obj_color):
    # define the lower and upper boundaries of the colors in the HSV color space
    lower = {'red':(-10, 84, 141), 'green':(50, 122, 129), 'blue':(97, 100, 117)}
    upper = {'red':(10,255,255), 'green':(75,255,255), 'blue':(127,255,255)}

    camera = cv2.VideoCapture(0)

    # keep looping
    while obj_color == "NULL":
        # move the right arm to those joint angles
        limb.move_to_joint_positions(camera_view_pose)

        # grab the current frame
        (grabbed, frame) = camera.read()

        # resize the frame, blur it, and convert it to the HSV color space
        frame = imutils.resize(frame, width=600)

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        #for each color in dictionary check object in frame
        for key, value in upper.items():
            # construct a mask for the color from dictionary`1, then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            kernel = np.ones((9,9),np.uint8)
            mask = cv2.inRange(hsv, lower[key], upper[key])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # find contours in the mask and initialize the current
            # (x, y) center of the object
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing object and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((obj_x, obj_y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size. Correct this value for your obect's size
                if radius > 0.5:
                    obj_color = key
                    obj_loc = (obj_x, obj_y) #position of the center of the object in pixels (x,y)
                    print("Object Color: %s" % object_color)
                    print("Distance: ", object_location)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF


''' #################### '''
'''  Inverse Kinematics  '''
''' #################### '''
def IK(object_location):
    ik_service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    # create desired position in x,y,z (in meters)
    desired_pose = PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=object_location[0], # forward/backward direction (default 0.7)
                        y=object_location[1], # left/right direction
                        z=sawyer_z_plane, # up/down direction
                    ),
                    orientation=Quaternion(
                        x=0.704020578925,
                        y=0.710172716916,
                        z=0.00244101361829,
                        w=0.00194372088834,
                    )
                )
            )

    # request desired position to ik
    ikreq.pose_stamp.append(desired_pose)

    # assign gripper hand
    ikreq.tip_names.append('right_gripper_tip')

    # apply IK request as response
    resp = iksvc(ikreq)

    # print resp

    # test target position
    target_pose = dict(zip(resp.joints[0].name, resp.joints[0].position))

    # move arm to target position
    # dir(assignment) to see possible joint commands
    limb.move_to_joint_positions(target_pose)
    if limb.move_to_joint_positions(target_pose) == limb.joint_angles():
        gripper.close()

''' ################### '''
'''    Bin Locations    '''
''' ################### '''
def bin_locations():
    # to set positions (hard-code)
    red_bin_pose = limb.joint_angles()
    green_bin_pose = limb.joint_angles()
    blue_bin_pose = limb.joint_angles()

    # red_bin_pose = {
    # 					'right_j6': 3.140002173613553,
    # 					'right_j5': 0.5699964613771344,
    # 					'right_j4': -1.6103727593197448e-05,
    # 					'right_j3': 2.1800525678571123,
    # 					'right_j2': 8.969925566759684e-06,
    # 					'right_j1': -1.1799162122493065,
    # 					'right_j0': -1.4938956081067545e-06}
    # green_bin_pose = {
    # 					'right_j6': 3.140002173613553,
    # 					'right_j5': 0.5699964613771344,
    # 					'right_j4': -1.6103727593197448e-05,
    # 					'right_j3': 2.1800525678571123,
    # 					'right_j2': 8.969925566759684e-06,
    # 					'right_j1': -1.1799162122493065,
    # 					'right_j0': -1.4938956081067545e-06}
    # blue_bin_pose = {
    # 					'right_j6': 0.0,
    # 					'right_j5': 0.5699964613771344,
    # 					'right_j4': -1.6103727593197448e-05,
    # 					'right_j3': 2.1800525678571123,
    # 					'right_j2': 8.969925566759684e-06,
    # 					'right_j1': -1.1799162122493065,
    # 					'right_j0': -1.4938956081067545e-06}



''' #################### '''
'''         MAIN         '''
''' #################### '''
def main():
    for x in range(1,3):
        camera_function(*object_location, *object_color)
        IK(&object_location)
        limb.move_to_joint_positions(default_view_pose)

        # Object color detected and retrieved, move to correct bin and release object
        if object_color == "red":
            limb.move_to_joint_positions(red_bin_pose)
            if(limb.joint_angles() == red_bin_pose):
                gripper.open())
            object_color = "NULL"
        elif object_color == "green":
            limb.move_to_joint_positions(green_bin_pose)
            if(limb.joint_angles() == green_bin_pose):
                gripper.open())
            object_color = "NULL"
        elif object_color == "blue":
            limb.move_to_joint_positions(blue_bin_pose)
            if(limb.joint_angles() == blue_bin_pose):
                gripper.open())
            object_color = "NULL"


if __name__ == 'main':
    main()
