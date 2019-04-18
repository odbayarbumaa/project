#object_retreival.py
# Import the necessary Python modules
import rospy # rospy - ROS Python API
import intera_interface # intera_interface - Sawyer Python API
import math # for rounding numbers when checking is gripper is at location (within a specific decimal range)
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
import time
import object_detection # import camera functionality

''' ################################################# '''
'''   Global Initialization of Sawyer Arm Functions   '''
''' ################################################# '''

# initialize our ROS node, registering it with the Master
rospy.init_node('Sawyer_Sort')
# create an instance of intera_interface's Limb class
limb = intera_interface.Limb('right')
# create instance for gripper class, and have open
gripper = intera_interface.Gripper('right_gripper')
gripper.open()

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

limb.move_to_joint_positions(camera_view_pose)
# Default values for object location and color (first two values called from object_detection.py)
# object_color = "NULL"
# object_location = (0.5,0.0)
sawyer_z_hover_plane = -0.05 # default at .15 below table, need to hardcode top of table (measured at 19.5 cm) -0.12 hover, -0.17 grab
sawyer_z_grab_plane = -0.11


''' ########################## '''
'''  Pixel,x,y Transformation  '''
''' ########################## '''
def pix_xy_trans(object_loc):
# pixels per meter calculated by camera pixel length divided by length of space (in meters)
# for x_coord: y_pixel location / (1280 pixels/meter (x pixels) ), then add .3 for the positioning from the arm itself
# for y_coord: x_pixel location - 500 pixels [the middle of map] divided by 1538.4615 for pixels/meter (y pixels)
	x_pixel_size = 960 # frame.shape
	y_pixel_size = 620 # frame.shape
	x_coord_ratio = x_pixel_size/.505 # number is x length in meters of camera space
	y_coord_ratio = y_pixel_size/.375 # number is y length in meters of camera space
	object_y = ((object_loc[0]-500)/x_coord_ratio) - .015
	object_x = ((object_loc[1]/y_coord_ratio) + .335) 

	return (object_x, object_y)


''' #################### '''
'''  Inverse Kinematics  '''
''' #################### '''
def IK(object_loc):
    alignment = 0

    for x in range(0,2):
    	object_x, object_y = pix_xy_trans(object_loc)
    	ik_service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
    	iksvc = rospy.ServiceProxy(ik_service_name, SolvePositionIK)
    	ikreq = SolvePositionIKRequest()
    	hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        if (alignment == 0):
    	# create desired position in x,y,z (in meters)
        	desired_pose = PoseStamped(
        		         header=hdr,
        		         pose=Pose(
        		             position=Point(
        		                 x=object_x, # forward/backward direction (default 0.7)
        		                 y=object_y, # left/right direction
        		                 z=sawyer_z_hover_plane, # up/down direction
        		             ),
        		             orientation=Quaternion(
        		                 x=0.704020578925,
        		                 y=0.710172716916,
        		                 z=0.00244101361829,
        		                 w=0.00194372088834,
        		             )
        		         )
    		     )
        else:
        	# create desired position in x,y,z (in meters)
            time.sleep(1)
            desired_pose = PoseStamped(
        		         header=hdr,
        		         pose=Pose(
        		             position=Point(
        		                 x=object_x, # forward/backward direction (default 0.7)
        		                 y=object_y, # left/right direction
        		                 z=sawyer_z_grab_plane, # up/down direction
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
        alignment+=1

	# include 'from collections import Counter' in header
	# location_for_close = dict(set(limb.joint_angles().items()) - set(target_pose.items()))
	# if (location_for_close[0] <= .1) and (location_for_close[1] <= .1) and (location_for_close[2] <= .1) and (location_for_close[3] <= .1):
	# no need for above, gripper will close after moving to target position
    time.sleep(1)
    gripper.close()

''' ################### '''
'''    Bin Locations    '''
''' ################### '''
def bin_locations():
    # to set positions (hard-code)
    # red_bin_pose = limb.joint_angles()
    # green_bin_pose = limb.joint_angles()
    # blue_bin_pose = limb.joint_angles()

    red_bin_pose = {'right_j6': 2.960640625, 
                    'right_j5': 0.93012109375, 
                    'right_j4': -0.40757421875, 
                    'right_j3': 1.156021484375, 
                    'right_j2': 0.3466845703125, 
                    'right_j1': -0.5025712890625, 
                    'right_j0': -0.84111328125}

    green_bin_pose = {'right_j6': 2.829439453125, 
                    'right_j5': 0.82478125, 
                    'right_j4': -0.3741611328125, 
                    'right_j3': 1.535158203125, 
                    'right_j2': 0.3254619140625, 
                    'right_j1': -0.6862373046875, 
                    'right_j0': -0.9905693359375}


    blue_bin_pose = {'right_j6': 2.465880859375, 
                    'right_j5': 0.57587109375, 
                    'right_j4': -0.1206806640625, 
                    'right_j3': 1.8370546875, 
                    'right_j2': 0.0270302734375, 
                    'right_j1': -0.801359375, 
                    'right_j0': -0.97309375}

    return(red_bin_pose, green_bin_pose, blue_bin_pose)


''' #################### '''
'''         MAIN         '''
''' #################### '''
def main():
    # define bin locations for colors
    limb.move_to_joint_positions(camera_view_pose)
    time.sleep(5)
    red_bin_pose, green_bin_pose, blue_bin_pose = bin_locations()
    blue_object = object_detection.blue_ob
    green_object = object_detection.green_ob
    red_object = object_detection.red_ob

    # initial position to have view out of the way of the camera
    IK(blue_object)
    default_view_pose = limb.joint_angles()
    default_view_pose = {'right_j1': -1.000}
    limb.move_to_joint_positions(default_view_pose)
    limb.move_to_joint_positions(blue_bin_pose)

    # Object color detected and retrieved, move to correct bin and release object        
    gripper.open()
    time.sleep(1)

    # initial position to have view out of the way of the camera
    IK(green_object)
    default_view_pose = limb.joint_angles()
    default_view_pose = {'right_j1': -1.000}
    limb.move_to_joint_positions(default_view_pose)
    limb.move_to_joint_positions(green_bin_pose)

    # Object color detected and retrieved, move to correct bin and release object        
    gripper.open()
    time.sleep(1)

    # initial position to have view out of the way of the camera
    IK(red_object)
    default_view_pose = limb.joint_angles()
    default_view_pose = {'right_j1': -1.000}
    limb.move_to_joint_positions(default_view_pose)
    limb.move_to_joint_positions(red_bin_pose)

    # Object color detected and retrieved, move to correct bin and release object        
    gripper.open()
    time.sleep(1)

    limb.move_to_joint_positions(camera_view_pose)


if __name__ == '__main__':
    main()
