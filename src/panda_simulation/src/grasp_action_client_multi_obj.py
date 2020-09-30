 #!/usr/bin/env python

## This script acts as an action client for the grasp-server provided by demo_cranfield node
## The actual moveit planning, execution and the assembly task is carried out in this client
## It also spawns & repositons, target objects, pick and place boxes in gazebo when required
## It sets up the planning scene, the movegroup and various preset configurations of robot
## It asks for user-prompts at various points to proceed accordingly, hence not fully automatic

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Transform, Vector3, Quaternion
import tf2_ros, tf
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Twist, PoseArray, PoseStamped, Point, Quaternion, Vector3, WrenchStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from panda_simulation.srv import computeGrasps_multiClass, getPoses_multiClass
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatusArray
from gazebo_msgs.msg import LinkState as LinkStateGZ
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetLinkState as SetLinkStateGZ
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetWorldProperties
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, JointState
import numpy as np
from collections import OrderedDict
import argparse
import ros_numpy
#sys.path.append('~/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts')
#from move_group_python_tutorial import MoveGroupPythonIntefaceTutorial as moveitpy


model_names_gazebo = {1:'piston_and_conrod', 2:'cran_field_round_peg', 3:'cran_field_square_peg', 4:'cran_field_pendulum', 5:'cran_field_pendulum_head', 6:'cran_field_separator', 7:'cran_field_shaft', 8:'cran_field_front_face_plate', 9:'Valve_Tappet', 10:'M11_50mm_Shoulder_Bolt'}
model_poses_gazebo = np.load('objs_in_gazebo.npy', allow_pickle=True).item()

parser = argparse.ArgumentParser(description="Arg parser")

objects_spawned = [1,2,3,4,5,6,7]
parser.add_argument('-objects_spawned', nargs='+', type=int)
args = parser.parse_args()

if args.objects_spawned is not None:
    objects_spawned = args.objects_spawned


rospy.init_node('grasp_action_client_multi_obj')
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)


def transform_poses( poses, transform):

    transform_np = ros_numpy.numpify(transform.transform)

    transformed_poses = [0] * len(poses)
    for i, pose in enumerate(poses):

        pose_np = ros_numpy.numpify(pose)
        tf_pose = np.matmul(transform_np, pose_np)
        transformed_poses[i] = ros_numpy.msgify(Pose, tf_pose)

    return transformed_poses

def moveL(group,X,Y,Z):
    wpose = group.get_current_pose().pose
    wpose.position.x += X
    wpose.position.y += Y
    wpose.position.z += Z
    waypoints = []
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    group.execute(plan, wait=True)
    while not is_static():
        continue

def moveL_endEffector_coord(group,X,Y,Z, eef_pose=None):

    '''
    This function allows cartesian motion planning in End-Effector coordinates.
    We calculate the R matrix from end-effector orientation. The three columns
    [Rx Ry Rz] give three unit vectors in three perpendicular directions centered
    at end-effector frame. We mutiply the required linear displacement in each
    direction with it's respective column in R and add them together for resultant
    linear displacement in end effector coordinates.

    '''

    wpose = group.get_current_pose().pose

    if eef_pose is None:
        q = wpose.orientation
    else:
        q = eef_pose.orientation
    qt = R.from_quat([q.x,q.y,q.z,q.w])
    R_mat = qt.as_dcm()

    trans = np.zeros((3,3))

    # Each column represents a unit vector in x y or z direction - gripper coordinates #
    # Multiplied with its respective displacement magnitude #
    trans[:,0] = X*R_mat[0:3,0]
    trans[:,1] = Y*R_mat[0:3,1]
    trans[:,2] = Z*R_mat[0:3,2]

    # Sum along rows to add the contribution to each direction
    # from three different unit vectors(columns)
    wpose.position.x += np.sum(trans[0,:])
    wpose.position.y += np.sum(trans[1,:])
    wpose.position.z += np.sum(trans[2,:])

    waypoints = []
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    group.execute(plan, wait=True)
    while not is_static():
        continue

def approach(dist, group):
    # This is just a moveL in end-effector Z-axis
    moveL_endEffector_coord(group,0,0,dist)
    while not is_static():
        continue
def twist(arm):

    print('Now twisting')
    pose1= arm.get_current_pose().pose
    pose2 = arm.get_current_pose().pose
    pose1.position.z -= 0.001
    pose2.position.z -= 0.001

    rad = np.pi/36
    rot = pose1.orientation

    while rad < np.pi/3:
        print('twisting')
        waypoints = []
        eul1 = R.from_quat(np.array([rot.x, rot.y, rot.z, rot.w])).as_euler('zyx')
        eul2 = R.from_quat(np.array([rot.x, rot.y, rot.z, rot.w])).as_euler('zyx')

        eul1[0] += rad
        rot1 = R.from_euler('zyx', eul1).as_quat()
        pose1.orientation = Quaternion(rot1[0], rot1[1], rot1[2], rot1[3])
        #waypoints.append(copy.deepcopy(pose1))
        #waypoints.append(copy.deepcopy(pose))

        eul2[0] -= rad
        rot2 = R.from_euler('zyx', eul2).as_quat()
        pose2.orientation = Quaternion(rot2[0], rot2[1], rot2[2], rot2[3])
        #waypoints.append(copy.deepcopy(pose2))
        #waypoints.append(copy.deepcopy(pose))

        '''(plan, fraction) = arm.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)'''


        #arm.execute(plan, wait=True)

        arm.set_pose_target(pose1)
        plan = arm.plan()
        arm.execute(plan, wait=True)
        #arm.stop()

        arm.set_pose_target(pose2)
        plan = arm.plan()
        arm.execute(plan, wait=True)
        #arm.stop()
        rad = 2*rad


def touchDown(arm):
    global zMaxForce, checkLateralForces, checkTransverseForce
    print('Moving for touchdown...')
    checkLateralForces = False
    checkTransverseForce = True
    while True:

        if zMaxForce:
            break
        wpose = arm.get_current_pose().pose
        wpose.position.z -= 0.005
        arm.set_pose_target(wpose)
        plan = arm.plan()
        arm.execute(plan, wait=True)
        arm.stop()
    print('Touched down!!!')
    resetForceLimits(80,80,40)

def spiral(group, eef, init_rad, max_rad, incr):

    global checkLateralForces, xMaxForce, yMaxForce, zMaxForce
    checkLateralForces = True
    print('Now Spiraling..')
    wpose = group.get_current_pose().pose
    spiral_pts =  0
    rad = init_rad
    phi = 0
    insert = False
    x = wpose.position.x
    y =  wpose.position.y
    wpose.position.z+=0.005
    #waypoints = []
    while rad < max_rad and not insert:

        for phi in np.arange(0,2*np.pi, np.pi/10):
            wpose.position.x = x + rad*np.cos(phi)
            wpose.position.y = y + rad*np.sin(phi)
            #waypoints.append(copy.deepcopy(wpose))
            print(phi)
            group.set_pose_target(wpose)
            plan = group.plan()
            group.execute(plan, wait=True)
            group.stop()
            eef.go([0 , 0])
            moveL(group,0,0,-0.01)
            #touchDown(group)
            checkLateralForces = True
            print('Spiral waypoint: '+str(spiral_pts))

            fall = ( wpose.position.z - group.get_current_pose().pose.position.z ) > 0.015
            insert = xMaxForce or yMaxForce or fall

            if insert:
                print('Peg inserted...Exiting Spiral!!')
                break
            rad += incr
            spiral_pts+=1



    resetForceLimits(80,80,40)



def clear_octomap():
    try:
        print('Clearing Octomap!!')
        rospy.wait_for_service('clear_octomap')
        client = rospy.ServiceProxy('clear_octomap', Empty)
        client()
        print('Cleared previous octomap from the planning scene!!')

    except Exception as inst:
            print('Error in clear_octomap service request: ' + str(inst) )

def update_octomap(cloud_topic):

    cloud_snap_publisher = rospy.Publisher('/grasping/cloud_snapshot',PointCloud2)
    cloud_snap = rospy.wait_for_message(cloud_topic, PointCloud2)
    cloud_snap_publisher.publish(cloud_snap)
    print('Octomap updated with new PointCloud!!')


def spawnGZ(model_name, model_xml, pose,model_namespace, reference_frame):

    try:
        print('Spawing' + model_name +' in Gazebo...')
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        client = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        resp = client(model_name, model_xml, model_namespace,pose,reference_frame)
        print('Model Spawned successfully in gazebo!!')
    except Exception as inst:
            print('Error in gazebo/spawn_sdf_model service request: ' + str(inst) )


def replaceGZ(cls_id):

    obj_pose = ros_numpy.msgify(Pose, model_poses_gazebo[cls_id])
    model_name = model_names_gazebo[cls_id]
    print('Replacing ' + model_name +' in Gazebo...')

    model_state = ModelState(model_name, obj_pose, Twist(Vector3(0, 0, 0) , Vector3(0, 0, 0)) , 'world' )
    try:
        rospy.wait_for_service('gazebo/set_model_state')
        client = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        resp = client(model_state)
        print('Model Replaced...')
    except Exception as inst:
            print('Error in gazebo/delete_model service request: ' + str(inst) )

def replaceBoxesGZ(boxPose, boxPose2):
    ''' Box to pick up from'''

    boxTwist = Twist(linear= Vector3(x=0, y=0, z=0) , angular= Vector3(x=0, y=0, z=0))
    link_state = LinkStateGZ('unit_box::link', boxPose, boxTwist, 'world' )

    ''' Box to place on '''

    boxTwist2 = Twist(linear= Vector3(x=0, y=0, z=0) , angular= Vector3(x=0, y=0, z=0))
    link_state2 = LinkStateGZ('unit_box_clone::link', boxPose2, boxTwist2, 'world' )

    print('Replacing pick and place Boxes in gazebo...')
    rospy.wait_for_service('gazebo/set_link_state')

    try:
        gzclient = rospy.ServiceProxy('gazebo/set_link_state', SetLinkStateGZ)
        resp = gzclient(link_state)
        resp2 = gzclient(link_state2)

        print('Boxes Replaced successfully in gazebo!!')
    except Exception as inst:
            print('Error in gazebo/set_link_state service request: ' + str(inst) )

def deleteGZ(cls_id):
    try:
        model_name = model_names_gazebo[cls_id]
        print('Deleting' + model_name +' from Gazebo...')
        rospy.wait_for_service('gazebo/delete_model')
        client = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        resp = client(model_name)
        print('Model deleted successfully in gazebo!!')
    except Exception as inst:
            print('Error in gazebo/delete_model service request: ' + str(inst) )

def ft_callback(ft_data):

    global xMaxForce, yMaxForce, zMaxForce
    global fxLimit, fyLimit, fzLimit
    global checkLateralForces, checkTransverseForce
    forces = ft_data.wrench.force

    if checkLateralForces:
        #print('Ft SensorCB: Checking force limits')

        xMaxForce = abs(forces.x) > fxLimit
        yMaxForce = abs(forces.y) > fyLimit

        if xMaxForce or yMaxForce:
            print('X and Y fLimits reached...')
            print('Force X '+str(forces.x)+' ForceY '+str(forces.y))
            checkLateralForces = False
            print('Ft SensorCB: Done checking force limits!!')

    elif checkTransverseForce:

        zMaxForce = abs(forces.z) > fzLimit
        if zMaxForce:
            print('Z fLimits reached...')
            checkTransverseForce = False


def resetForceLimits(limX, limY, limZ):
    print('Resetting force limit readings!!')
    global xMaxForce, yMaxForce, zMaxForce
    global fxLimit, fyLimit, fzLimit
    global checkLateralForces, checkTransverseForce
    fxLimit = limX
    fyLimit = limY
    fzLimit = limZ
    xMaxForce = False
    yMaxForce = False
    zMaxForce = False
    checkLateralForces = False
    checkTransverseForce = False


def is_static():
    ''' The moveit's way of checking if the robot's trajectory has fully terminated or not is a bit faulty.
    This function bypasses moveit and subscribes directly to ros_controller's Joint states topic and signals
    only when all the joint velocities are below a defined threshold, i.e.; Robot is fully static'''

    states = rospy.wait_for_message('/joint_states', JointState)
    velocities = np.abs(np.array(states.velocity))
    #print(velocities.max())
    return np.all(velocities[2:-1]<0.02)  ## This threshold should be equal to or greater than the stopped_velocity_tolerance param in controller config.yaml .

def homing(arm, hand):
	
                #home = geometry_msgs.msg.Pose()
		print('Homing the Robot. Please Wait...')

		# Preset Home configurations of joints
                arm_joint_goal =[-0.129357756648, -1.18744531174, 0.0853945964882, -2.13600317485, 0.113542634763, 0.41314229023, 0.788530202896]
                hand_joint_goal = [0.04, 0.04]
                arm.go(arm_joint_goal, wait=True)
                hand.go(hand_joint_goal, wait=True)
                arm.stop()
                hand.stop()
                print('Homing Successful.')
                while not is_static():
                    continue



def get_fixtures():
    ''' This function is used to retrieve the cranfield assembly fixtures i.e; target locations to place the objects in.
        It simply makes a /computeGrasp request to demo_cranfield node and acquires only the grasps of the "Faceplate"
        class, which are infact the fixtures on it. All the remaining objects are placed w.r.t these fixtures. '''
    rospy.set_param('pvn3d_cam', '/kinect1')
    rospy.set_param('pvn3d_cam_frame', 'kinect1_optical_link')
    rospy.wait_for_service('/compute_grasps')
    compute_grasp_client = rospy.ServiceProxy('/compute_grasps', computeGrasps_multiClass)
    resp = compute_grasp_client(True)
    idx = np.where(np.array(resp.graspLabels.data)==8)[0]           # Choose only the grasps labeled class_id==8 i.e.; Faceplate
    fixtures = np.array(resp.graspProposals.poses)[idx]
    return fixtures

def get_true_fixtures():
    ''' The same as the get_fixtures function except, we don't use pose estimation this time on the faceplate but ...
        directly query the true faceplate pose from gazebo and return fixtures based on the true pose. This is to
        test manipuability of the robot irrespective of the pose-estimation accuracy and compare the results of both'''
    rospy.wait_for_service('gazebo/get_model_state')
    client = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    faceplate_pose = ros_numpy.numpify(client('cran_field_front_face_plate', 'world').pose)
    fixtures = np.loadtxt('/home/ahmad3/PVN3D/pvn3d/datasets/openDR/dataset_config/preDefined_grasps/8.txt')
    true_fixtures = []
    for j in range(fixtures.shape[0]):
        fixt_rot = R.from_euler('zyx', fixtures[j, 3:6] ).as_dcm()
        fixt_pos = fixtures[j, 0:3].reshape(3,1)
        fixt_in_obj = np.vstack(( np.hstack(( fixt_rot, fixt_pos)), np.array([0,0,0,1]) ))    #Fixtures defined in faceplate coordinates
        fixt_in_world = np.matmul( faceplate_pose, fixt_in_obj )                          #Fixtures in world coordinates
        true_fixtures.append(ros_numpy.msgify(Pose, fixt_in_world))
    return true_fixtures

def get_pose(cls_id):
    ''' This functions sends a /get_pose_pvn3d request to the pvn3d_detection_server
        in order to acquire pose of the object after it is grasped and before placing it '''
    pose_client = rospy.ServiceProxy('get_poses_pvn3d', getPoses_multiClass)
    rospy.set_param('pvn3d_cam', '/kinect1')
    rospy.set_param('pvn3d_cam_frame', 'kinect1_optical_link')
    rospy.set_param('remove_panda', True)
    resp = pose_client(True)
    rospy.set_param('remove_panda', False)
    idx = np.where(np.array(resp.objectLabels.data)==cls_id)[0]
    pose = np.array(resp.poses.poses)[idx]
    print('Pose shape '+str(pose.shape))
    return pose

def place(arm, hand, cls_id, upright=True):

    ''' This function moves the robot in pre-place position, re-adjusts the robot's goal pose w.r.t the pose
        of the object in hand and then proceeds to place it in it's respective fixture on the faceplate'''


    pub_pose = rospy.Publisher('/pre_place_pose', PoseStamped, queue_size=10)


    print('Moving to the pre-place position. Please Wait...')

    # Preset configurations of joints for place-position
    arm_joint_goal =[2.8970090067, -0.14661533001, -1.37000730752, -1.58669425797, -0.240870798379, 1.4372895365, 1.49148429199]
    #hand_joint_goal = [0.04, 0.04]

    ## Move the arm to a pre-place position
    arm.go(arm_joint_goal, wait=True)
    #hand.go(hand_joint_goal, wait=True)
    arm.stop()
    #hand.stop()
    while not is_static():
        continue
    #rospy.sleep(5)
    #graspStabilityCheck(arm, hand)
    ## Pre-defined fixtures for each class.
    #NOTE: Some classes have multiple fixtures because the cranfield assembly has multiple instances of a few object classes.
    fix_dict = {2:[3], 3:[0,1], 7:[4]}
    goal_idx = fix_dict[cls_id].pop(0)
    #goal_pose =  arm.get_current_pose().pose
    fixt_pose = fixtures[goal_idx]
    fixt_pose.position.z += 0.11            # We stop a few cm above the fixture check the object pose to re-adjust the arm
    if upright:
        print('Placing upright...')
        goal_pose = fixt_pose     # Set the goal position based on the current object-class in robot hand
    else:
        print('Tilting to place upright...')
        quat = np.array([fixt_pose.orientation.x, fixt_pose.orientation.y, fixt_pose.orientation.z, fixt_pose.orientation.w])
        eul = R.from_quat(quat).as_euler('zyx')
        eul[2]= -np.pi/2
        eul[0] = np.pi
        quat_rotated= R.from_euler('zyx', eul).as_quat()
        fixt_pose.orientation = Quaternion(quat_rotated[0], quat_rotated[1], quat_rotated[2], quat_rotated[3])
        goal_pose = fixt_pose
    pub_pose.publish(PoseStamped(Header(stamp=rospy.Time.now(), frame_id='world'), goal_pose))
    update_octomap('/panda/camera/depth/points')

    print('Moving above fixture '+str(goal_idx)+' for class_id '+str(cls_id) )
    arm.set_pose_target(goal_pose)
    plan = arm.plan()
    arm.execute(plan, wait=True)
    arm.stop()
    #rospy.sleep(5)
    while not is_static():
        continue
    '''
    print('Computing final pre-place pose of the object...')
    while True:
        pose = get_pose(cls_id)[0]
        try:
            tf_stamped = tf_buffer.lookup_transform('world', 'kinect1_optical_link' , rospy.Time(0))

        except Exception as inst:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            print('exception: '+str(inst)+' in '+ str(exc_tb.tb_lineno))
        #obj_in_world = do_transform_pose(PoseStamped(Header(stamp=rospy.Time.now(), frame_id='kinect1_optical_link'), pose), cam_transform)
        obj_in_world = transform_poses([pose], tf_stamped )
        # A difference of translation between obj_pose and goal_pose gives a rough estimate of displacement that the arm needs
        # before approaching for final placement
        transX = goal_pose.position.x - obj_in_world[0].position.x
        transY = goal_pose.position.y - obj_in_world[0].position.y
        #transZ = obj_in_world.position.z - goal_pose.position.z
        #moveL(arm,transX, transY,0)    #Re-adjustment before approach

        This step comes with the assumption that object might be slightly tilted after grasping.
            Moving against the tilt provides a hacky way of pushing the object back into it's
            desired fitting position(position-compliance), provided that the tilt is not too
            high because then the object might miss the fitting point entirely

        # We invert obj_rotation to move against the tilt of object-axis
        # This would make the object collide with the clearance of the
        # fixture and hence re-orient itself to fit the final clearance


        obj_pose_np = ros_numpy.numpify(obj_in_world[0])
        rob_pose_np = ros_numpy.numpify(goal_pose)

        #obj_pose_np = np.matmul(R.from_euler('zyx',[0,0,np.pi]).as_dcm(), obj_pose_np)

        #obj_pose_np = np.matmul( np.linalg.inv(rob_pose_np) , obj_pose_np)
        #goal_adjusted = np.matmul(rob_pose_np, obj_pose_np )
        invert_mat = np.vstack(( np.hstack(( R.from_euler('zyx', [0,0,np.pi]).as_dcm(), np.zeros((3,1)) )), np.array([0,0,0,1]) ))
        goal_adjusted = np.matmul( obj_pose_np,invert_mat)
        #goal_adjusted = np.matmul( invert_mat, goal_adjusted)

        obj_rpy = R.from_dcm(obj_pose_np[:3,:3]).as_euler('zyx')
        goal_rpy = R.from_dcm(rob_pose_np[:3,:3]).as_euler('zyx')

        roll_diff = goal_rpy[2] - obj_rpy[2]
        goal_rpy[2] += roll_diff
        rob_pose_np[:3, :3] = R.from_euler('zyx', goal_rpy).as_dcm()
        #rob_pose_np[:3, 3] += np.array([transX, transY, 0]).T
        goal_adjusted = rob_pose_np
        goal_adjusted = ros_numpy.msgify(Pose, goal_adjusted)
        goal_adjusted.position.z = goal_pose.position.z
        pub_pose.publish(PoseStamped(Header(stamp=rospy.Time.now(), frame_id='world'),goal_adjusted))
        arm.set_pose_target(goal_adjusted)
        plan = arm.plan()
        arm.execute(plan, wait=True)
        arm.stop()
        #rospy.sleep(5)
        while not is_static():
            continue'''


    while True:

        zForce = rospy.wait_for_message('/ft_sensor_topic', WrenchStamped).wrench.force.z
        if abs(zForce) > 40:
            break
        wpose = arm.get_current_pose().pose
        wpose.position.z -= 0.005
        arm.set_pose_target(wpose)
        plan = arm.plan()
        arm.execute(plan, wait=True)
        arm.stop()
    print('Touched down!!!')
    while not is_static():
        continue
    #hand.go([0.04, 0.04])
    #rospy.sleep(2)
    #hand.go([0, 0])
    spiral(arm, hand,  0.0001, 0.005, 0.0001)
    #twist(arm)
    hand.go([0.04, 0.04])

    '''# We moveLinear in obj_coord i.e; The object pose is faked as end-effector pose because we want to move parallel to objAxis(z-direction)
    usrIn = ''
    #while not usrIn=='Y' and not usrIn=='y':

    usrIn = raw_input('Enter Y or y to continue to final placement , N or n to recalculate pose...')


    #rospy.sleep(2)
    if usrIn=='Y' or usrIn=='y':
        moveL_endEffector_coord(arm,0,0,0.02, eef_pose=goal_adjusted  )     #Fake the adjusted goal pose as an end-effector so the arm moves in goal coordinates
        hand.go([0.04, 0.04])       #Fully open the gripper and let the gravity do the rest ;)
        break

    else:
        continue'''


def graspStabilityCheck(arm,hand):

    pose = arm.get_current_pose().pose
    pose1= arm.get_current_pose().pose
    pose2 = arm.get_current_pose().pose
    pose3 = arm.get_current_pose().pose
    pose4 = arm.get_current_pose().pose
    #pose_np = ros_numpy.numpify(pose)

    waypoints = []

    rot = pose.orientation
    eul1 = R.from_quat(np.array([rot.x, rot.y, rot.z, rot.w])).as_euler('zyx')
    eul2 = R.from_quat(np.array([rot.x, rot.y, rot.z, rot.w])).as_euler('zyx')
    eul3 = R.from_quat(np.array([rot.x, rot.y, rot.z, rot.w])).as_euler('zyx')
    eul4 = R.from_quat(np.array([rot.x, rot.y, rot.z, rot.w])).as_euler('zyx')

    eul1[0] += np.pi/4
    rot1 = R.from_euler('zyx', eul1).as_quat()
    pose1.orientation = Quaternion(rot1[0], rot1[1], rot1[2], rot1[3])
    waypoints.append(copy.deepcopy(pose1))
    #waypoints.append(copy.deepcopy(pose))

    eul2[0] -= np.pi/4
    rot2 = R.from_euler('zyx', eul2).as_quat()
    pose2.orientation = Quaternion(rot2[0], rot2[1], rot2[2], rot2[3])
    waypoints.append(copy.deepcopy(pose2))
    #waypoints.append(copy.deepcopy(pose))

    eul3[2] += np.pi/6
    rot3 = R.from_euler('zyx', eul3).as_quat()
    pose3.orientation = Quaternion(rot3[0], rot3[1], rot3[2], rot3[3])
    waypoints.append(copy.deepcopy(pose3))
    #waypoints.append(copy.deepcopy(pose))

    eul4[2] -= np.pi/6
    rot4 = R.from_euler('zyx', eul4).as_quat()
    pose4.orientation = Quaternion(rot4[0], rot4[1], rot4[2], rot4[3])
    waypoints.append(copy.deepcopy(pose4))
    waypoints.append(copy.deepcopy(pose))

    #for loop in range(0,2):
    print('Now starting oscillation...')
    '''
    arm.set_pose_target(pose1)
    plan = arm.plan()
    arm.execute(plan, wait=True)
    #arm.stop()

    arm.set_pose_target(pose2)
    plan = arm.plan()
    arm.execute(plan, wait=True)
    #arm.stop()

    arm.set_pose_target(pose)
    plan = arm.plan()
    arm.execute(plan, wait=True)

    arm.set_pose_target(pose3)
    plan = arm.plan()
    arm.execute(plan, wait=True)
    #arm.stop()

    arm.set_pose_target(pose4)
    plan = arm.plan()
    arm.execute(plan, wait=True)

    arm.set_pose_target(pose)
    plan = arm.plan()
    arm.execute(plan, wait=True)'''

    (plan, fraction) = arm.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    arm.execute(plan, wait=True)

    while not is_static():
        continue

    states = rospy.wait_for_message('/joint_states', JointState)

    if states.position[0] > 0.01 and states.position[1] > 0.01:     #This checks if the object fell out of the gripper or not
        print('Grasp Stability check: Passed')
        return True
    print('Grasp Stability check: failed')
    return False
## Retrieve the faceplate fixtures
fixtures = get_true_fixtures()

#Force-sensor initializations
checkLateralForces = False
checkTransverseForce = False
fxLimit = 80
fyLimit = 80
fzLimit = 40
xMaxForce = False
yMaxForce = False
zMaxForce = False

ft_sub = rospy.Subscriber('/ft_sensor_topic', WrenchStamped, ft_callback)
def main():
	

  try:
		 
	
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                             moveit_msgs.msg.DisplayTrajectory,
		                                             queue_size=20)
                grasp_viz_publisher = rospy.Publisher('/grasp_marker',
                                                             PoseStamped,
		                                             queue_size=20)


                ## Moveit commander initialization
                moveit_commander.roscpp_initialize(sys.argv)

                robot = moveit_commander.RobotCommander()
                scene = moveit_commander.PlanningSceneInterface()
                group_name = "panda_arm"
                group_name2 = "hand"
                group = moveit_commander.MoveGroupCommander(group_name)
                group2 = moveit_commander.MoveGroupCommander(group_name2)
                planning_frame = group.get_planning_frame()
                #eef_link = group.get_end_effector_link()
                #group.set_end_effector_link('panda_hand')
                goal = Pose()

                while not rospy.is_shutdown():

                    ## NOTE!! Homing should be successful before starting grasp-detection node ##

                    homing(group,group2)
                    #rospy.sleep(5)

                    ## Reset the Planning scene Octomap ##

                    clear_octomap()

                    ## Replace the boxes in the Gazebo grasp world at the desired pose ##
                    # NOTE: This is for a gazebo World that already has a box in spawned in it
                    boxPose = Pose(position= Point(x=0.556488, y=-0.041543, z=0.173316),orientation= Quaternion(x=0, y=0, z=0, w=1))
                    boxPose2 = Pose(position= Point(x=-0.024996, y=0.533282, z=0.102870),orientation= Quaternion(x=0, y=0, z=0.707, w=0.707))
                    replaceBoxesGZ(boxPose, boxPose2)
                    #boxPose = Pose(position= Point(x=0.561442, y=0, z=0.173316),orientation= Quaternion(x=0, y=0, z=0, w=1))
                    #boxPose2 = Pose(position= Point(x=0, y=0.48, z=0.102870),orientation= Quaternion(x=0, y=0, z=0.707, w=0.707))

                    ## Spawn the target object in Gazebo ##
                    rot_rand = R.from_rotvec([0, 0, np.random.uniform(-1.57, 1.57)])
                    rot_rand = rot_rand.as_quat()
                    obj_pose = Pose(position= Point(x=1, y=1, z=0.2),orientation= Quaternion(x=0, y=0, z=0, w=1))
                    rot_rand = Quaternion(x=rot_rand[0], y=rot_rand[1], z=rot_rand[2], w=rot_rand[3])
                    #obj_pose = Pose(position= Point(x= np.random.uniform(0.7, 0.8), y=np.random.uniform(-0.15, 0.15), z=0.391),orientation = rot_rand)
                    for obj in objects_spawned:

                        sdf = open('/home/ahmad3/.gazebo/models/'+str(model_names_gazebo[obj])+'/model.sdf')
                        model_xml = sdf.read()
                        spawnGZ(model_names_gazebo[obj], model_xml, obj_pose, '', 'world' )
                        #rospy.sleep(1)

                    ##Replace target objects
                    for obj in objects_spawned:
                        replaceGZ(obj)
                    ## Add the boxes to moveit planning scene ##

                    print('Adding boxes to the planning scene...')

                    while not 'pickBox' in scene.get_known_object_names():
                        box_pose = PoseStamped(header = Header(stamp= rospy.Time.now(), frame_id='world'), pose= boxPose )
                        scene.add_box('pickBox', box_pose, size=(0.325959, 0.450021, 0.341363))         #Box size is a preset already spawned in the empty_world in gazebo


                    while not 'placeBox' in scene.get_known_object_names():
                        box_pose = PoseStamped(header = Header(stamp= rospy.Time.now(), frame_id='world'), pose= boxPose2 )
                        scene.add_box('placeBox', box_pose, size=(0.304797, 0.450021, 0.205744))         #Box size is a preset already spawned in the empty_world in gazebo

                    rospy.sleep(2)
                    ## Update the octomap with initial cloud snapshot ##

                    update_octomap('/panda/camera/depth/points')


                    ## This node waits here until the grasp-detection node starts sending some data ##
                    '''
                    rospy.loginfo('Waiting for ROS messages from grasp-detection node...')
                    grasp_msg = rospy.wait_for_message('/grasp_proposals', PoseArray)			# We subscribe to the grasps published at grasp detection node, arranged in decreasing order of their grasp quality.
                    grasp_array = grasp_msg.poses
                    '''
                    rospy.loginfo('Looking for grasp_detection server on proxy: /compute_grasps ...')
                    rospy.wait_for_service('/compute_grasps')
                    rospy.loginfo('Successfully connected to grasp_detection server...')
                    usrIn = ''
                    while not usrIn=='Y' and not usrIn=='y':

                        usrIn = raw_input('Enter Y or y to start grasp detection...')

                    try:
                        rospy.loginfo('Request sent to compute grasps.Waiting for response...')
                        rospy.set_param('pvn3d_cam', '/panda/camera')
                        rospy.set_param('pvn3d_cam_frame', 'panda_camera_optical_frame')
                        rospy.set_param('remove_panda', False)
                        compute_grasp_client = rospy.ServiceProxy('/compute_grasps', computeGrasps_multiClass)
                        gr_resp = compute_grasp_client(True)
                        #print(resp)

                        print('Successfully recieved '+ str(len(gr_resp.graspProposals.poses))+' grasp poses in response.')

                    except Exception as inst:
                        print('Error in grasp_detection service request: ' + str(inst) )



                    grasp_list = gr_resp.graspProposals.poses
                    grasp_labels = gr_resp.graspLabels.data

                    grasp_dict = OrderedDict()

                    # Organize grasps w.r.t. their class labels
                    for lbl in args.objects_spawned:#sgrasp_labels:
                        idx = np.where(np.array(grasp_labels)==lbl)[0]
                        grasp_dict[lbl] = np.array(grasp_list)[idx].tolist()


                    print(grasp_dict.keys())
                    #while it < len(grasp_array):
                    for cls_id in grasp_dict.keys():

                            while len(grasp_dict[cls_id]) > 0:
                                #for grasp in grasp_dict[cls_id]:

                                    ##Replace target objects
                                    for obj in objects_spawned:
                                        replaceGZ(obj)

                                    ##Replace the boxes
                                    replaceBoxesGZ(boxPose, boxPose2)

                                    ## Publish Grasp as a Marker for RViz Display in world frame

                                    goal = grasp_dict[cls_id][0]

                                    grasp_viz = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='world'),pose= goal)

                                    print('Publishing goal pose visualization...')

                                    grasp_viz_publisher.publish(grasp_viz)


                                    ## Homing is done before trying every grasp as it is the most viable position to plan all grasps from ##

                                    homing(group, group2)
                                    #rospy.sleep(5)          #Waiting for the robot to come to a complete stop
                                    '''
                                    ## Respawn before every grasp - This will only spawn if the object was previously deleted from the world ##

                                    client = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
                                    resp = client()

                                    if not model_name in resp.model_names:
                                        spawnGZ(model_name, model_xml, obj_pose, '', 'world' )'''


                                    ## Update the octomap before every new grasp ##

                                    update_octomap('/panda/camera/depth/points')

                                    #print ('Planning grasp ', str(it+1),' in the List')

                                    group.set_pose_target(goal)

                                    try:
                                            print('Planning grasp for '+ str(model_names_gazebo[cls_id]) )
                                            plan = group.plan()
                                            #print(plan)

                                    except Exception as inst:
                                            print('Planning error: ' + str(inst) )
                                            print('Continuing to the next grasp...')
                                            continue


                                    #group.stop()	# Calling `stop()` ensures that there is no residual movement

                                    # It is always good to clear your targets after planning with poses.
                                    # Note: there is no equivalent function for clear_joint_value_targets()
                                    group.clear_pose_targets()

                                    if len(plan.joint_trajectory.joint_names)!=0:           # Check if the plan is empty

                                            rospy.loginfo('Planning Successful: Goal : X: %.2f, Y: %.2f, Z: %.2f, Quaternion(x, y, z, w): %.2f, %.2f, %.2f, %.2f', goal.position.x, goal.position.y, goal.position.z, goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w)


                                            print('Moving to the grasp '+str(cls_id))

                                        ## Trajectory Execution
                                            try:
                                                    group.execute(plan, wait=True)
                                                    group.stop()
                                                    #group.clear_pose_targets()

                                            except Exception as inst:

                                                    ## Moving failed...What next? ... Planning scene could have been compromised so we also have the option te re-plan grasps
                                                    print('Trajectory Execution failed due to: '+ str(inst))
                                                    usrIn= raw_input("Enter Y or y if you want to re-plan the same grasp, N or n if you want to try grasps on another object, or anything else if you want to try a different grasp for the same object. ")
                                                    if usrIn=='y' or usrIn=='Y':
                                                        continue
                                                    elif usrIn=='n' or usrIn=='N':
                                                        grasp_dict.pop(cls_id, None)
                                                        break
                                                    else:
                                                        grasp_dict[cls_id].pop(0)
                                                        continue

                                            ## Moved Successfully. What next ?? ##
                                            clear_octomap()
                                            scene.remove_world_object('pickBox')
                                            scene.remove_world_object('placeBox')

                                            usrIn =  raw_input("Trajectory execution successful. Enter Y or y to try next grasp, A or a to approach and pick or anything else to try grasps on next object!! ")

                                            if usrIn=='y' or usrIn=='Y':
                                                grasp_dict[cls_id].pop(0)
                                                continue

                                            elif usrIn=='a' or usrIn=='A':
                                                             #We clear Octomap because target_object is not an obstacle anymore
                                                approach(0.03, group)
                                                #rospy.sleep(1)              # Waiting for the robot to come to a complete stop
                                                group2.go([0, 0])           # Fully close the gripper
                                                #approach(-0.2, group)       # Retreat motion - Negation of approach
                                                moveL(group, -0.1,0,0.15)


                                                #rospy.sleep(6)
                                                #group2.go([0.04, 0.04])
                                                #deleteGZ(cls_id)
                                                #place(group, group2, cls_id)

                                                #print('Now going to place the object '+str(cls_id))
                                                '''
                                                grasp_quat = np.array([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w])
                                                tilt = abs(np.rad2deg(R.from_quat(grasp_quat).as_euler('zyx')[2]))
                                                if tilt > 90:
                                                    tilt = 180 - tilt

                                                print('Tilt of the grasp is '+str(tilt))
                                                if tilt < 45:
                                                    place(group, group2, cls_id, upright=True)
                                                else:
                                                    place(group, group2, cls_id, upright=False)'''

                                                usrIn= raw_input("Enter Y or y if you want to retry the same grasp, N or n if you want to try grasps on another object, or anything else if you want to try a different grasp for the same object. ")
                                                if usrIn=='y' or usrIn=='Y':
                                                    replaceGZ(cls_id)
                                                    continue
                                                elif usrIn=='n' or usrIn=='N':
                                                    grasp_dict.pop(cls_id, None)
                                                    break
                                                else:
                                                    replaceGZ(cls_id)
                                                    grasp_dict[cls_id].pop(0)
                                                    continue



                                            else:
                                                break

                                            break



                                    ## Planning failed...We try the next proposed grasp ##
                                    else:               # No plan was generated
                                            rospy.logerr('Planning Failed: Goal : X: %.2f, Y: %.2f, Z: %.2f, Quaternion(x, y, z, w): %.2f, %.2f, %.2f, %.2f', goal.position.x, goal.position.y, goal.position.z, goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w)
                                            grasp_dict[cls_id].pop(0)
                                            continue
	
		

  except rospy.ROSInterruptException:
    #deleteGZ('target_object')
    return
  except KeyboardInterrupt:
    #deleteGZ('target_object')
    return

if __name__ == '__main__':
  main()


