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


    (plan, fraction) = arm.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    arm.execute(plan, wait=True)
    arm.stop()
    while not is_static():
        continue
    rospy.sleep(5)
    states = rospy.wait_for_message('/joint_states', JointState)

    if states.position[0] > 0.005 and states.position[1] > 0.005:     #This checks if the object fell out of the gripper or not
        print('Grasp Stability check: Passed')
        return True
    print('Grasp Stability check: failed')
    return False

