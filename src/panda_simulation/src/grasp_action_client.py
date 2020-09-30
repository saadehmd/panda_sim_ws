#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Twist, PoseArray, PoseStamped, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from panda_simulation.srv import computeGrasps
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatusArray
from gazebo_msgs.msg import LinkState as LinkStateGZ
from gazebo_msgs.srv import SetLinkState as SetLinkStateGZ
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetWorldProperties
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import numpy as np
#sys.path.append('~/catkin_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts')
#from move_group_python_tutorial import MoveGroupPythonIntefaceTutorial as moveitpy

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

def moveL_endEffector_coord(group,X,Y,Z):

    '''
    This function allows cartesian motion planning in End-Effector coordinates.
    We calculate the R matrix from end-effector orientation. The three columns
    [Rx Ry Rz] give three unit vectors in three perpendicular directions centered
    at end-effector frame. We mutiply the required linear displacement in each
    direction with it's respective column in R and add them together for resultant
    linear displacement in end effector coordinates.

    '''

    wpose = group.get_current_pose().pose
    q = wpose.orientation
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

def approach(dist, group):
    # This is just a moveL in end-effector Z-axis
    moveL_endEffector_coord(group,0,0,dist)



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

def deleteGZ(model_name):
    try:
        print('Deleting' + model_name +' from Gazebo...')
        rospy.wait_for_service('gazebo/delete_model')
        client = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        resp = client(model_name)
        print('Model deleted successfully in gazebo!!')
    except Exception as inst:
            print('Error in gazebo/delete_model service request: ' + str(inst) )

def homing(arm, hand):
	
                #home = geometry_msgs.msg.Pose()
		print('Homing the Robot. Please Wait...')

		# Preset Home configurations of joints
                #joint_goal = movegroup.get_current_joint_values()
                arm_joint_goal =[-0.03583304047727598, -0.48172364049446603, 0.038122642984109945, -1.6568494887114191, 0.0727312988443316, 0.48245959568428365, 0.8863241006558038]
                hand_joint_goal = [0.04, 0.04]
                '''
                joint_goal[0] = 0
		joint_goal[1] = -pi/4
		joint_goal[2] = 0
                joint_goal[3] = -pi/4
		joint_goal[4] = 0
		joint_goal[5] = pi/3
		joint_goal[6] = 0		
                #joint_goal[7] = 0
                #joint_goal[8] = 0
                '''
                #home = joint_goal

                move_success = False
		## Keep trying until the homing is successful ##
                #while not move_success:

                try:
                    arm.go(arm_joint_goal, wait=True)
                    hand.go(hand_joint_goal, wait=True)
                    arm.stop()
                    hand.stop()

                    #status = rospy.wait_for_message('/move_group/status', GoalStatusArray).status_list[-1].status
                    #move_success = (status == 3)

                except Exception as inst:
                    print('Homing Failed. Check your planning scene for obstacles and press Enter to try again. Exception: %s', inst)
                    raw_input('')

                print('Homing Successful.')


def main():
	

  try:
		 
	
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                             moveit_msgs.msg.DisplayTrajectory,
		                                             queue_size=20)
                grasp_viz_publisher = rospy.Publisher('/grasp_marker',
                                                             PoseStamped,
		                                             queue_size=20)



		moveit_commander.roscpp_initialize(sys.argv)
                rospy.init_node('grasp_action_client',anonymous=True, disable_signals=True)
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
                group_name = "panda_arm"
                group_name2 = "hand"
		group = moveit_commander.MoveGroupCommander(group_name)
                group2 = moveit_commander.MoveGroupCommander(group_name2)
		planning_frame = group.get_planning_frame()
                #eef_link = group.get_end_effector_link()
                #group.set_end_effector_link('panda_hand')
                goal = geometry_msgs.msg.Pose()
		
		
		while not rospy.is_shutdown():
			
			## NOTE!! Homing should be successful before starting grasp-detection node ##
				
                        homing(group,group2)
                        rospy.sleep(5)

                        ## Reset the Planning scene Octomap ##

                        clear_octomap()


                        ## Replace the box in the Gazebo empty world at the desired pose ##

                        boxPose = Pose(position= Point(x=1, y=0, z=0.196),orientation= Quaternion(x=0, y=0, z=0, w=1))
                        boxTwist = Twist(linear= Vector3(x=0, y=0, z=0) , angular= Vector3(x=0, y=0, z=0))
                        link_state = LinkStateGZ('unit_box::link', boxPose, boxTwist, 'world' )

                        print('Replacing Box in gazebo...')
                        rospy.wait_for_service('gazebo/set_link_state')

                        try:
                            gzclient = rospy.ServiceProxy('gazebo/set_link_state', SetLinkStateGZ)
                            resp = gzclient(link_state)
                            print('Box Replaced successfully in gazebo!!')
                        except Exception as inst:
                                print('Error in gazebo/set_link_state service request: ' + str(inst) )


                        ## Add the box to moveit planning scene ##

                        print('Adding box to the planning scene...')
                        while not 'box' in scene.get_known_object_names():
                            box_pose = geometry_msgs.msg.PoseStamped(header = Header(stamp= rospy.Time.now(), frame_id='world'), pose= boxPose )
                            scene.add_box('box', box_pose, size=(0.726433, 0.741655, 0.392008))         #Box size is a preset already spawned in the empty_world in gazebo

                        ## Spawn the target object in Gazebo ##
                        rot_rand = R.from_rotvec([0, 0, np.random.uniform(-1.57, 1.57)])
                        rot_rand = rot_rand.as_quat()
                        obj_pose = Pose(position= Point(x=0.7, y=0, z=0.391),orientation= Quaternion(x=0, y=0, z=0, w=1))
                        rot_rand = Quaternion(x=rot_rand[0], y=rot_rand[1], z=rot_rand[2], w=rot_rand[3])
                        #obj_pose = Pose(position= Point(x= np.random.uniform(0.7, 0.8), y=np.random.uniform(-0.15, 0.15), z=0.391),orientation = rot_rand)
                        model_name = 'target_object'
                        sdf = open('/home/ahmad3/.gazebo/models/bottle_red_wine/model.sdf')
                        model_xml = sdf.read()
                        spawnGZ(model_name, model_xml, obj_pose, '', 'world' )
                        rospy.sleep(3)
                        ## Update the octomap with initial cloud snapshot ##

                        update_octomap('/panda/camera/depth/points')


			## This node waits here until the grasp-detection node starts sending some data ##
                        '''
			rospy.loginfo('Waiting for ROS messages from grasp-detection node...')
                        grasp_msg = rospy.wait_for_message('/grasp_proposals', PoseArray)			# We subscribe to the grasps published at grasp detection node, arranged in decreasing order of their grasp quality.
                        grasp_array = grasp_msg.poses
                        '''
                        rospy.loginfo('Looking for grasp_detection server...')
                        rospy.wait_for_service('compute_grasps')
                        rospy.loginfo('Successfully connected to grasp_detection server...')
                        usrIn = ''
                        while not usrIn=='Y' and not usrIn=='y':

                            usrIn = raw_input('Enter Y or y to start grasp detection...')

                        try:
                            rospy.loginfo('Request sent to compute grasps.Waiting for response...')
                            compute_grasp_client = rospy.ServiceProxy('compute_grasps', computeGrasps)
                            resp = compute_grasp_client(True)
                            #print(resp)
                            grasp_array = resp.graspProposals.poses
                            print('Successfully recieved '+ str(len(grasp_array))+' grasp poses in response.')
                        except Exception as inst:
                                print('Error in grasp_detection service request: ' + str(inst) )


                        it = 0
               		#marker_array.append(marker)
                        while it < len(grasp_array):


                                ## Publish Grasp as a Marker for RViz Display in world frame

                                goal = grasp_array[it]

                                grasp_viz = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id='world'),pose= goal)

                                print('Publishing goal pose visualizartion...')

                                grasp_viz_publisher.publish(grasp_viz)


				## Homing is done before trying every grasp as it is the most viable position to plan all grasps from ##

                                homing(group, group2)
                                rospy.sleep(5)          #Waiting for the robot to come to a complete stop

                                ## Respawn before every grasp - This will only spawn if the object was previously deleted from the world ##

                                client = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
                                resp = client()

                                if not model_name in resp.model_names:
                                    spawnGZ(model_name, model_xml, obj_pose, '', 'world' )


                                ## Update the octomap before every new grasp ##

                                update_octomap('/panda/camera/depth/points')

                                print ('Planning grasp ', str(it+1),' in the List')
				 
				group.set_pose_target(goal)

				try:
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

                                if len(plan.joint_trajectory.joint_names)!=0:
		
                                        rospy.loginfo('Planning Successful: Goal : X: %.2f, Y: %.2f, Z: %.2f, Quaternion(x, y, z, w): %.2f, %.2f, %.2f, %.2f', goal.position.x, goal.position.y, goal.position.z, goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w)
			

					print('Moving to the grasp pose...')

                                    ## Trajectory Execution
                                        try:
                                                group.execute(plan, wait=True)
                                                group.stop()
                                                #group.clear_pose_targets()

                                        except Exception as inst:

                                                ## Moving failed...What next? ... Planning scene could have been compromised so we also have the option te re-plan grasps
                                                print('Trajectory Execution failed due to: '+ str(inst))
                                                usrIn= raw_input("Enter Y or y if you want to re-plan grasps or anything else if you want to try the next grasps in the proposed list or retry the same grasp: ")
                                                if usrIn=='y' or usrIn=='Y':
                                                        break
                                                else:

                                                        continue

                                        ## Moved Successfully. What next ?? ##

                                        usrIn =  raw_input("Trajectory execution successful. Enter Y or y to try next grasp, A or a to approach and pick or anything else to exit!! ")

                                        if usrIn=='y' or usrIn=='Y':
                                                it+=1
                                                continue

                                        elif usrIn=='a' or usrIn=='A':
                                            clear_octomap()             #We clear Octomap because target_object is not an obstacle anymore
                                            approach(0.1, group)
                                            rospy.sleep(1)              # Waiting for the robot to come to a complete stop
                                            group2.go([0, 0])           # Fully close the gripper
                                            #approach(-0.2, group)      # Retreat motion - Negation of approach
                                            moveL(group,-0.3,0,0.3)
                                            rospy.sleep(6)
                                            deleteGZ('target_object')

                                            it+=1
                                            continue

                                        else:
                                                #rospy.signal_shutdown('')
                                                #break
                                                rospy.sleep(3)
                                                deleteGZ('target_object')

                                                continue

                                ## Planning failed...We try the next proposed grasp ##
                                else:
                                        rospy.logerr('Planning Failed: Goal : X: %.2f, Y: %.2f, Z: %.2f, Quaternion(x, y, z, w): %.2f, %.2f, %.2f, %.2f', goal.position.x, goal.position.y, goal.position.z, goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w)
                                        it+=1
                                        continue
	
		

  except rospy.ROSInterruptException:
    deleteGZ('target_object')
    return
  except KeyboardInterrupt:
    deleteGZ('target_object')
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/kinetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
