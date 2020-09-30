import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
import open3d as o3d
import tf
from gazebo_msgs.srv import GetModelState
import ros_numpy
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from lib.utils.basic_utils import Basic_Utils

cfg = Config(dataset_name='openDR')
bs_utils = Basic_Utils(cfg)

 
listener = tf.TransformListener()
all_link_meshes = {'link0':[], 'link1':[] 'link2':[], 'link3':[], 'link4':[],'link5':[], 'link6':[], 'hand':[]  }
panda_points = np.empty()
intrinsic_mat = np.array(rospy.wait_for_message('', CameraInfo).P)

pub = rospy.Publisher('/panda_points_projected', Image)
## This is for a gazebo-simulated camera
	try:
		rospy.wait_for_service('gazebo/get_model_state')
		client = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
		resp1 = client('kinect_ros', 'world')
		
		break
	except Exception as inst:
        	print('Error in gazebo/get_link_state service request: ' + str(inst) )
		break

cam_pos = np.array([resp1.pose.position.x, resp1.pose.position.y, resp1.pose.position.z]).reshape(3,1)
cam_quat = np.array([resp1.pose.orientation.x, resp1.pose.orientation.y, resp1.pose.orientation.z, resp1.pose.orientation.w])

rotMat = R.from_quat(cam_quat).as_dcm()
cam_tf = np.hstack (( rotMat, cam_pos ))
cam_tf = np.vstack(( cam_tf , [0,0,0,1] ))


for i in all_link_meshes.keys():
	
	(trans, rot) = listener.lookupTransform('/panda_'+i , '/world', rospy.Time(0))
	link_pose = np.array([trans.x, trans.y, trans,z])
	link_quat = np.array([rot.x, rot.y, rot,z, rot.w])
	l_rotMat = R.from_quat(link_quat).as_dcm()
	mesh_pose = np.hstack (( l_rotMat, link_pose )) 
	mesh_pose = np.vstack(( mesh_pose, [0, 0, 0, 1] ))
	## Mesh Transformation of each link to camera coordinates 
	mesh_pose = np.dot(mesh_pose , np.linalg.inv(cam_tf) )
	all_link_meshes(i) = np.matmul( mesh_pose, np.array(o3d.io.read_triangle_mesh('/home/ahmad3/catkin_ws/src/franka_ros/franka_description/meshes/visual/'+ i +'.ply')) )
	panda_points = np.vstack(( panda_points , all_link_meshes(i).copy() )) 

''' Perspective projection of links into Depth Image '''

panda_points = np.hstack(( cloud_temp, np.ones((cloud_temp.shape[0], 1)) ))
cam2optical = R.from_eul(['zyx', 1.57, 0, 1.57]).as_dcm() 
cam2optical = np.hstack(( np.vstack(( optical2cam , [0,0,0] )) , np.array([[0],[0],[0],[1]]) ))
panda_points = np.matmul( cam2optical, panda_points.T )

pub_img = np.zeros((480, 640)).astype(np.uint8)
panda_points_2D = bs_utils.project_p3d(panda_points.T , 1, K=cfg.intrinsic_matrix['openDR']):
panda_points_img = bs_utils.draw_p2ds(pub_img, panda_points_2D, color=(255,255,255), 1)

pub.publish(ros_numpy.msgify(panda_points_img.astype(np.uint8), Image))
'''
panda_points_2D = np.matmul(intrinsic_mat , panda_points ).T 
panda_points_2D = np.hstack(( panda_points_2D[:,0]/panda_points_2D[:,2], panda_points_2D[:,1]/panda_points_2D[:,2] ))''' 
