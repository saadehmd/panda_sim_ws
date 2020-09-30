import rospy
from gazebo_msgs.msg import LinkState as LinkStateGZ
from gazebo_msgs.srv import GetModelState as getStateGZ
from geometry_msgs.msg import Pose
import ros_numpy
import numpy as np

poses_in_gazebo = {}

client = rospy.ServiceProxy('gazebo/get_model_state', getStateGZ)

poses_in_gazebo[1] = ros_numpy.numpify(client('piston_and_conrod', 'world').pose)
poses_in_gazebo[2] = ros_numpy.numpify(client('cran_field_round_peg', 'world').pose)
poses_in_gazebo[3] = ros_numpy.numpify(client('cran_field_square_peg', 'world').pose)
poses_in_gazebo[4] = ros_numpy.numpify(client('cran_field_pendulum', 'world').pose)
poses_in_gazebo[5] = ros_numpy.numpify(client('cran_field_pendulum_head', 'world').pose)
poses_in_gazebo[6] = ros_numpy.numpify(client('cran_field_separator', 'world').pose)
poses_in_gazebo[7] = ros_numpy.numpify(client('cran_field_shaft', 'world').pose)
poses_in_gazebo[8] = ros_numpy.numpify(client('cran_field_front_face_plate', 'world').pose)
poses_in_gazebo[9] = ros_numpy.numpify(client('Valve_Tappet', 'world').pose)
poses_in_gazebo[10] = ros_numpy.numpify(client('M11_50mm_Shoulder_Bolt', 'world').pose)

print('writing all values...')
np.save('objs_in_gazebo.npy', poses_in_gazebo )



