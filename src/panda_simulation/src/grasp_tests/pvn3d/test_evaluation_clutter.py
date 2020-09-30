import numpy as np

all_tests = {1:np.zeros((1,4)), 2:np.zeros((1,4)), 3:np.zeros((1,4)), 4:np.zeros((1,4)), 6:np.zeros((1,4)), 7:np.zeros((1,4))}
RPs = np.empty((1,2))



clutter_tests = np.concatenate(( np.load('clutter_tests/clutter1.npy', allow_pickle=True),  
				 np.load('clutter_tests/clutter2.npy', allow_pickle=True), 
				 np.load('clutter_tests/clutter3.npy', allow_pickle=True)), axis=0)

for test in clutter_tests.tolist():

	all_tests[test[0]] = np.concatenate(( all_tests[test[0]], np.array(test[1:]).reshape(1,4)), axis=0)
	
	


#RPs= RPs[1:]
#print all_tests
	



for cls_id in all_tests.keys():
	
	grasp_test_passed=0
	place_test_passed=0
	placement_error= np.zeros((1,3))
	print(all_tests[cls_id].shape)
	for test in all_tests[cls_id]:

		errors = np.abs(test[3])
		#print(errors[3:5])
		if test[0]:
			grasp_test_passed+=1

			#Place pose errors
			x = errors[0]
			y = errors[1]
			yaw = errors[2]
			pitch = errors[3]
			roll = errors[4]
			if roll<8 and pitch <8 and x<0.05 and y<0.05: 		##Object stands upright and is not too far from the place position
				place_test_passed+=1
				placement_error= np.concatenate((placement_error, np.array([x, y, yaw]).reshape(1,3)), axis=0) 
	print('Cls_id: '+str(cls_id))
	print('Total generated grasps '+str(all_tests[cls_id].shape[0]))
	print('Grasp test passed: ' + str(grasp_test_passed) + ': ' + str(float(grasp_test_passed)*100/float(all_tests[cls_id].shape[0])) +'%')
	print('Place test passed: '+ str(place_test_passed) + ': ' + str(float(place_test_passed)*100/float(all_tests[cls_id].shape[0])) +'%')
	#print(placement_error.shape)

	print('Average placement error [x y yaw] ' +str(np.nanmean(placement_error[1:],axis=0)) )
	print('Placement- error deviation [x y yaw] ' +str(np.std(placement_error[1:],axis=0)) )
	#print('Average pitch and roll errors '+ str(np.nanmean(RPs, axis=0) ) )

