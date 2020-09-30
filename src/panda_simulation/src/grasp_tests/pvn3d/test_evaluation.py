import numpy as np

all_tests = np.empty((1,4))
RPs = np.empty((1,2))
for i in range(1,10):

	test = np.load('shaft/test'+str(i)+'.npy', allow_pickle=True)

	all_tests = np.concatenate((all_tests, test), axis =0)
	
	

all_tests= all_tests[1:]
#RPs= RPs[1:]
#print all_tests
	
grasp_test_passed=0
place_test_passed=0
placement_error= np.zeros((1,3))


for test in all_tests:
	errors = np.abs(test[3])
	#print(errors[3:5])
	RPs = np.concatenate((RPs, np.array(errors[3:5]).reshape(1,2)), axis=0)
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


print('Total generated grasps '+str(all_tests.shape[0]))
print('Grasp test passed: ' + str(grasp_test_passed) + ': ' + str(float(grasp_test_passed)*100/float(all_tests.shape[0])) +'%')
print('Place test passed: '+ str(place_test_passed) + ': ' + str(float(place_test_passed)*100/float(all_tests.shape[0])) +'%')
#print(placement_error.shape)

print('Average placement error [x y yaw] ' +str(np.nanmean(placement_error[1:],axis=0)) )
print('Placement- error deviation [x y yaw] ' +str(np.std(placement_error[1:],axis=0)) )
print('Average pitch and roll errors '+ str(np.nanmean(RPs, axis=0) ) )

