import numpy as np

all_tests = np.empty((1,2))

for i in range(1,4):

	test = np.load('test'+str(i)+'_cls7.npy', allow_pickle=True)
	print(test.shape)
	all_tests = np.concatenate((all_tests, test), axis =0)
	
	

all_tests= all_tests[1:]

grasp_test_passed=0
oscillation_passed=0


for test in all_tests:

	if test[0]:
		grasp_test_passed+=1
		if test[1]:
			oscillation_passed+=1


print('Total generated grasps '+str(all_tests.shape[0]))
print('Grasp test passed: ' + str(grasp_test_passed) + ': ' + str(float(grasp_test_passed)*100/float(all_tests.shape[0])) +'%')
print('Oscillation test passed: '+ str(oscillation_passed) + ': ' + str(float(oscillation_passed)*100/float(all_tests.shape[0])) +'%')

