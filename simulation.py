import pdb
import time
from sys import *
from trainingdata import mNavigationPlanningLabels
from openravepy import *
from numpy import *


if len(argv) == 3:
	envPath = argv[1]
	envnum = float(argv[2])
else:
	print '\nincorrect number of arguments passed'
	exit(0)

try:
	env = Environment()
	env.Load(envPath)
	env.SetViewer('qtcoin')
	viewer = env.GetViewer()
	viewer.SetCamera([[  9.99989147e-01,  -3.08830989e-03,   3.48815543e-03,
	          3.30109894e-02],
	       [ -3.08664267e-03,  -9.99995120e-01,  -4.83250519e-04,
	          5.10825627e-02],
	       [  3.48963084e-03,   4.72478585e-04,  -9.99993800e-01,
	          6.83978987e+00],
	       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
	          1.00000000e+00]])
	#viewer.SetSize(300, 300) 
	#viewer.Move(1140, 0) # move viewer to up right corner, modify based on screen (screen.width - viewer.width) (1380 or 1320 on big monitor)
	
	# set collision checker to Bullet (default collision checker might not recognize cylinder collision for Ubuntu) (causes issues when moving joints)
	collisionChecker = RaveCreateCollisionChecker(env, 'bullet')
	env.SetCollisionChecker(collisionChecker)

	# load robots
	robot = env.GetRobots()[0]
	env.UpdatePublishedBodies()
	time.sleep(0.1)

	# obtain boundaries of the environment
	with env:
		envmin = []
		envmax = []

		for b in env.GetBodies():
			ab = b.ComputeAABB()
			envmin.append(ab.pos()-ab.extents())
			envmax.append(ab.pos()+ab.extents())

		abrobot = robot.ComputeAABB()
		envmin = numpy.min(numpy.array(envmin),0)+abrobot.extents()
		envmax = numpy.max(numpy.array(envmax),0)-abrobot.extents()

	bounds = numpy.array(((envmin[0],envmin[1],0),(envmax[0],envmax[1],0)))

	###
	#pdb.set_trace()
	#robot.SetActiveDOFs([robot.GetJoint('hor').GetDOFIndex(),robot.GetJoint('vert').GetDOFIndex(),robot.GetJoint('rot').GetDOFIndex()])
	#basemanip = interfaces.BaseManipulation(robot)
	#pdb.set_trace()
	#basemanip.MoveActiveJoints(goal=[-0.75, 0.75, 0])
	###

	# perform motion planning
	planner = mNavigationPlanningLabels(robot, envnum, bounds, [envmin, envmax])
	planner.performNavigationPlanning()
	
finally:
	# destory environment
	RaveDestroy()

