import pdb
import time
import numpy as np
from sys import exit
from openravepy import *

envPath = 'envs/0.0.xml'

try:
	env = Environment()
	env.SetViewer('qtcoin')
	viewer = env.GetViewer()

	# load and set environment
	env.Load(envPath)
	viewer.SetCamera([[  9.99989147e-01,  -3.08830989e-03,   3.48815543e-03,
	          3.30109894e-02],
	       [ -3.08664267e-03,  -9.99995120e-01,  -4.83250519e-04,
	          5.10825627e-02],
	       [  3.48963084e-03,   4.72478585e-04,  -9.99993800e-01,
	          6.83978987e+00],
	       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
	          1.00000000e+00]])
	viewer.SetSize(700, 700) 
	viewer.Move(740, 0) # move viewer to up right corner, modify based on screen (screen.width - viewer.width) (1380 or 1320 on big monitor)

	# set collision checker to Bullet (default build of ODE might not recognize cylinder collision for Ubuntu)
	#collisionChecker = RaveCreateCollisionChecker(env, 'bullet')
	#env.SetCollisionChecker(collisionChecker)

	# load robot
	robot = env.GetRobots()[0]
	robot.SetActiveDOFs([0,1,2]) # , DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis, [0,0,1]
	env.UpdatePublishedBodies()
	time.sleep(0.1)
	'''
	# obtain boundaries of the environment
	with env:
		envmin = []
		envmax = []

		for b in env.GetBodies():
			ab = b.ComputeAABB()
			envmin.append(ab.pos()-ab.extents())
			envmax.append(ab.pos()+ab.extents())

		abrobot = robot.ComputeAABB()
		envmin = numpy.min(np.array(envmin), 0) + abrobot.extents()
		envmax = numpy.max(np.array(envmax), 0) - abrobot.extents()

	bounds = np.array(((envmin[0],envmin[1], 0),(envmax[0],envmax[1], 0)))
	goal = bounds[0, :] + np.random.rand(3) * (bounds[1, :] - bounds[0, :]) # picks random goal location
	#goal = np.concatenate(([0], goal))
	'''
	start = [2, -2, 0]
	goal = [-2, 2, 0]
	pdb.set_trace()
	robot.SetActiveDOFValues(start)

	# OMPL planner
	planner = RaveCreatePlanner(env, 'OMPL_RRTConnect')
	simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')
	
	# Setup the planning instance.
	params = Planner.PlannerParameters()
	params.SetRobotActiveJoints(robot)
	params.SetGoalConfig(goal)

	
	pdb.set_trace()
	planner.InitPlan(robot, params) # executes here
	pdb.set_trace()

	# Invoke the planner.
	traj = RaveCreateTrajectory(env, '')
	result = planner.PlanPath(traj)
	assert result == PlannerStatus.HasSolution

	# Shortcut the path.
	simplifier.InitPlan(robot, Planner.PlannerParameters())
	result = simplifier.PlanPath(traj)
	assert result == PlannerStatus.HasSolution

	# Time the trajectory.
	result = planningutils.RetimeTrajectory(traj)
	assert result == PlannerStatus.HasSolution

	# Execute the trajectory.
	robot.GetController().SetPath(traj)

	#pdb.set_trace()
	
finally:
	# destory environment
	RaveDestroy() 
	exit(0)