import pdb
import time
import numpy
import os
from sys import *
from openravepy import *
from trainingdata import mNavigationPlanningLabels


numbodies = int(argv[1])
envnum = float(argv[2])

numpy.random.seed(None)
st0 = numpy.random.get_state()
#pdb.set_trace()

try:
	env = Environment()
	collisionChecker = RaveCreateCollisionChecker(env, 'bullet')
	env.SetCollisionChecker(collisionChecker)
	env.SetViewer('qtcoin')
	viewer = env.GetViewer()

	# load and set environment
	env.Load('envs/baseworld.xml')
	viewer.SetCamera([[  9.99989147e-01,  -3.08830989e-03,   3.48815543e-03,
	          3.30109894e-02],
	       [ -3.08664267e-03,  -9.99995120e-01,  -4.83250519e-04,
	          5.10825627e-02],
	       [  3.48963084e-03,   4.72478585e-04,  -9.99993800e-01,
	          6.83978987e+00],
	       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
	          1.00000000e+00]])
	#viewer.Move(1140, 0)

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
	
	# create specified number of random-sized obstacles at random location
	while numbodies > 0:
		loc = bounds[0, :] + numpy.random.rand(3) * (bounds[1, :] - bounds[0, :]) # random location
		size =  numpy.random.rand(2) * [0.75, 0.75] # random size

		body = RaveCreateKinBody(env,'')
		body.SetName('body' + str(numbodies))
		body.InitFromBoxes(numpy.array([[loc[0], loc[1], 0.210, size[0], size[1], 0.205]]), True) 
		
		# random rotation
		degrees = 2 * numpy.pi * numpy.random.rand() # random number between 0 and 2pi
		Tz = matrixFromAxisAngle([0, 0, degrees]) # rotates body along world z-direction by random degrees
		body.SetTransform(numpy.dot(Tz, body.GetTransform()))
		
    	# add body to environment
		env.AddKinBody(body)
		env.UpdatePublishedBodies()
		if env.CheckCollision(body, env.GetKinBody('floorwalls')) or env.CheckCollision(robot, body):
			env.Remove(body)
			continue
		else:
			numbodies -= 1
	'''
    #save environment
	pdb.set_trace()
	env.Save('autoenvs/auto' + str(len(os.listdir('autoenvs'))) + '.xml', options=NoRobots)
	pdb.set_trace()
	'''
	#pdb.set_trace()
	# perform motion planning
	#pdb.set_trace()
	planner = mNavigationPlanningLabels(robot, envnum, bounds, [envmin, envmax])
	planner.performNavigationPlanning()
	#pdb.set_trace()
	

finally:
	# destory environment
	RaveDestroy()
