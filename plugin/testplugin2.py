#!/usr/bin/env python2.7
from openravepy import *
import time as t

RaveInitialize()
RaveLoadPlugin('qtcreator-build/openMR')
try:
    env=Environment()
    env.SetViewer( 'qtcoin')
    env.Load('../../openmr/examples/models/Minicube-II.env.xml')

    snake = env.GetRobots()[0]
    t.sleep(3)

    print "Creating the ServoController"
    controller = RaveCreateController( env, 'servocontroller')
    snake.SetController( controller)

    env.StopSimulation()
    env.StartSimulation(timestep=0.001)

    with env:
	snake.GetController().SendCommand( 'record_on data-test2')
	print "Getpos output: " + snake.GetController().SendCommand( 'Getpos')
        snake.GetController().SendCommand( 'setpos -30 45 -60')
	t.sleep(3)
	print "Getpos1 1 output: " + snake.GetController().SendCommand( 'Getpos1 1')

	snake.GetController().SendCommand( 'record_off')

    
    while True:
	t.sleep(0.01)
finally:
    RaveDestroy()
