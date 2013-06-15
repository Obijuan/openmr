#!/usr/bin/env python2.7
from openravepy import *
import time as t

RaveInitialize()
RaveLoadPlugin('qtcreator-build/openMR')
try:
    env=Environment()
    env.SetViewer( 'qtcoin')
    env.Load('../../openmr/examples/models/Minicube-II.env.xml')

    with env:
    	snake = env.GetRobots()[0]
    t.sleep(3)

    print "[test4]Creating the SinosController"
    with env:
    	controller = RaveCreateController( env, 'sinoscontroller')
    	snake.SetController( controller)

        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    #-- Set all amplitudes to 45
    A = [45 for j in snake.GetJoints()]
    snake.GetController().SendCommand('setamplitude '+' '.join(str(f) for f in A))

    #-- Set the initial phase to 0, 120, 240....
    phase0 = [i*120 for i in range(snake.GetDOF())]
    snake.GetController().SendCommand('setinitialphase '+' '.join(str(f) for f in phase0))

    #-- Set the period
    snake.GetController().SendCommand('setperiod 1.5');

    snake.GetController().SendCommand('oscillation on');
    
    while True:
	t.sleep(1)
finally:
    RaveDestroy()
