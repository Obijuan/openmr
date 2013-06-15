from openravepy import *
import numpy as np
import time as t
import sys

def simulationLoop( desiredPos, path):
	RaveInitialize()
	RaveLoadPlugin('qtcreator-build/openMR')
	try:
    		env=Environment()
    		env.SetViewer( 'qtcoin')
    		env.Load('../../openmr/examples/models/Minicube-II.env.xml')

    		snake = env.GetRobots()[0]

    		print "Creating the ServoController"
    		controller = RaveCreateController( env, 'servocontroller')
    		snake.SetController( controller)

    		t.sleep(3)

    		env.StopSimulation()
    		env.StartSimulation(timestep=0.001)

		while True:
			command = 'setpos '
			for i in range( len( desiredPos) ):
				command = command + str( desiredPos[i]) + ' '
	    		with env:
        			snake.GetController().SendCommand( command)
    
			t.sleep(0.01)
	finally:
    		RaveDestroy()


def setValues( array, n):
	answer = ''
	print "[setValues] Thread started! Ready to get input:"
	while True:
		answer = raw_input("[setValues] > ")

		if answer == '!q':
			print "[setValues] Exiting... "
			break

		answer = answer.split(' ')

		print "[setValues] Read: " + str( answer)

		for i in range( n):
			array[i] = int(answer[i])

def main():
	from threading import Thread

	if len(sys.argv) > 1:
		path = sys.argv[1]
	else: 
		path = None

	if len(sys.argv) > 2:
		dimensions = int( sys.argv[2])
	else:
		dimensions = 3
	
	desiredPos = np.array([0,]*dimensions);
	
	t1 = Thread( name = 'data', target=setValues, args=(desiredPos, dimensions) )
	t1.start()
	
	t2 = Thread( name = 'simulation', target=simulationLoop, args=(desiredPos, path))
	t2.start()


	
if __name__ == '__main__':
	main()

