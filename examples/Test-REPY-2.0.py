#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

__author__ = 'Juan Gonzalez, David Estevez'
__copyright__ = 'Copyright (C) 2013 Juan Gonzalez (juan@iearobotics.com), David Estevez'
__license__ = 'GPLv3 license'

#-- This is an example for testing the REPY-2.0 module and its snake configurations
#-- provided with openmr


from openravepy import *
from numpy import *
import time
from optparse import OptionParser


def trans(T,x,y,z):
    T[0,3]=x; T[1,3]=y; T[2,3]=z;
    return T

def run():

    #-- Read the name of the xml fiel passed as an argument
    #-- or use the default name

    parser = OptionParser( "Test-REPY-2.0.py -e <env xml file>")
    parser.add_option('-e', '--environment', dest='file_env', type='string', 
					  help='xml file containing the enviroment to simulate', metavar='FILE', 
					  default='../models/REPY-2.0/Kusanagi-8.env.xml')

    options, notUnderstood = parser.parse_args()
	
    if ( len(notUnderstood) != 0):
    	raise Exception('Command line input not understood: ' + str(notUnderstood) )

    file_env = options.file_env

	#-- Environment startup
    env = Environment()
    env.Load(file_env)
    env.SetViewer('qtcoin')

    #-- Configure the camera view
    #-- Rotation
    T = matrixFromQuat(array((0.505073, 0.268078, 0.395983, 0.718493)))

    #-- Translation
    T=trans(T,0.412915, 0.156822, 0.285362)

    #env.GetViewer().SetCamera(T)

    with env:
        robot = env.GetRobots()[0]
        robot.SetController(RaveCreateController(env, 'sinoscontroller'))
        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    #-- Set all amplitudes to 45
    A = [45 for j in robot.GetJoints()]
    robot.GetController().SendCommand('setamplitude '+' '.join(str(f) for f in A))

    #-- Set the initial phase to 0, 120, 240....
    phase0 = [i*120 for i in range(robot.GetDOF())]
    robot.GetController().SendCommand('setinitialphase '+' '.join(str(f) for f in phase0))

    #-- Set the period
    robot.GetController().SendCommand('setperiod 1.5')

    #-- Set the oscillation
    robot.GetController().SendCommand('oscillation on')

    while True:
		time.sleep(1.0)


if __name__=='__main__':
    run()
