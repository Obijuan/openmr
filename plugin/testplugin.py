#!/usr/bin/env python2.7
from openravepy import *
RaveInitialize()
RaveLoadPlugin('qtcreator-build/openMR')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')

    print "Testing the HelloWorld module"
    openMR = RaveCreateModule(env,'openMR')
    print openMR.SendCommand('HelloWorld')
    
    print "Testing the ServoController"
    contrl = RaveCreateController( env, 'servocontroller')
    print contrl.SendCommand('Test')

finally:
    RaveDestroy()
