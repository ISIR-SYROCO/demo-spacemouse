#!/usr/bin/env python

####################################
#                                  #
# Import all modules: configure... #
#                                  #
####################################
import xde_world_manager as xwm

import dsimi.interactive
shell = dsimi.interactive.shell()

TIME_STEP = .01

import xde_robot_loader as xrl
clock, phy, graph = xwm.createAllAgents(TIME_STEP)

import xde_resources as xr

import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

import rtt_interface_corba
rtt_interface_corba.Init(sys.argv)

#Creating Scene

mecha_name = "kuka2"

#Damping 100 ?
kukaWorld = xrl.createWorldFromUrdfFile(xr.kuka, mecha_name, [0.5,0.2,0.4, 1, 0, 0, 0], True, 100, 0.005) #, "material.concrete")
xrl.addContactLaws(kukaWorld)

groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0.0, 1, 0, 0, 0], True, 0.1, 0.05) #, "material.concrete")

xwm.addWorld(groundWorld)
xwm.addWorld(kukaWorld)

#Joint position controller
import controlQ

controllerq = controlQ.createControllerQ("kukacontrollerq", TIME_STEP)
controllerq.connectToRobot(phy, kukaWorld, mecha_name)

#import dummyGetQ
#dumm = dummyGetQ.createDummyGetQ("dummyget", TIME_STEP)

#Get desired position from corba
taskQRead = dsimi.rtt.Task(rtt_interface_corba.GetProxy("kukaqreader"))

taskQRead.getPort("q_out").connectTo( controllerq.getPort("qdes") )

#dumm.getPort("q").connectTo( taskQRead.getPort("q_out") )

#dumm.s.start()
controllerq.s.start()

phy.s.startSimulation()

shell()
