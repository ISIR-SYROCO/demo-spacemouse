import loader

import deploy
deploy.loadTypekitsAndPlugins()
import deploy.deployer as ddeployer

import time

import math
import numpy as np
import lgsm

# to create custom rtt tasks in python and instantiate them

# a special version of IPython
import dsimi.interactive
shell = dsimi.interactive.shell()

# this modules provides the dynamic calculator (gives inertia matrix, jacobians, non linear terms, gravity terms and so on)
import physicshelper

# to create basic rtt tasks
import rtt_interface as rtt
import dsimi.rtt

import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

TIME_STEP = .01

import rtt_interface_corba
rtt_interface_corba.Init(sys.argv)

print "CREATE SCENE..."
import scene1
mechaName = "kuka2"

world = scene1.buildKuka(mechaName, "Yellow")
scene1.addContactLaws(world)
scene1.addGround(world)
scene1.addCollisionPairs(world, "ground", mechaName)

clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
clock.s.setPeriod(TIME_STEP)

print "CREATE GRAPHIC..."
import graphic

graph = graphic.createTask()
graphic.init()
graphic.deserializeWorld(world)

print "CREATE PHYSIC..."
import physic

phy = physic.createTask()
physic.init(TIME_STEP)
physic.deserializeWorld(world, mechaName)

print "CREATE PORTS..."
phy.addCreateInputPort("clock_trigger", "double")
icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("clock_trigger")

graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))
graph.getPort("contacts").connectTo(phy.getPort("contacts"))

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))

print "CREATE SIMPLE CONTROLLER..."
print "Type controller.target_pos to change target position."
import controlQ

controllerq = controlQ.createControllerQ("kukacontrollerq", TIME_STEP, physic.dynmodel)

phy.getPort(mechaName+"_q").connectTo(controllerq.getPort("q"))
phy.getPort(mechaName+"_qdot").connectTo(controllerq.getPort("qdot"))
phy.getPort(mechaName+"_Troot").connectTo(controllerq.getPort("t"))
phy.getPort(mechaName+"_Hroot").connectTo(controllerq.getPort("d"))
controllerq.getPort("tau").connectTo(phy.getPort(mechaName+"_tau"))

#import dummyGetQ

#dumm = dummyGetQ.createDummyGetQ("dummyget")

#dumm.s.setPeriod(TIME_STEP)

taskQRead = dsimi.rtt.Task(rtt_interface_corba.GetProxy("kukaqreader"))
controllerq.getPort("qdes").connectTo( taskQRead.getPort("q_out") )
#dumm.getPort("q").connectTo( taskQRead.getPort("q_out") )

#Enable contacts
physic.robot.enableContactWithBody("ground", True)

controllerq.s.start()
phy.s.setPeriod(TIME_STEP)
graph.s.start()
phy.s.start()
clock.s.start()

shell()
