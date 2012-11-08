import loader
import deploy
deploy.loadTypekitsAndPlugins()
import deploy.deployer as ddeployer
import time
import math
import numpy as np
import lgsm

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

import rtt_interface_corba
rtt_interface_corba.Init(sys.argv)

TIME_STEP = .01
print "TIME_STEP ", TIME_STEP

print "BEGIN OF SCRIPT..."

print "CREATE SCENE..."
import scene1

world = scene1.buildKuka()
scene1.addContactLaws(world)
scene1.addGround(world)
scene1.addCollisionPairs(world)

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
physic.deserializeWorld(world)

print "CREATE PORTS..."
phy.addCreateInputPort("clock_trigger", "double")
icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("clock_trigger")

graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))
graph.getPort("contacts").connectTo(phy.getPort("contacts"))

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))

print "CREATE Q READER..."
import qreader

qreader = qreader.createQReader("kukaqreader", physic.dynmodel)

qreader.s.start()

rtt_interface_corba.SetServer(qreader._obj)

print "CREATE SIMPLE CONTROLLER..."
print "Type controller.target_pos = np.matrix([ [X], [Y], [Z] ]) to change target position."
import control

controller = control.createController("kukacontroller", TIME_STEP, physic.dynmodel)

phy.getPort("kuka1_q").connectTo(controller.getPort("q"))
phy.getPort("kuka1_qdot").connectTo(controller.getPort("qdot"))
phy.getPort("kuka1_Troot").connectTo(controller.getPort("t"))
phy.getPort("kuka1_Hroot").connectTo(controller.getPort("d"))
controller.getPort("tau").connectTo(phy.getPort("kuka1_tau"))

graphic.graph_scn.MarkersInterface.addMarker("target", False)
graphic.graph_scn.MarkersInterface.setMarker6DPosition("target", lgsm.Displacement(controller.target_pos))


phy.s.setPeriod(TIME_STEP)
controller.s.setPeriod(TIME_STEP)
qreader.s.setPeriod(TIME_STEP)
graph.s.start()
phy.s.start()
controller.s.start()
clock.s.start()

shell()
