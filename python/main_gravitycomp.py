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

import numpy
numpy.set_printoptions(precision=6, suppress=True)
from numpy import array, zeros, ones, pi

#Utility
def lToS(L):
    return " ".join(str(v) for v in L)

print "BEGIN OF SCRIPT..."

print "CREATE SCENE..."
import scene1
mechaName = "kuka1"

world = scene1.buildKuka(mechaName)
worldKuka1 = scene1.buildKuka(mechaName)
scene1.addContactLaws(world)
scene1.addGround(world)
scene1.addWall(world)
scene1.addCube(world)
scene1.addCollisionPairs(world, "ground", mechaName)
scene1.addCollisionPairs(world, "env1", mechaName)
scene1.addCollisionPairs(world, "cube", "ground")
scene1.addCollisionPairs(world, "cube", mechaName)

clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
clock.s.setPeriod(TIME_STEP)

print "CREATE GRAPHIC..."
import graphic

graph = graphic.createTask()
graphic.init()
graphic.deserializeWorld(world)

#Deserialized physics
print "CREATE PHYSIC..."
import physic

phy = physic.createTask()
physic.init(TIME_STEP)
physic.createDynamicModel(worldKuka1, mechaName)
physic.deserializeWorld(world, mechaName)

phy.s.Connectors.OConnectorBodyStateList("ocb").addBody("cube")

print "CREATE PORTS..."
phy.addCreateInputPort("clock_trigger", "double")
icps = phy.s.Connectors.IConnectorSynchro.new("icps")
icps.addEvent("clock_trigger")

graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))
graph.getPort("contacts").connectTo(phy.getPort("contacts"))

clock.getPort("ticks").connectTo(phy.getPort("clock_trigger"))

#spacemouse
# create a factory to build a space mouse
smf = dsimi.rtt.Task(ddeployer.load("smf", "dio::SpaceMouseFactory", "dio-hw-spacemouse", "dio/factory/"))
smf.s.start()
# search for any spacemouses
d = smf.s.scan()
# instantiate the spacemouse
sm = dsimi.rtt.Task(smf.s.build(d[0], 'smf'))

sm.s.start()
# connectors : create a port to get the camera position
phy.addCreateInputPort("sm_vel", "Twistd", True)
phy.getPort("sm_vel").connectTo(sm.getPort("out_vel"))
graph.s.Connectors.OConnectorObs.new("sm_ocof", "obsFrame", "mainScene")
graph.getPort("obsFrame").connectTo(sm.getPort("obs_frame"))
import sm_interface
si=sm_interface.createSM_interface("smi", sm, phy.s.GVM.RigidBody("cube"), 0.01)
si.s.start()

print "CREATE Q READER..."
import qreader

qreader = qreader.createQReader("kukaqreader", physic.dynmodel)

qreader.s.start()

rtt_interface_corba.SetServer(qreader._obj)


print "CREATE SIMPLE CONTROLLER..."
print "Type controller.target_pos = np.matrix([ [X], [Y], [Z] ]) to change target position."
import control

controller = control.createController("kukacontroller", TIME_STEP, physic.dynmodel)

phy.getPort(mechaName+"_q").connectTo(controller.getPort("q"))
phy.getPort(mechaName+"_qdot").connectTo(controller.getPort("qdot"))
phy.getPort(mechaName+"_Troot").connectTo(controller.getPort("t"))
phy.getPort(mechaName+"_Hroot").connectTo(controller.getPort("d"))
controller.getPort("tau").connectTo(phy.getPort(mechaName+"_tau"))

dm    = physic.dynmodel
robot = physic.robot
robot.enableGravity(True)


#################################################
print "SET INIT STATE..."
robot.setJointPositions(np.array([.4]*7).reshape(7,1))
dm.setJointPositions(np.array([.4]*7).reshape(7,1))
robot.setJointVelocities(np.array([0.0]*7).reshape(7,1))
dm.setJointVelocities(np.array([0.0]*7).reshape(7,1))


#Enable contacts
physic.robot.enableContactWithBody("ground", True)
physic.robot.enableContactWithBody("env1", True)
physic.robot.enableContactWithBody("cube", True)

phy.s.setPeriod(TIME_STEP)
qreader.s.setPeriod(TIME_STEP)
controller.s.start()
graph.s.start()
phy.s.start()
clock.s.start()

shell()
