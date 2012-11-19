import loader

import deploy
deploy.loadTypekitsAndPlugins()
import deploy.deployer as ddeployer

import time

import math
import numpy as np
import lgsm

import numpy
numpy.set_printoptions(precision=6, suppress=True)
from numpy import array, zeros, ones, pi

#Utility
def lToS(L):
    return " ".join(str(v) for v in L)

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

world = scene1.buildKuka(mechaName, "Red")
scene1.addContactLaws(world)
scene1.addGround(world)
scene1.addWall(world)
scene1.addCollisionPairs(world, "ground", mechaName)
scene1.addCollisionPairs(world, "env1", mechaName)

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



print "CREATE CONTROLLER..."
#####
# add orcisir_ISIRController
oIT = ddeployer.load("oIT", "Orocos_ISIRController", module="orcisir_Orocos_IsirController-gnulinux", prefix="", libdir="/home/shak/src/orcisir_IsirController/_build/src/")
TT = dsimi.rtt.Task(oIT)

phy.getPort(mechaName+"_q").connectTo(TT.getPort("q"))
phy.getPort(mechaName+"_qdot").connectTo(TT.getPort("qDot"))
phy.getPort(mechaName+"_Hroot").connectTo(TT.getPort("Hroot"))
phy.getPort(mechaName+"_Troot").connectTo(TT.getPort("Troot"))
TT.getPort("tau").connectTo(phy.getPort(mechaName+"_tau"))

dm    = physic.dynmodel
robot = physic.robot
robot.enableGravity(True)

TT.s.setDynModelPointerStr(str(dm.this.__long__()), "qld") #"qld" quadprog


#################################################
print "SET TASKS..."
#TT.s.updateTasks("createTask  myTask  0  1  fullState")
#TT.s.updateTasks("updateTask  myTask  -Kp 25  -Kd 10  -qDes 0 0 0 0 0 0 0  -dqDes 0 0 0 0 0 0 0  endUpdate")

Kp, Kd = 25, 10
qDes = 0.*ones(dm.nbDofs())
dqDes = zeros(dm.nbDofs())
pqDes = -0.7*ones(2)
pdqDes = zeros(2)
posDes = lgsm.Displacement(.38, .110, .30)
velDes = lgsm.Twist()
TT.s.updateTasks("createTask  taskFull  4  1  fullState")
TT.s.updateTasks("updateTask  taskFull  -Kp {0}  -Kd {1}  -qDes {2}  -dqDes {3}  endUpdate".format(Kp, Kd, lToS(qDes), lToS(dqDes)))
#TT.s.updateTasks("createTask  taskPart  3  1  partialState 2 0 1")
#TT.s.updateTasks("updateTask  taskPart  -Kp {0}  -Kd {1}  -partqDes {2}  -partdqDes {3}  endUpdate".format(Kp, Kd, lToS(pqDes), lToS(pdqDes)))
#TT.s.updateTasks("createTask  taskFrameOri  2  1  frame 07 {0} R".format(lToS(lgsm.Displacement())))
#TT.s.updateTasks("updateTask  taskFrameOri  -Kp {0}  -Kd {1}  -posDes {2}  -velDes {3}  endUpdate".format(Kp, Kd, lToS(posDes), lToS(velDes)))

#TT.s.updateTasks("createTask  taskFrame  1  1  frame 07 {0} XYZ".format(lToS(lgsm.Displacement())))
#TT.s.updateTasks("updateTask  taskFrame  -Kp {0}  -Kd {1}  -posDes {2}  -velDes {3}  endUpdate".format(Kp, Kd, lToS(posDes), lToS(velDes)))


#################################################
print "SET INIT STATE..."
robot.setJointPositions(np.array([.4]*7).reshape(7,1))
dm.setJointPositions(np.array([.4]*7).reshape(7,1))
robot.setJointVelocities(np.array([0.0]*7).reshape(7,1))
dm.setJointVelocities(np.array([0.0]*7).reshape(7,1))

taskQRead = dsimi.rtt.Task(rtt_interface_corba.GetProxy("kukaqreader"))
import taskMan
taskm = taskMan.createTaskMan("taskman", TT, physic.dynmodel)

taskQRead = dsimi.rtt.Task(rtt_interface_corba.GetProxy("kukaqreader"))
taskm.getPort("q").connectTo( taskQRead.getPort("q_out") )

#Enable contacts
physic.robot.enableContactWithBody("ground", True)
physic.robot.enableContactWithBody("env1", True)

taskm.s.setPeriod(TIME_STEP)
taskm.s.start()
phy.s.setPeriod(TIME_STEP)
graph.s.start()
phy.s.start()
TT.s.setPeriod(TIME_STEP)
#TT.s.start()
clock.s.start()

shell()
