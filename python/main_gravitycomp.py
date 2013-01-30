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

import xde_spacemouse as spacemouse

import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

import rtt_interface_corba
rtt_interface_corba.Init(sys.argv)

#Creating Scene

mecha_name = "kuka1"
kukaWorld = xrl.createWorldFromUrdfFile("resources/urdf/kuka.xml", mecha_name, [0.5,0.2,0.4, 1, 0, 0, 0], True, 1, 0.005) #, "material.concrete")
xrl.addContactLaws(kukaWorld)

groundWorld = xrl.createWorldFromUrdfFile("resources/urdf/ground.xml", "ground", [0,0,0.0, 1, 0, 0, 0], True, 0.1, 0.05) #, "material.concrete")

env1World = xrl.createWorldFromUrdfFile("resources/urdf/env1.xml", "env1", [0,0.2,0.1, 1, 0, 0, 0], True, 0.1, 0.05)

sphereWorld = xrl.createWorldFromUrdfFile("resources/urdf/sphere.xml", "sphere", [0,0.6,1.2, 1, 0, 0, 0], False, 0.2, 0.005)# , "material.concrete")

xwm.addWorld(groundWorld)
xwm.addWorld(kukaWorld)
xwm.addWorld(env1World)
xwm.addMarkers(sphereWorld, ["spheresphere"], thin_markers=False)
xwm.addWorld(sphereWorld, True)

kuka = phy.s.GVM.Robot(mecha_name)
kuka.enableContactWithBody("groundground", True)
kuka.enableContactWithBody("env1env1", True)

# Create simple gravity compensator controller
import control
controller = control.createTask("controlKuka1", TIME_STEP)
controller.connectToRobot(phy, kukaWorld, mecha_name)

#spacemouse
#PDC Control mode
sm = spacemouse.createTask("smi", TIME_STEP, phy, graph, "spheresphere", pdc_enabled=True, body_name="07"+mecha_name)

import qreader
qreader = qreader.createQReader("kukaqreader", TIME_STEP)
qreader.connectToRobot(kuka)
rtt_interface_corba.SetServer(qreader._obj)

sm.s.start()
qreader.s.start()
controller.s.start()
phy.s.startSimulation()

shell()
