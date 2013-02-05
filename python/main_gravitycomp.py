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
wm = xwm.WorldManager()
wm.createAllAgents(TIME_STEP)

import xde_spacemouse as spacemouse

import xde_resources as xr

import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

import rtt_interface_corba
rtt_interface_corba.Init(sys.argv)

#Creating Scene

mecha_name = "kuka1"
kukaWorld = xrl.createWorldFromUrdfFile(xr.kuka, mecha_name, [0.5,0.2,0.4, 1, 0, 0, 0], True, 1, 0.005) #, "material.concrete")
xrl.addContactLaws(kukaWorld)

groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0.0, 1, 0, 0, 0], True, 0.1, 0.05) #, "material.concrete")

env1World = xrl.createWorldFromUrdfFile(xr.env1, "env1", [0,0.2,0.1, 1, 0, 0, 0], True, 0.1, 0.05)

sphereWorld = xrl.createWorldFromUrdfFile(xr.sphere, "sphere", [0,0.6,1.2, 1, 0, 0, 0], False, 0.2, 0.005)# , "material.concrete")

wm.addWorld(groundWorld)
wm.addWorld(kukaWorld)
wm.addWorld(env1World)
wm.addMarkers(sphereWorld, ["sphere.sphere"], thin_markers=False)
wm.addWorld(sphereWorld, True)

kuka = wm.phy.s.GVM.Robot(mecha_name)
kuka.enableContactWithBody("ground.ground", True)
kuka.enableContactWithBody("env1.env1", True)

# Create simple gravity compensator controller
import control
controller = control.createTask("controlKuka1", TIME_STEP)
controller.connectToRobot(wm.phy, kukaWorld, mecha_name)

#spacemouse
#PDC Control mode
sm = spacemouse.createTask("smi", TIME_STEP, wm.phy, wm.graph, "sphere.sphere", pdc_enabled=True, body_name=mecha_name+".07")

import qreader
qreader = qreader.createQReader("kukaqreader", TIME_STEP)
qreader.connectToRobot(kuka)
rtt_interface_corba.SetServer(qreader._obj)

sm.s.start()
qreader.s.start()
controller.s.start()
wm.startSimulation()

shell()
