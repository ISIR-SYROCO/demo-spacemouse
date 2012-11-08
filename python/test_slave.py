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

import dummyGetQ

dumm = dummyGetQ.createDummyGetQ("dummyget")

dumm.s.setPeriod(TIME_STEP)

taskQRead = dsimi.rtt.Task(rtt_interface_corba.GetProxy("kukaqreader"))
dumm.getPort("q").connectTo( taskQRead.getPort("q_out") )

shell()
