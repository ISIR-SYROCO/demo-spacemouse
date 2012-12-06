import lgsm
import rtt_interface
import dsimi.rtt
import numpy as np

dummygetq = None

class DummyGetQ(dsimi.rtt.Task):
	def __init__(self, name):
		super(DummyGetQ, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

		#Create the port to read robot state
		self.q_port = self.addCreateInputPort("q", "VectorXd", True)

	def startHook(self):
		pass

	def stopHook(self):
		pass

	def updateHook(self):
		pass

def createDummyGetQ(name):
	dummygetq = DummyGetQ(name)
	setProxy(dummygetq)
	return dummygetq

def setProxy(_dummygetq):
	global dummygetq
	dummygetq = _dummygetq

