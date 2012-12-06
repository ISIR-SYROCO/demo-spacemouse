import lgsm
import rtt_interface
import dsimi.rtt
import numpy as np

sm_interface = None

class SM_interface(dsimi.rtt.Task):
	def __init__(self, name, sm, freebody, time_step):
		super(SM_interface, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

		self.sm_in_port = self.addCreateInputPort("sm_in", "Twistd", True)
		self.sm = sm
		self.freebody = freebody

		self.getPort("sm_in").connectTo(self.sm.getPort("out_vel"))

		self.s.setPeriod(time_step)

	def startHook(self):
		pass

	def stopHook(self):
		pass

	def updateHook(self):

		t,tok = self.sm_in_port.read()

		if tok :
			self.freebody.setVelocity(t)
		else:
			self.freebody.setVelocity(lgsm.Twist())

def createSM_interface(name, sm, freebody, time_step):
	sm_interface = SM_interface(name, sm, freebody, time_step)
	setProxy(sm_interface)
	return sm_interface

def setProxy(_sm_interface):
	global sm_interface
	sm_interface = _sm_interface
