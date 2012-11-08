import lgsm
import rtt_interface
import dsimi.rtt
import numpy as np

qreader = None

class QReader(dsimi.rtt.Task):
	def __init__(self, name, model):
		super(QReader, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

		#Create the port to read robot state
		self.qout_port = self.addCreateOutputPort("q_out", "VectorXd")

		self.model = model

	def startHook(self):
		pass

	def stopHook(self):
		pass

	def updateHook(self):
		q = self.model.getJointPositions()

		self.qout_port.write(q)

def createQReader(name, model):
	qreader = QReader(name, model)
	setProxy(qreader)
	return qreader

def setProxy(_qreader):
	global qreader
	qreader = _qreader
