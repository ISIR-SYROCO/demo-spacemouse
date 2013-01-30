import lgsm
import rtt_interface
import dsimi.rtt
import physicshelper

qreader = None

class QReader(dsimi.rtt.Task):
	def __init__(self, name, time_step):
		super(QReader, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

		self.s.setPeriod(time_step)

		#Create the port to read robot state
		self.qout_port = self.addCreateOutputPort("q_out", "VectorXd")

		self.model = None

	def connectToRobot(self, phy, world, robot_name):
		self.model = physicshelper.createDynamicModel(world, robot_name)

	def startHook(self):
		pass

	def stopHook(self):
		pass

	def updateHook(self):
		q = self.model.getJointPositions()

		self.qout_port.write(q)

def createQReader(name, time_step):
	qreader = QReader(name, time_step)
	setProxy(qreader)
	return qreader

def setProxy(_qreader):
	global qreader
	qreader = _qreader
