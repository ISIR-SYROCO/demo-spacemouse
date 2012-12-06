import lgsm
import rtt_interface as rtt
import dsimi.rtt
import numpy as np

def lToS(L):
    return " ".join(str(v) for v in L)

class KeyboardInterface(dsimi.rtt.Task):
	def __init__(self, name, period, targetPos, controller, keyb_port, kp, kd, velDes, markerName, graph_scn ):
		super(KeyboardInterface, self).__init__(rtt.PyTaskFactory.CreateTask(name))
		# periodic task: 10ms
		self.s.setPeriod(period)
		# create a port of std::pair<int, int>
		self.in_keyb = self.addCreateInputPort("keyb", "pair_int_int", True)
		self.in_keyb.connectTo(keyb_port)
		self.controller = controller
		self.targetPos = targetPos
		self.Kp = kp
		self.Kd = kd
		self.velDes = velDes
		self.graph_scn = graph_scn
		self.markerName = markerName
# arrows value
#   200
# 203 205
#   208
# q 16
# a 30
# d 32
# w 17
# x 45

	def setTranslation(self, axis, step):
		current = self.targetPos.getTranslation()
		current[axis, 0] += step
		self.targetPos.setTranslation(current)

	def increaseX(self):
		self.setTranslation(0, 0.01)

	def decreaseX(self):
		self.setTranslation(0, -0.01)

	def increaseY(self):
		self.setTranslation(1, 0.01)

	def decreaseY(self):
		self.setTranslation(1, -0.01)

	def increaseZ(self):
		self.setTranslation(2, 0.01)

	def decreaseZ(self):
		self.setTranslation(2, -0.01)

	def updateHook(self):
		(state, key), new_data = self.in_keyb.read()
		if new_data:
			if key == 16: 	 self.increaseX()
			elif key == 30: self.decreaseX()
			elif key == 17: self.increaseY()
			elif key == 31: self.decreaseY()
			elif key == 18:  self.increaseZ()
			elif key == 32:  self.decreaseZ()
			else: print "key: ", key
			self.graph_scn.MarkersInterface.setMarker6DPosition(self.markerName, self.targetPos)
			self.controller.s.updateTasks("updateTask  taskFrame  -Kp {0}  -Kd {1}  -posDes {2}  -velDes {3}  endUpdate".format(self.Kp, self.Kd, lToS(self.targetPos), lToS(self.velDes)))


