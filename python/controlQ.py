import lgsm
import rtt_interface
import dsimi.rtt
import numpy as np

controllerQ = None

class ControllerQ(dsimi.rtt.Task):
	def __init__(self, name, time_step, model):
		super(ControllerQ, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

		# Create ports to read the state of the robot from the physical scene
		self.qdes_port = self.addCreateInputPort("qdes", "VectorXd", True)
		self.q_port = self.addCreateInputPort("q", "VectorXd", True)
		self.qdot_port = self.addCreateInputPort("qdot", "VectorXd", True)
		self.d_port = self.addCreateInputPort("d", "Displacementd", True)
		self.t_port = self.addCreateInputPort("t", "Twistd", True)
		self.tau_port = self.addCreateOutputPort("tau", "VectorXd")

		# Ref to the dynamic model
		self.model = model
		self.kp = 90
		self.kd = 2

		# Flag for position control/gravity compensation
		self.enable_pos_control = True
		self.enable_gravity_comp = True

		# Target position of the end effector
		nbDofs = self.model.nbDofs()
		self.target_pos = np.matrix(np.ones((nbDofs, 1)))

		self.s.setPeriod(time_step)

	def startHook(self):
		pass

	def stopHook(self):
		pass

	def enablePositionControl(self, enable):
		self.enable_pos_control = enable

	def updateHook(self):
		model = self.model

		q,qok = self.q_port.read()
		qdot, qdotok = self.qdot_port.read()
		qdes, qdesok = self.qdes_port.read()
		d, dok = self.d_port.read()
		t, tok = self.t_port.read()

		#tau = np.zeros((model.nbDofs(),1))
		tau = lgsm.vector([0] * model.nbInternalDofs())

		if qok and qdotok and dok and tok and qdesok:
			# Refresh dynamic and kinematic models
			model.setFreeFlyerPosition(d)
			model.setFreeFlyerVelocity(t)
			model.setJointPositions(q)
			model.setJointVelocities(qdot)

			N = model.nbDofs()
			Na = model.nbInternalDofs() # actuated dof

			# Control
			#qdes = self.target_pos # desired position of the last segment
			q = self.model.getJointPositions()
			v = self.model.getJointVelocities()
			fc = self.kp * (qdes - q) + self.kd * v

			if self.enable_gravity_comp:
				tau = model.getGravityTerms()

			if self.enable_pos_control:
				tau = tau + fc

			self.tau_port.write(tau)

def createControllerQ(name, time_step, model):
	controllerQ = ControllerQ(name, time_step, model)
	setProxy(controllerQ)
	return controllerQ

def setProxy(_controllerQ):
	global controllerQ
	controllerQ = _controllerQ

