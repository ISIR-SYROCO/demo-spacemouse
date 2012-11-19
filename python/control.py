import lgsm
import rtt_interface
import dsimi.rtt
import numpy as np

controller = None

class SimpleController(dsimi.rtt.Task):
	def __init__(self, name, time_step, model):
		super(SimpleController, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))

		# Create ports to read the state of the robot from the physical scene
		self.q_port = self.addCreateInputPort("q", "VectorXd", True)
		self.qdot_port = self.addCreateInputPort("qdot", "VectorXd", True)
		self.d_port = self.addCreateInputPort("d", "Displacementd", True)
		self.t_port = self.addCreateInputPort("t", "Twistd", True)
		self.tau_port = self.addCreateOutputPort("tau", "VectorXd")

		# Ref to the dynamic model
		self.model = model
		self.kp = 100
		self.kd = 20

		# Flag for position control/gravity compensation
		self.enable_pos_control = False
		self.enable_gravity_comp = True

		# Target position of the end effector
		self.target_pos = np.matrix([ [0.5], [0.3], [0.3] ])

		self.s.setPeriod(time_step)

	def startHook(self):
		pass

	def stopHook(self):
		pass

	def enablePositionControl(self, enable):
		self.enable_pos_control = enable

	def updateHook(self):
		q,qok = self.q_port.read()
		qdot, qdotok = self.qdot_port.read()
		d, dok = self.d_port.read()
		t, tok = self.t_port.read()

		tau = lgsm.vector([0] * self.model.nbInternalDofs())

		if qok and qdotok and dok and tok:
			model = self.model

			# Refresh dynamic and kinematic models
			model.setFreeFlyerPosition(d)
			model.setFreeFlyerVelocity(t)
			model.setJointPositions(q)
			model.setJointVelocities(qdot)

			N = model.nbDofs()
			Na = model.nbInternalDofs() # actuated dof

			# Control
			H7 = model.getSegmentPosition(model.getSegmentIndex("07"))
			H = lgsm.Displacement(lgsm.vector(0,0,0), H7.getRotation() )
			J77 = model.getSegmentJacobian(model.getSegmentIndex("07"))
			J70 = H.adjoint() * J77
			T70 = H.adjoint() * model.getSegmentVelocity(model.getSegmentIndex("07"))

			xd = self.target_pos # desired position of the last segment
			x = H7.getTranslation()
			v = T70.getLinearVelocity()
			fc = self.kp * (xd -x) - self.kd * v

			if self.enable_gravity_comp:
				tau += model.getGravityTerms()

			if self.enable_pos_control:
				tau = tau + J70.transpose() * fc

			self.tau_port.write(tau)

def createController(name, time_step, model):
	#task = rtt_interface.PyTaskFactory.CreateTask("kukacontroller")
	controller = SimpleController(name, time_step, model)
	setProxy(controller)
	return controller

def setProxy(_controller):
	global controller
	controller=_controller

