import agents.physic.core
import agents.physic.builder
import physicshelper

phy = None
ms = None
xcd = None
robot = None
dynmodel = None

def createTask():
	phy = agents.physic.core.createAgent("physic", 0)
	setProxy(phy);
	return phy

def setProxy(_phy):
	global phy
	phy = _phy

def init(_timestep):
	global ms, xcd, phy

	ms = agents.physic.core.createGVMScene(phy, "main", time_step=_timestep, uc_relaxation_factor=.1)
	xcd = agents.physic.core.createXCDScene(phy, "xcd", "LMD", lmd_max=0.01)
	ms.setGeometricalScene(xcd)

	# create material. Used to defined a contact law
	ms.addContactMaterial("gum") # Define a new material
	ms.addContactMaterial("metal")
	ms.setContactLawForMaterialPair("gum", "gum", 1, 1.) # Set the contact parameters for the material pair
	ms.setContactLawForMaterialPair("gum", "metal", 1, .5)
	ms.setContactLawForMaterialPair("metal", "metal", 0, .2) # .2 is ignored

	phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")
	phy.s.Connectors.OConnectorContactBody.new("occ", "contacts")

def createDynamicModel(world, mechaName):
	global phy
	global robot, dynmodel
	robot = phy.s.GVM.Robot(mechaName)
	dynmodel = physicshelper.createDynamicModel(world, mechaName)
	return dynmodel

def deserializeWorld(world, mechaName):
	global phy
	agents.physic.builder.deserializeWorld(phy, ms, xcd, world)

	ocb = phy.s.Connectors.OConnectorBodyStateList("ocb")

	for b in world.scene.rigid_body_bindings:
		if len(b.graph_node) and len(b.rigid_body):
			ocb.addBody(str(b.rigid_body))

	occ = phy.s.Connectors.OConnectorContactBody("occ")
	occ.addInteraction(phy.s.GVM.Robot(mechaName).getSegmentRigidBody2("03"), "ground")
	occ.addInteraction(phy.s.GVM.Robot(mechaName).getSegmentRigidBody2("04"), "ground")
	occ.addInteraction(phy.s.GVM.Robot(mechaName).getSegmentRigidBody2("05"), "ground")
	occ.addInteraction(phy.s.GVM.Robot(mechaName).getSegmentRigidBody2("06"), "ground")
	occ.addInteraction(phy.s.GVM.Robot(mechaName).getSegmentRigidBody2("07"), "ground")

	phy.s.Connectors.IConnectorRobotJointTorque.new("ict", mechaName+"_", mechaName)
	phy.s.Connectors.OConnectorRobotState.new("ocpos", mechaName+"_", mechaName)

def startTask():
	phy.s.start()

