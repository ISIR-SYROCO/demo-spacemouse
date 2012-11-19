import desc.robot
import desc.simple.physic
import desc.simple.scene
import desc.simple.graphic
import desc.simple.collision
import desc.scene
import desc.material
import desc.graphic
import desc.core
import desc.physic
import lgsm
import numpy as np
import math

RESOURCES_PATH = "resources/dae/"

def buildKuka(mechaName, color="Blue"):
	kukaworld = desc.scene.parseColladaFile(RESOURCES_PATH + "kuka_lwr.dae")
	root_node = kukaworld.scene.graphical_scene.root_node

	def setNodePosition(node, H):
		node.ClearField("position")
		node.position.extend(H.tolist())
	map(lambda node: setNodePosition(node, lgsm.Displacementd()), root_node.children)

	mat = kukaworld.library.materials.add()
	mat.name = "xde/BlueOpaqueAvatars"
	desc.material.fillColorMaterial(mat, [0.1,0.1,0.2,1.0]) # R,G,B,A

	desc.graphic.applyMaterialSet(root_node, material_set=["xde/"+color+"OpaqueAvatars"])

	#create material
	#mat = kukaworld.library.materials.add()
	#mat.name = "purple"
	#desc.material.fillColorMaterial(mat, [1,0,1,1]) # R,G,B,A
	kuka_mass = 11.5
	kuka_damping = 100.0 # TODO!!!!!! change to 1.0 in the original script

	H00 = lgsm.Displacementd(0.0, 0., 0.0, 0, 0., 0., 1.)
	H01 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
	H12 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
	H23 = lgsm.Displacement(lgsm.vectord(0,0.,0.4) , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
	H34 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[-1,0,0],[0,-1,0],[0,0,1]])))
	H45 = lgsm.Displacement(lgsm.vectord(0,0.,0.39), lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))
	H56 = lgsm.Displacement(lgsm.vectord(0,0.,0)   , lgsm.Rotation3.fromMatrix(np.matrix([[0,1,0],[-1,0,0],[0,0,1]])))
	H67 = lgsm.Displacement(lgsm.vectord(0,0.,0.)   , lgsm.Rotation3.fromMatrix(np.matrix([[1,0,0],[0,1,0],[0,0,1]])))

#    kinematic_tree = ("00", kuka_mass, H00, [], [
#      ("01", kuka_mass, H01, [('hinge', [0,0,0], [0,0,1], kuka_damping, -170*math.pi/180, 170*math.pi/180, -45*math.pi/180)], [
#       ("02", kuka_mass, H12, [('hinge', [0,0,0], [0,1,0], kuka_damping, -120*math.pi/180, 120*math.pi/180, -2*30*math.pi/180)], [
#        ("03", kuka_mass, H23, [('hinge', [0,0,0], [0,0,1], kuka_damping, -170*math.pi/180, 170*math.pi/180, 60*math.pi/180)], [
#         ("04", kuka_mass, H34, [('hinge', [0,0,0], [0,1,0], kuka_damping, -120*math.pi/180, 120*math.pi/180, 45*math.pi/180)], [
#          ("05", kuka_mass, H45, [('hinge', [0,0,0], [0,0,1], kuka_damping, -170*math.pi/180, 170*math.pi/180, 0*math.pi/180)], [
#           ("06", kuka_mass/2, H56, [('hinge', [0,0,0], [0,1,0], kuka_damping, -120*math.pi/180, 120*math.pi/180, 90*math.pi/180)], [
#            ("07", kuka_mass/20, H67, [('hinge', [0,0,0], [0,0,1], kuka_damping, -170*math.pi/180, 170*math.pi/180, 90*math.pi/180)], [
#            ])
#           ])
#          ])
#         ])
#        ])
#       ])
#      ])
#     ])


	kinematic_tree = ("00", kuka_mass, H00, [], [
	  ("01", kuka_mass, H01, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, -45*math.pi/180)], [
	   ("02", kuka_mass, H12, [('hinge', [0,0,0], [0,1,0], kuka_damping, -320*math.pi/180, 320*math.pi/180, -2*30*math.pi/180)], [
	    ("03", kuka_mass, H23, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, 60*math.pi/180)], [
	     ("04", kuka_mass, H34, [('hinge', [0,0,0], [0,1,0], kuka_damping, -320*math.pi/180, 320*math.pi/180, 45*math.pi/180)], [
	      ("05", kuka_mass, H45, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, 0*math.pi/180)], [
	       ("06", kuka_mass/2, H56, [('hinge', [0,0,0], [0,1,0], kuka_damping, -320*math.pi/180, 320*math.pi/180, 90*math.pi/180)], [
	        ("07", kuka_mass/20, H67, [('hinge', [0,0,0], [0,0,1], kuka_damping, -370*math.pi/180, 370*math.pi/180, 90*math.pi/180)], [
	        ])
	       ])
	      ])
	     ])
	    ])
	   ])
	  ])
	 ])

	root = desc.robot.addKinematicTree(kukaworld.scene.physical_scene, parent_node=None, tree=kinematic_tree, fixed_base=True, H_init=lgsm.Displacementd())

	def setNodeMaterial(node):
	    node.rigid_body.contact_material = "material.metal"
	desc.core.visitDepthFirst(setNodeMaterial, root)

	kuka_offset = 0.004

	def createKukaComposite(node_name, composite_name):
	    graph_node = desc.core.findInTree(kukaworld.scene.graphical_scene.root_node, node_name)
	    composite = desc.collision.addCompositeMesh(kukaworld.scene.collision_scene, composite_name, offset=kuka_offset)
	    desc.collision.copyFromGraphicalTree(composite.root_node, graph_node)
	    composite.root_node.ClearField("position")
	    composite.root_node.position.extend([0,0,0,1,0,0,0])

	createKukaComposite("kuka_base", "00.comp")
	createKukaComposite("kuka_1", "01.comp")
	createKukaComposite("kuka_2", "02.comp")
	createKukaComposite("kuka_3", "03.comp")
	createKukaComposite("kuka_4", "04.comp")
	createKukaComposite("kuka_5", "05.comp")
	createKukaComposite("kuka_6", "06.comp")
	createKukaComposite("kuka_7", "07.comp")

	# association between physics, graphics and collision

	def createKukaBinding(node_name, body_name, comp_name):
	    graph_node = desc.core.findInTree(kukaworld.scene.graphical_scene.root_node, node_name)
	    graph_node.name = body_name # it is suitable to have the same name for both graphics and physics.
	    desc.scene.addBinding(kukaworld, body_name, body_name, "", comp_name)

	createKukaBinding("kuka_base", "00", "00.comp")
	createKukaBinding("kuka_1", "01", "01.comp")
	createKukaBinding("kuka_2", "02", "02.comp")
	createKukaBinding("kuka_3", "03", "03.comp")
	createKukaBinding("kuka_4", "04", "04.comp")
	createKukaBinding("kuka_5", "05", "05.comp")
	createKukaBinding("kuka_6", "06", "06.comp")
	createKukaBinding("kuka_7", "07", "07.comp")

	kuka_segments = ["00", "01", "02", "03", "04", "05", "06", "07"]
	kuka_bodies = kuka_segments

	desc.physic.addMechanism(kukaworld.scene.physical_scene, mechaName, "00", [], kuka_bodies, kuka_segments)
	return kukaworld

def addObjectFromDae(world, filename, nodename, objectmass, objectmaterial="material.concrete", scale = [1, 1, 1]):
	object_world = desc.simple.scene.parseColladaFile(RESOURCES_PATH+filename)

	setNodeScale = lambda node: desc.graphic.setNodeScale(node, scale)
	map(setNodeScale, object_world.scene.graphical_scene.root_node.children)

	desc.simple.graphic.addGraphicalTree(world, object_world, node_name=nodename)
	desc.simple.collision.addCompositeMesh(world, object_world, composite_name=nodename+".comp", offset=0.0, clean_meshes=False, ignore_library_conflicts=True)
	desc.simple.physic.addRigidBody(world, nodename, mass=objectmass, contact_material=objectmaterial)
	desc.simple.physic.addFixedJoint(world, nodename+".joint", nodename, lgsm.Displacementd(-0.3,0.2,-0.3))   #(0,0.4,0)
	desc.simple.scene.addBinding(world, phy="env1", graph="env1", graph_ref="", coll="env1.comp")

def addGround(world):
	ground_world = desc.simple.scene.parseColladaFile(RESOURCES_PATH+"ground.dae")
	root_node = ground_world.scene.graphical_scene.root_node
	phy_ground_world = desc.simple.scene.parseColladaFile(RESOURCES_PATH+"ground_phy_50mm.dae", append_label_library=".phyground")

	mat = ground_world.library.materials.add()
	mat.name = "grass"
	desc.material.fillColorMaterial(mat, [0.01,0.1,0.01,1.0]) # R,G,B,A

	desc.graphic.applyMaterialSet(root_node, material_set=["grass"])

	desc.simple.graphic.addGraphicalTree(world, ground_world, node_name="ground")
	desc.simple.collision.addCompositeMesh(world, phy_ground_world, composite_name="ground.comp", offset=0.05, clean_meshes=False, ignore_library_conflicts=False)
	desc.simple.physic.addRigidBody(world, "ground", mass=1, contact_material="material.concrete")
	desc.simple.physic.addFixedJoint(world, "ground.joint", "ground", lgsm.Displacementd(0,0,-0.4))
	desc.simple.scene.addBinding(world, phy="ground", graph="ground", graph_ref="", coll="ground.comp")

def addWall(world):
	env1 = desc.simple.scene.parseColladaFile(RESOURCES_PATH+"env1.dae")

	setNodeScale = lambda node: desc.graphic.setNodeScale(node, [.6, .6, .6])
	map(setNodeScale, env1.scene.graphical_scene.root_node.children)

	desc.simple.graphic.addGraphicalTree(world, env1, node_name="env1")
	desc.simple.collision.addCompositeMesh(world, env1, composite_name="env1.comp", offset=0.0, clean_meshes=False, ignore_library_conflicts=True)
	desc.simple.physic.addRigidBody(world, "env1", mass=1, contact_material="material.concrete")
	desc.simple.physic.addFixedJoint(world, "env1.joint", "env1", lgsm.Displacementd(-0.3,0.2,-0.3))   #(0,0.4,0)
	desc.simple.scene.addBinding(world, phy="env1", graph="env1", graph_ref="", coll="env1.comp")

def addContactLaws(world):
	world.scene.physical_scene.contact_materials.extend(["material.concrete", "material.metal"])
	cl = world.scene.physical_scene.contact_laws.add()
	cl.material_i = "material.metal"
	cl.material_j = "material.concrete"
	cl.law = cl.COULOMB
	cl.friction = .4

def addCollisionPairs(world, body, mechaName):
	cp = world.scene.collision_pairs.add()
	cp.body_i = body
	cp.mechanism_i = "mechaName"
	#cp.enabled = True

def addCube(world):
	cube_world = desc.simple.scene.parseColladaFile(RESOURCES_PATH+"knob.dae")

	setNodeScale = lambda node: desc.graphic.setNodeScale(node, [.06, .06, .06])
	map(setNodeScale, cube_world.scene.graphical_scene.root_node.children)

	desc.simple.graphic.addGraphicalTree(world, cube_world, node_name="cube")
	desc.simple.collision.addCompositeMesh(world, cube_world, composite_name="cube.comp", offset=0.0, clean_meshes=False, ignore_library_conflicts=True)
	cube = desc.simple.physic.addRigidBody(world, "cube", mass=5, contact_material="material.concrete", weight_enabled=False)
	free = desc.simple.physic.addFreeJoint(world, "cube.joint", "cube", lgsm.Displacementd(0.001,0.02,1.7,1,0,0,0))   #(0,0.4,0)
	desc.simple.scene.addBinding(world, phy="cube", graph="cube", graph_ref="", coll="cube.comp")

	#phys_node = desc.physic.findInPhysicalScene(world.scene.physical_scene, "cube")
	#desc.physic.fillRigidBody( phys_node.rigid_body, mass=5, weight_enabled=False)
