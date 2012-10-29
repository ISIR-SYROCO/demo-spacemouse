import agents.graphic.core
import agents.graphic.simple
import agents.graphic.proto
import agents.graphic.builder

graph = None
graph_scn = None

def createTask():
  graph = agents.graphic.simple.createAgent("graphic", 0)
  setProxy(graph)
  return graph

def setProxy(_graph):
  global graph
  graph = _graph

def init():
  global graph_scn

  graph_scn,scene_name,window_name,viewport_name = agents.graphic.simple.setupSingleGLView(graph)

  agents.graphic.proto.configureBasicLights(graph_scn)
  agents.graphic.proto.configureBasicCamera(graph_scn)

  graph.s.Viewer.enableNavigation(True)
  graph_scn.SceneryInterface.showGround(False)

  graph.s.Connectors.IConnectorBody.new("icf", "body_state_H", "mainScene")
  graph.s.Connectors.IConnectorFrame.new("icm", "frame_state_H", "mainScene")

  icc = graph.s.Connectors.IConnectorContacts.new("icc", "contacts", "mainScene")
  icc.setMaxProximity(.05)
  icc.setGlyphScale(2)

def deserializeWorld(world):
  agents.graphic.builder.deserializeWorld(graph, graph_scn, world)

def startTask():
  graph.s.start()

