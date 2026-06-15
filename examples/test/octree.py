import Sofa
from Sofa.Types import RGBAColor
from splib3 import animation
from splib3 import numerics
import math
from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
import os
import SofaRuntime

class DofManipulator(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.target = kwargs.get("target", None)
        self.direction = kwargs.get("direction", 1)

        def myAnimate1(target, factor):
            angle = self.direction * factor * 3.14 * 2.0
            state = target.mechanical.state
            ref = state.rest_position.value
            with state.position.writeableArray() as w:
                for i in range(w.shape[0]):
                    q = numerics.Quat.createFromAxisAngle([0,0,1], angle + float(i)*angle*0.2)
                    x = numerics.Quat(ref[i,3:])
                    r = numerics.Quat.product(q, x)
                    w[i,3:] = r

        animation.animate(myAnimate1, {"target" : self.target}, 1, mode="loop")

def Container(name, position, splat):
    with Node(name) as spot:
        with Node("geometry") as geometry:
            Object("PointCloudContainer", name="splats", filename=splat)
            
        with Node("mechanical") as mechanical:
            Object("MechanicalObject", name="state", template="Rigid3", 
                                       position=position)
            
        with Node("visual") as visual:
            Object("PointCloudVisualModel", name="renderer", 
                                        geometries=[geometry.splats.linkpath],
                                        frames=mechanical.state.position.linkpath)
    return spot

def Octree(name, position, splat, maxSplats):
    with Node(name) as quad:
        with Node("geometries") as geometries:
            Object("PointCloudContainer", name="container", filename=splat)

        with Node("mechanical") as mechanical:
            Object("MechanicalObject", name="state", template="Rigid3", 
                                       position=position)
            
        with Node("selection") as selection:
            Object("PointCloudOctreeSelector", name="octreeSelector",
                    geometries=[geometries.container.linkpath],
                    maxSplats=maxSplats,
                    showIntersecCube=True)
        
        with Node("visual") as visual:
            geometries.container.init()
            indices = [0]*len(geometries.container.indices.value)
            Object("PointCloudVisualModel", name="renderer",
                            geometries=[geometries.container.linkpath],
                            frames=mechanical.state.position.linkpath,
                            selector=selection.octreeSelector.linkpath,
                            frameIndices=indices)
    return quad

@SofaScene
def createScene(root):
    """Démonstration du contrôle d'un nuage de points avec LOD/Octree"""  
    
    useLOD = True
    Object("RequiredPlugin", name="Sofa.PointCloud")
    Object("BackgroundSetting", name="settings", color=[0.1, 0.1, 0.1, 1.0])    
    
    Object("InteractiveCamera", name="camera", position=[2.90374, -5.65386, 6.267, -0.170925, 0.452691, -0.870314, 0.0917029], zFar=1000)
       
    Object('OglViewport', name='viewportView', 
                cameraPosition= [2.90374, -5.65386, 6.267],
                cameraOrientation= [-0.170925, 0.452691, -0.870314, 0.0917029],
                screenPosition=[0,0], 
                screenSize=[300, 150],
                useFBO= False,
                zNear= 0.1,
                zFar= 1000)
   
    Object("PointCloudRenderer", name="pointRenderer", camera=root.camera.linkpath)
    Object(animation.AnimationManager(root))

    with Node("Modelling"):
        if useLOD:            
            with Octree("spot_octree", position=[0,0,0,0,0,0,1], splat="splats/truck.ply", maxSplats=100) as lod:
                lod.addObject(DofManipulator(name="manipulator", direction=1, target=lod))
        else:
            # Mode simple sans LOD
            with Container("spot_simple", position=[0,0,0,0,0,0,1], splat="splats/truck.ply") as container:
                container.addObject(DofManipulator(name="manipulator", direction=1, target=container))