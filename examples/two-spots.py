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
            angle=self.direction*factor*3.14*2.0
            ref = target.mechanical.state.rest_position.value
            with target.mechanical.state.position.writeableArray() as w:
                for i in range(w.shape[0]):
                    q = numerics.Quat.createFromAxisAngle([0,0,1],angle+float(i)*angle*0.2)
                    x = numerics.Quat(ref[i,3:])
                    r = numerics.Quat.product(q,x)
                    w[i,3:]=r

        animation.animate(myAnimate1, {"target" : self.target}, 1, mode="loop")

def Spot(name, position):
    with Node(name) as spot:
        with Node("geometry") as geometry:
            Object("PointCloudContainer", name="splats", filename="splats/spot/spot.ply")
            
        with Node("mechanical") as mechanical:
            Object("MechanicalObject", name="state", template="Rigid3", 
                                       position=position)
            
        with Node("visual"):
            geometry.splats.init()
            indices = [0]*len(geometry.splats.indices.value)
            Object("PointCloudVisualModel", name="renderer", 
                                        indices=geometry.splats.indices.linkpath, 
                                        geometries=[geometry.splats.linkpath], 
                                        frames=mechanical.state.position.linkpath,
                                        frameIndices=indices)
    return spot


@SofaScene
def createScene(root):
    """Demonstrate the control of a point cloud by a sofa rigid Frame
    """
    Object("RequiredPlugin", name="Sofa.PointCloud")
    Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    Object("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    
    Object("PointCloudRenderer", name="renderer", camera=root.camera.linkpath)    

    Object(animation.AnimationManager(root))

    with Node("Modelling"):
        with Spot("spot1", position=[0,0,0,0,0,0,1]) as spot1:
            spot1.addObject(DofManipulator(name="manipulator", direction=1, target=spot1))
    
        with Spot("spot2", position=[0,1,0,0,0,0,1]) as spot2:
            spot2.addObject(DofManipulator(name="manipulator", direction=-1, target=spot2))
    
    spot1.mechanical.state.showObject = True
    spot2.mechanical.state.showObject = True

