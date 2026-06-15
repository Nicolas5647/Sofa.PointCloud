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

def Container(name, position, splat):
    with Node(name) as container:
        with Node("geometry") as geometry:
            Object("PointCloudContainer", name="splats", filename=splat)
            
        with Node("mechanical") as mechanical:
            Object("MechanicalObject", name="state", template="Rigid3", 
                                       position=position)
            
        with Node("visual") as visual:
            geometry.splats.init()
            indices = [0]*len(geometry.splats.indices.value)
            Object("PointCloudVisualModel", name="renderer",
                                        indices=geometry.splats.indices.linkpath,
                                        geometries=[geometry.splats.linkpath],
                                        frames=mechanical.state.position.linkpath,
                                        frameIndices=indices)
    return container

def Lod(name, position, splats, distances):
    with Node(name) as quad:
        with Node("geometries") as geometries:
            for i, filename in enumerate(splats):
                Object("PointCloudContainer", name=f"lod_{i}", filename=filename)

        with Node("mechanical") as mechanical:
            Object("MechanicalObject", name="state", template="Rigid3", 
                                       position=position)
            
        with Node("selection") as selection:
            lod_paths = [f"@../geometries/lod_{i}" for i in range(len(splats))]
            Object("PointCloudLodSelector", name="lodSelector",
                    geometries=lod_paths,
                    thresholds=distances,
                    printLog=True)
        
        with Node("visual") as visual:
            for i, filename in enumerate(splats):
                geometries[f"lod_{i}"].init()
            lod_paths = [f"@../geometries/lod_{i}" for i in range(len(splats))]
            indices = [0]*len(geometries.lod_0.indices.value)
            Object("PointCloudVisualModel", name="renderer",
                            geometries=lod_paths,
                            frames=mechanical.state.position.linkpath,
                            selector=selection.lodSelector.linkpath,
                            frameIndices=indices)
    return quad

@SofaScene
def createScene(root):
    """Démonstration du contrôle d'un nuage de points avec LOD/Octree"""  
    
    useLOD = True
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")
    root.addObject("BackgroundSetting", name="settings", color=[0.1, 0.1, 0.1, 1.0])    
    
    root.addObject("InteractiveCamera", name="camera", position=[1.25331, -5.37672, -5.57541, 0.918399, 0.173357, -0.0340889, 0.354019], zFar=1000)
    root.addObject("PointCloudRenderer", name="pointRenderer", camera=root.camera.linkpath)
    root.addObject(animation.AnimationManager(root))

    with Node("Modelling"):
        if useLOD:
            files = ["splats/spot/spot_r4.ply", "splats/spot/spot_r10.ply", "splats/spot/spot_r18.ply"]
            dist_thresholds = [10, 20, 30] 
            
            with Lod("spot_lod", position=[0,0,0,0,0,0,1], splats=files, distances=dist_thresholds) as lod:
                lod.addObject(DofManipulator(name="manipulator", direction=1, target=lod))
        else:
            # Mode simple sans LOD
            with Container("spot_simple", position=[0,0,0,0,0,0,1], splat="splats/spot/spot_r4.ply") as container:
                container.addObject(DofManipulator(name="manipulator", direction=1, target=container))