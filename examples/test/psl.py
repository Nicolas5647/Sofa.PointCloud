import Sofa
import inspect
import traceback
import os
import sys
import json

this_node = None
currentBase = None
currentNode = {}
depthLevel = 0
nameContext = {}

def __apply(self : Sofa.Core.Node, typeName, **kwargs):
    if not callable(typeName):
        raise Exception("Invalid parameter")
    return typeName(self, **kwargs)
Sofa.Core.Node.apply = __apply

def piggypatcher_query(filename, lineno, scenepath, id):
    return {}
    uid = f"{filename}_{lineno}_{scenepath}/{id}"
    uid = uid.replace("@/","root_")
    uid = uid.replace("/","_")
    
    pathname = f"init-data/{uid}.json"
    if not os.path.exists(pathname):
        with open(pathname,"wt") as file:
            file.write("{}")
    return dict(json.load(open(pathname)))

class NodeStack(object):
    def __init__(self, *args, **kwargs):
        self.nodes = []
    
    def push(self, node):
        self.nodes.append(node)

    def pop(self):
        return self.nodes.pop()

    def top(self):
        return self.nodes[-1]
__nodestack__ = NodeStack()

def set(**kwargs):
    global currentBase
    for k,v in kwargs.items():
        currentBase.findData(k).value = v

class InvalidObject(Sofa.Core.Controller):
  def __init__(self, *args, **kwargs):
    Sofa.Core.Controller.__init__(self, *args, **kwargs)

def Node(name, *args, **kwargs):
    global __nodestack__ , depthLevel, currentNode, nameContext, currentBase
    node = __nodestack__.top().addChild(name, *args, **kwargs)

    # Register the call point in the node 
    ss=inspect.stack()[1:2]
    node.setInstanciationSourceFileName(ss[0][1])
    node.setInstanciationSourceFilePos(ss[0][2])
    node.extra_context = ss[0][4]

    def enter(self):
        global depthLevel, currentNode, nameContext, currentBase
        __nodestack__.push(self)
        depthLevel+=3
        
        if self.name.value in nameContext:
            nameContext[self.name.value].append(self)
        else:
            nameContext[self.name.value] = []

        globals()[self.name.value] = self 
        globals()["this_node"] = self 
        currentBase = self 
     
    def exit(self, exc_type, exc_val, exc_tb):
        global depthLevel, currentNode, nameContext, currentBase, __nodestack__
        depthLevel-=3
        if self.name.value in nameContext:
            if len(nameContext[self.name.value]) != 0:
                globals()[self.name.value] = nameContext[self.name.value].pop()
            else:
                del globals()[self.name.value]     
        else:
            pass
            
        currentNode = __nodestack__.pop()
        globals()["this_node"] = __nodestack__.top()
        globals()["indent"] = depthLevel 
        currentBase = currentNode

        if exc_type is not None:
            import traceback
            traceback.print_exception(exc_type, exc_val, exc_tb)
            tmp = Sofa.sofaFormatHandler(exc_type, exc_val, exc_tb)
            tmp += "\nSofa Scene Hierarchy (most recent are at the end): \n"
        
            for snode in __nodestack__.nodes:
                tmp += f' Node("{snode.name.value}")\n'
                tmp += f'   File "{snode.getInstanciationFileName()}", line {snode.getInstanciationSourceFilePos()}\n'
            tmp += f' Node("{node.name.value}")\n'
            tmp += f'   File "{node.getInstanciationFileName()}", line {node.getInstanciationSourceFilePos()}\n'
            tb = traceback.extract_tb(exc_tb)        
            Sofa.Helper.msg_error(node, tmp, tb[-1].filename, tb[-1].lineno)
            return True

    node.onCtxEnter = enter
    node.onCtxExit = exit
    currentBase = node
    return node    
    
def Object(type, *args, **kwargs):
    global __nodestack__, currentBase
    
    # Register the call point in the node 
    ss=inspect.stack()[1:2]
    try:
        p = __nodestack__.nodes[0]
        nd = piggypatcher_query(p.getInstanciationFileName(), 
                                p.getInstanciationSourceFilePos(), 
                                str(__nodestack__.top().linkpath),
                                 kwargs.get("name", type))
        if len(nd) != 0:
            print("DUMP ", nd, kwargs)
        kwargs = kwargs | nd 
        if len(nd) != 0:
            print("POS DUMP ", kwargs)

        if isinstance(type, Sofa.Core.Object):
            __nodestack__.top().addObject(type)
            object = type     
        elif isinstance(type, str):
            object =  __nodestack__.top().addObject(type, *args, **kwargs)
        else:
            object = __nodestack__.top().addObject(type(*args,**kwargs))
    except Exception:
        exc_type, exc_value, exc_tb = sys.exc_info()
        object =  __nodestack__.top().addObject(InvalidObject(name=f"UncreatableObject({type})"))

        tmp = Sofa.sofaFormatHandler(exc_type, exc_value, exc_tb)
        tmp += "\nSofa Scene Hierarchy (most recent are at the end): \n"
        
        for node in __nodestack__.nodes:
            tmp += f' Node("{node.name.value}")\n'
            tmp += f'   File "{node.getInstanciationFileName()}", line {node.getInstanciationSourceFilePos()}\n'
        
        tmp += f'  {(ss[0][4][0]).strip()}\n'
        tmp += f'   File "{ss[0][1]}", line {ss[0][2]}\n'

        tb = traceback.extract_tb(exc_tb) 
        Sofa.Helper.msg_error(object, tmp, ss[0][1], ss[0][2])
    object.setInstanciationSourceFileName(ss[0][1])
    object.setInstanciationSourceFilePos(ss[0][2])
    currentBase = object
    return object

def SofaPrefab(func):
    rs=inspect.stack()[1:2] 
 
    def wrapper(*args, **kwargs):
        with Node(*args) as node:
            ss=inspect.stack()[1:2] 
            node.setInstanciationSourceFileName(ss[0][1])
            node.setInstanciationSourceFilePos(ss[0][2])
      
            node.setDefinitionSourceFileName(rs[0][1])
            node.setDefinitionSourceFilePos(rs[0][2]+1)
            func(node, **kwargs)
        return node

    return wrapper

def SofaScene(func):
    def wrapper(node, **kwargs):
        global currentNode, depthLevel, currentBase, this_node, __nodestack__
        currentNode = None
        depthLevel = 0
        currentBase = None
        this_node = None
        __nodestack__ = NodeStack()

        __nodestack__.push(node)
        currentBase = node
        currentNode = node
        this_node = currentNode 
        
        this_node.setInstanciationSourceFileName(func.__name__)
        this_node.setInstanciationSourceFilePos(1)
      
        func(node, **kwargs)
    return wrapper
 