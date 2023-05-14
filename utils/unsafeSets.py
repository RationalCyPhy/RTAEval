import abc
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle, Circle, Polygon
from matplotlib.collections import PatchCollection
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import polytope as pc
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

from polytope import extreme, cheby_ball, bounding_box, is_fulldim

class baseUnsafeSet(abc.ABC):
    @abc.abstractmethod
    def update_def(self, simulatorState):
        pass

    @abc.abstractmethod
    def plot_set(self, ax, plotType):
        pass

# The four types of unsafe sets we provide to the user: point, ball, rectangle, polytope   
# These define the shared unsafe sets amongst all agents defined in the workspace
# Thus, the workspace type can be 1D, 2D, or 3D
# These can also be static or relative (except polytope)

class staticUnsafePoint(baseUnsafeSet):
    def __init__(self, id, x0) -> None:
        self.id = id
        self.center = x0
        self.set_type = 'point'
        self.num_dims = len(x0)
        if self.num_dims not in [1, 2, 3]:
            raise Exception('Number of dims must be 1, 2, or 3!')

    def update_def(self, simulatorState):
        return self.center

    def plot_set(self, ax, plotType):
        x_center = self.center[0]
        y_center = 0
        z_center = 0
        if self.num_dims > 1:
            y_center = self.center[1]
            if self.num_dims == 3:
                z_center = self.center[2]
                
        if plotType == "2D":
            ax.plot([x_center], [y_center], marker = "o", color = "red")
        elif plotType == "3D":
            ax.plot([x_center], [y_center], [z_center], marker = "o", color = "red")

class staticUnsafeBall(baseUnsafeSet):
    def __init__(self, id, x0, r) -> None:
        self.id = id
        self.center = x0
        self.radius = r
        self.set_type = 'ball'
        self.num_dims = len(x0)

    def update_def(self, simulatorState):
        return (self.center, self.radius)

    def plot_set(self, ax, plotType):
        x_center = self.center[0]
        y_center = 0
        z_center = 0
        if self.num_dims > 1:
            y_center = self.center[1]
            if self.num_dims == 3:
                z_center = self.center[2]
                 
        if plotType == "2D":
            ax.add_patch(Circle((x_center, y_center), self.radius, color = "red", alpha = 0.2))
            ax.plot([x_center], [y_center], marker = "o", color = "red")
        elif plotType == "3D":
            u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
            x = self.radius*(np.cos(u) * np.sin(v)) + x_center
            y = self.radius*(np.sin(u) * np.sin(v)) + y_center
            z = self.radius*(np.cos(v))  + z_center
            ax.plot_surface(x, y, z, color='red', alpha = 0.2)

class staticUnsafeRectangle(baseUnsafeSet):
    def __init__(self, id, x0, dx=0, dy=0, dz=0) -> None:
        self.id = id
        self.center = x0
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.set_type = 'rectangle'
        self.num_dims = len(x0)
    
    def update_def(self, simulatorState):
        return (self.center, self.dx, self.dy, self.dz)

    def plot_set(self, ax, plotType):
        x_center = self.center[0]
        y_center = 0
        z_center = 0
        if self.num_dims > 1:
            y_center = self.center[1]
            if self.num_dims == 3:
                z_center = self.center[2]
                
        if plotType == "2D":
            ax.add_patch(Rectangle((x_center-self.dx/2, y_center-self.dx/2), self.dx, self.dy, color = "red"))
            ax.plot([x_center], [y_center], marker = "o", color = "red")
        elif plotType == "3D":
            corners = np.array([[x_center - self.dx/2, y_center + self.dy/2, z_center + self.dz/2],
                       [x_center + self.dx/2, y_center + self.dy/2, z_center + self.dz/2],
                       [x_center - self.dx/2, y_center + self.dy/2, z_center - self.dz/2],
                       [x_center + self.dx/2, y_center + self.dy/2, z_center - self.dz/2],
                       [x_center - self.dx/2, y_center - self.dy/2, z_center + self.dz/2],
                       [x_center + self.dx/2, y_center - self.dy/2, z_center + self.dz/2],
                       [x_center - self.dx/2, y_center - self.dy/2, z_center - self.dz/2],
                       [x_center + self.dx/2, y_center - self.dy/2, z_center - self.dz/2]])

            hull = ConvexHull(corners)
            # draw the polygons of the convex hull
            for s in hull.simplices:
                tri = Poly3DCollection(corners[s])
                tri.set_color('red')
                tri.set_alpha(0.1)
                ax.add_collection3d(tri)

class staticUnsafePolytope(baseUnsafeSet):
    def __init__(self, id, A, b) -> None:
        self.id = id
        self.set_type = "polytope"
        self.num_dims = len(A[0])
        self.num_faces = len(b)
        self.A = np.array(A)
        self.b = np.array(b)

    def update_def(self, simulatorState):
        return (self.A, self.b)

    def plot_poly_3d(self, cubes, ax, color='r'):
        for cube in cubes:
            hull = ConvexHull(cube)
            # draw the polygons of the convex hull
            for s in hull.simplices:
                tri = Poly3DCollection(cube[s])
                tri.set_color(color)
                tri.set_alpha(0.1)
                ax.add_collection3d(tri)

    def get_patch(self, poly1, color="red"):
        # From tulip polytope source code: https://tulip-control.sourceforge.io/doc/_modules/polytope/plot.html
        V = extreme(poly1)
        rc,xc = cheby_ball(poly1)
        x = V[:,1] - xc[1]
        y = V[:,0] - xc[0]
        mult = np.sqrt(x**2 + y**2)
        x = x/mult
        angle = np.arccos(x)
        corr = np.ones(y.size) - 2*(y < 0)
        angle = angle*corr
        ind = np.argsort(angle) 

        patch = Polygon(V[ind,:], True, color=color)
        return patch
        
    def plot_poly_2d(self, poly1, ax, show=False):
        # From tulip polytope source code: https://tulip-control.sourceforge.io/doc/_modules/polytope/plot.html
        if is_fulldim(poly1):
            if self.num_dims != 2:
                print("Can not plot polytopes of dimension larger than 2")
                return
            
            if len(poly1) == 0:
        
                poly = self.get_patch(poly1)
                l,u = bounding_box(poly1)
                ax.add_patch(poly)        
            
                ax.set_xlim(l[0,0],u[0,0])
                ax.set_ylim(l[1,0],u[1,0])
            
            else:
                l,u = bounding_box(poly1)
            
                fig = plt.figure()
                ax = fig.add_subplot(111)
                    
                for poly2 in poly1.list_poly:
                    poly = self.get_patch(poly2, color=np.random.rand(3))
                    ax.add_patch(poly)
            
                ax.set_xlim(l[0,0],u[0,0])
                ax.set_ylim(l[1,0],u[1,0])
        else:
            print("Cannot plot empty polytope")

    def plot_set(self, ax, plotType):                
        if plotType == "2D":
            poly = pc.Polytope(self.A, self.b)
            self.plot_poly_2d(poly, ax)
        elif plotType == "3D":
            poly = pc.Polytope(self.A, self.b)
            corners = pc.extreme(poly)
            self.plot_poly_3d([pc.extreme(poly)], ax)

class relativeUnsafePoint(staticUnsafePoint):
    def __init__(self, id, x0, refAgentid) -> None:
        super().__init__(id, x0)
        self.refAgentid = refAgentid

    def update_def(self, simulatorState):
        center_state = simulatorState['agents'][self.refAgentid]['state_trace'][-1][1:]
        center = center_state[0:self.num_dims]
        self.center = center
        return self.center

class relativeUnsafeBall(staticUnsafeBall):
    def __init__(self, id, x0, r, refAgentid) -> None:
        super().__init__(id, x0, r)
        self.refAgentid = refAgentid
        

    def update_def(self, simulatorState):
        center_state = simulatorState['agents'][self.refAgentid]['state_trace'][-1][1:]
        center = center_state[0:self.num_dims]
        self.center = center
        return (self.center, self.radius)

class relativeUnsafeRectangle(staticUnsafeRectangle):
    def __init__(self, id, x0, refAgentid, dx=0, dy=0, dz=0) -> None:
        super().__init__(id, x0, dx, dy, dz)
        self.refAgentid = refAgentid
    
    def update_def(self, simulatorState):
        center_state = simulatorState['other'][self.refAgentid]['state_trace'][-1][1:]
        center = center_state[0:self.num_dims]
        self.center = center
        return (self.center, self.dx, self.dy, self.dz)

if __name__ == "__main__":
    print('testing')

    A = [[-1,0,0],[1,0,0],[0,-1,0],[0,1,0],[0,0,-1],[0,0,1]]
    b = [1,1,1,1,1,1]
    myPoly = staticUnsafePolytope(A,b)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    myPoly.plot_set(ax, "3D")

    # A = [[-1,0],[1,0],[0,-1],[0,1]]
    # b = [1,1,1,1]
    # myPoly2 = staticUnsafePolytope(A,b)
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # myPoly2.plot_set(ax, "2D")
    # plt.show()    

    # refAgentid = 'other1'
    # x0 = [0, 100,100]
    # testSimState = {'other':{'other1':{'state_trace':[[0, 100,100,0,0,0,100]]}}, 'ego':{'ego1':{'state_trace':[[0, -150,-150,0,0,0,100]]}, 'ego2':{'state_trace':[[0, -300,-300,0,0,0,100]]}, 'ego3':{'state_trace':[[0, -150,150,0,0,0,100]]}}}

    # unsafeState = relativeUnsafePoint('unsafe1', x0=x0, refAgentid=refAgentid)
    # print(unsafeState.update_def(testSimState))