import meshcat
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer as PMV

def rgb2int(r, g, b):
    '''
    Convert 3 integers (chars) 0 <= r, g, b < 256 into one single integer = 256**2*r+256*g+b, as expected by Meshcat.
    '''
    return int((r << 16) + (g << 8) + b)


def material(color, transparent=False):
    mat = meshcat.geometry.MeshPhongMaterial()
    mat.color = color
    mat.transparent = transparent
    return mat

colormap = {
    'red': material(color=rgb2int(255, 0, 0), transparent=False),
    'blue': material(color=rgb2int(0, 0, 255), transparent=False),
    'green': material(color=rgb2int(0, 255, 0), transparent=False),
    'yellow': material(color=rgb2int(255, 255, 0), transparent=False),
    'magenta': material(color=rgb2int(255, 0, 255), transparent=False),
    'cyan': material(color=rgb2int(0, 255, 255), transparent=False),
    'black': material(color=rgb2int(250, 250, 250), transparent=False),
    'white': material(color=rgb2int(5, 5, 5), transparent=False),
    'grey': material(color=rgb2int(120, 120, 120), transparent=False)
}

def materialFromColor(color):
    if isinstance(color, meshcat.geometry.MeshPhongMaterial):
        return color
    elif isinstance(color, str):
        material = colormap[color]
    elif isinstance(color, list):
        material = meshcat.geometry.MeshPhongMaterial()
        material.color = rgb2int(*[int(c * 255) for c in color[:3]])
        if len(color) == 3:
            material.transparent = False
        else:
            material.transparent = color[3] < 1
            material.opacity = float(color[3])
    else:
        material = colormap['black']
    return material

class MeshcatVisualizer(PMV):
    def __init__(self, robot=None, model=None, collision_model=None, visual_model=None, url=None):
        if robot is not None:
            super().__init__(robot.model, robot.collision_model, robot.visual_model)
        elif model is not None:
            super().__init__(model, collision_model, visual_model)

        if url is not None:
            if url == 'classical':
                url = 'tcp://127.0.0.1:6000'
            print('Wrapper tries to connect to server <%s>' % url)
            server = meshcat.Visualizer(zmq_url=url)
        else:
            server = None

        if robot is not None or model is not None:
            self.initViewer(loadModel=True, viewer=server)
        else:
            self.viewer = server if server is not None else meshcat.Visualizer()

    def addSphere(self, name, radius, color):
        material = materialFromColor(color)
        self.viewer[name].set_object(meshcat.geometry.Sphere(radius), material)

    def visualize_frame(self, name, tform, line_length=0.2, line_width=3):
        """
        Visualizes a coordinate frame as an axis triad at a specified pose.

        Parameters
            name : str
                The name of the MeshCat component to add.
            tform : `pinocchio.SE3`
                The transform at which to display the frame.
            line_length : float, optional
                The length of the axes in the triad.
            line_width : float, optional
                The width of the axes in the triad.
        """
        frame_axis = (np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]])
                        .astype(np.float32)
                        .T)

        color = (np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]])
                   .astype(np.float32)
                   .T)

        self.viewer[name].set_object(
            meshcat.geometry.LineSegments(
                meshcat.geometry.PointsGeometry(
                    position=line_length * frame_axis,
                    color=color,
                ),
                meshcat.geometry.LineBasicMaterial(
                    linewidth=line_width,
                    vertexColors=True,
                ),
            )
        )
        self.viewer[name].set_transform(tform.homogeneous)

    def visualize_axis(self, name, axis_point, axis_direction, distance_along_axis, line_width=3, color=[0, 0, 1]):
        """
        Visualizes a screw axis as a line in the 3D viewer.

        Parameters:
            name : str
                The name of the MeshCat component to add.
            axis_point : np.array
                A point on the axis (3x1 vector).
            axis_direction : np.array
                The direction vector of the axis (3x1 vector).
            length : float, optional
                The length of the axis line to be visualized.
            line_width : int, optional
                The width of the axis line.
            color : list, optional
                The color of the axis as an RGB list.
        """
        start_point = axis_point
        end_point = axis_point + distance_along_axis * axis_direction
        points = np.vstack([start_point, end_point]).T
        colors = np.tile(np.array(color, dtype=np.float32).reshape(3, 1), (1, 2))

        self.viewer[name].set_object(
            meshcat.geometry.LineSegments(
                meshcat.geometry.PointsGeometry(position=points, color=colors),
                meshcat.geometry.LineBasicMaterial(linewidth=line_width, vertexColors=True)
            )
        )

    def applyConfiguration(self, name, placement):
        if isinstance(placement, list) or isinstance(placement, tuple):
            placement = np.array(placement)
        if isinstance(placement, pin.SE3):
            R, p = placement.rotation, placement.translation
            T = np.r_[np.c_[R, p], [[0, 0, 0, 1]]]
        elif isinstance(placement, np.ndarray):
            if placement.shape == (7, ):  # XYZ-quat
                R = pin.Quaternion(np.reshape(placement[3:], [4, 1])).matrix()
                p = placement[:3]
                T = np.r_[np.c_[R, p], [[0, 0, 0, 1]]]
            else:
                print('Error, np.shape of placement is not accepted')
                return False
        else:
            print('Error format of placement is not accepted')
            return False
        self.viewer[name].set_transform(T)

    def delete(self, name):
        self.viewer[name].delete()

    def __getitem__(self, name):
        return self.viewer[name]