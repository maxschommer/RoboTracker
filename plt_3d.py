import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

from matplotlib import pyplot as plt
import random
import cv2

class Tag():
    def __init__(self, tag_id, origin=[0,0,0], x_axis=[1,0,0], y_axis=[0,1,0]):
        self.tag_id = tag_id
        self.csys = CSys(origin=origin, x_axis=x_axis, y_axis=y_axis)

    def set_with_pose(self, camera_params, tag_size, pose, z_sign=1):
        self.csys.set_with_pose(camera_params, tag_size, pose, z_sign)
    
    def get_artists(self, arrow_len=1):
        return self.csys.get_artists(arrow_len)

class CSys():
    def __init__(self, origin=[0,0,0], x_axis=[1,0,0], y_axis=[0,1,0]):
        self.origin = np.asarray(origin)
        self.x_axis = np.asarray(x_axis)
        self.y_axis = np.asarray(y_axis)
        self.z_axis = np.cross(x_axis, y_axis)

    def set_rotation(self, R):
        self.x_axis = np.matmul(R, np.asarray([1,0,0]))
        self.y_axis = np.matmul(R, np.asarray([0,1,0]))
        self.z_axis = np.matmul(R, np.asarray([0,0,1]))

    def rotate(self, R):
        self.x_axis = np.matmul(R, self.x_axis)
        self.y_axis = np.matmul(R, self.y_axis)
        self.z_axis = np.matmul(R, self.z_axis)

    def get_rotation_matrix(self):
        res = np.transpose(np.asarray([self.x_axis, self.y_axis, self.z_axis]))
        return res
 
    def set_with_pose(self, camera_params, tag_size, pose, z_sign=1):
        fx, fy, cx, cy = camera_params

        K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

        rvec, _ = cv2.Rodrigues(pose[:3,:3])
        tvec = pose[:3, 3]
        # tvec[2] = z_sign*tvec[2]
        self.origin = tvec
        self.set_rotation(np.asarray(pose[:3, :3]))
        # self.x_axis[2] = z_sign*self.x_axis[2]
        # self.y_axis[2] = z_sign*self.y_axis[2]
        # self.z_axis[2] = z_sign*self.z_axis[2]

    def get_artists(self, arrow_len=1):
        res = []
        for axis in [self.x_axis, self.y_axis, self.z_axis]:
            a = Arrow3D([self.origin[0], self.origin[0]+axis[0]*arrow_len], [self.origin[1], self.origin[1]+axis[1]*arrow_len], 
                    [self.origin[2], self.origin[2]+axis[2]*arrow_len], mutation_scale=20, 
                    lw=3, arrowstyle="-|>", color="r")
            res.append(a)
        return res
        


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

class LivePlotter():
    def __init__(self, limits=[-1,1,-1,1,-1,1]):
        self.fig = plt.figure()
        self.fig_ax = Axes3D(self.fig)
        self.fig_ax.autoscale(enable=True, axis='both', tight=True)

        # # # Setting the axes properties
        self.fig_ax.set_xlim3d(limits[0:2])
        self.fig_ax.set_ylim3d(limits[2:4])
        self.fig_ax.set_zlim3d(limits[4:6])

        self.all_artists = []
        # self.hl, = self.fig_ax.plot3D([0], [0], [0], 'o')
        # self.csys = 

    def add_artists(self, new_artists = []):
        self.all_artists = self.all_artists + new_artists

    def clear_artists(self):
        for artist in self.all_artists:
            try:
                artist.remove()
            except Exception as e:
                pass            
        self.all_artists = []

    def remove_artists(self):
        for artist in self.all_artists:
            artist.remove()

    def update_plt(self):
        for artist in self.all_artists:
            self.fig_ax.add_artist(artist)
        plt.show(block=False)
        plt.pause(.01)
        self.remove_artists()
        # xdata, ydata, zdata = artist._verts3d
        # artist.set_xdata(new_data[0])
        # artist.set_ydata(new_data[1])
        # artist.set_3d_properties(new_data[2])
        

    def run(self):
        # pt_artist, = self.fig_ax.plot3D([random.random()], [random.random()], [random.random()], 'ro')
        while True:
            # print(random.random())
            for artist in self.all_artists:
                self.fig_ax.add_artist(artist)
            # self.update_plt(pt_artist, [random.random(), random.random(), random.random()])
            plt.show(block=False)
            plt.pause(.1)
            for artist in self.all_artists:
                artist.remove()
            # pt_artist.remove()



    def plt_pose(self, camera_params, tag_size, pose, z_sign=1):
        new_csys = CSys()
        new_csys.set_with_pose(camera_params, tag_size, pose, z_sign=1)
        new_artists = new_csys.get_artists(arrow_len=2)
        self.all_artists = new_artists


# map = plt.figure()
# map_ax = Axes3D(map)
# map_ax.autoscale(enable=True, axis='both', tight=True)
 
# # # # Setting the axes properties
# map_ax.set_xlim3d([0.0, 10.0])
# map_ax.set_ylim3d([0.0, 10.0])
# map_ax.set_zlim3d([0.0, 10.0])
 
# hl, = map_ax.plot3D([0], [0], [0])
 
# update_line(hl, (2,2, 1))
# plt.show(block=False)
# plt.pause(1)
 
# update_line(hl, (5,5, 5))
# plt.show(block=False)
# plt.pause(2)
 
# update_line(hl, (8,1, 4))
# plt.show(block=True)

def main():
    acsys = CSys()
    R_90_z = np.asarray([[0, -1, 0],[1,0,0],[0,0,1]])
    acsys.rotate(R_90_z)
    print(R_90_z)
    print(acsys.get_rotation_matrix())

if __name__ == '__main__':
    main()
