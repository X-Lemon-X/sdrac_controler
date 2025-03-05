import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sympy import symbols,eye
from sim import Kinematic6axisModel, KinamticsMatrices


class Kinematic6axisVisaulization:
  def __init__(self, model:Kinematic6axisModel,arrow_length=0.4):
    self.model = model
    self.arrow_length = arrow_length

    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(111, projection='3d')
    self.ax.set_xlim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_ylim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_zlim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_xlabel('X')
    self.ax.set_ylabel('Y')
    self.ax.set_zlabel('Z')

  def get_transformation_matrices(self):
    q1,q2,q3,q4,q5,q6 = self.model.get_angles()
    combined = np.eye(4)
    transformations = [combined]

    combined = combined @ self.model.A_0_1(q1)
    transformations.append(combined)

    combined = combined @ self.model.A_1_2(q2)
    transformations.append(combined)

    combined = combined @ self.model.A_2_3(q3)
    transformations.append(combined)

    combined = combined @ self.model.A_3_4(q4)
    transformations.append(combined)

    combined = combined @ self.model.A_4_5(q5)
    transformations.append(combined)

    combined = combined @ self.model.A_5_6(q6)
    transformations.append(combined)

    return transformations
  
  def get_symbolic_transformation_matrices(self):
    combined = eye(4,4)
    transformations = [combined]

    combined = combined @ self.model.a_0_1()
    transformations.append(combined)

    combined = combined @ self.model.a_1_2()
    transformations.append(combined)

    combined = combined @ self.model.a_2_3()
    transformations.append(combined)

    combined = combined @ self.model.a_3_4()
    transformations.append(combined)

    combined = combined @ self.model.a_4_5()
    transformations.append(combined)

    combined = combined @ self.model.a_5_6()
    transformations.append(combined)

    a00 = transformations[0]
    a01 = transformations[1]
    a02 = transformations[2]
    a03 = transformations[3]
    a04 = transformations[4]
    a05 = transformations[5]
    a06 = transformations[6]
    q1,q2,q3,q4,q5,q6 = self.model.get_angles()
    l1,l2,l3,l4 = self.model.get_link_lengths()
    substitutes = {
      symbols('q1'): q1,
      symbols('q2'): q2,
      symbols('q3'): q3,
      symbols('q4'): q4,
      symbols('q5'): q5,
      symbols('q6'): q6,
      symbols('l1'): l1,
      symbols('l2'): l2,
      symbols('l3'): l3,
      symbols('l4'): l4
    }
    a01 = a01.subs(substitutes)
    a02 = a02.subs(substitutes)
    a03 = a03.subs(substitutes)
    a04 = a04.subs(substitutes)
    a05 = a05.subs(substitutes)
    a06 = a06.subs(substitutes)
    values = [a00,a01,a02,a03,a04,a05,a06]

    return values, transformations
  
  def __plot(self,transformations=None, limits_const=False):
    self.ax.clear()

    if transformations is None:
      transformations, _ = self.get_symbolic_transformation_matrices()
    xs, ys, zs = [0], [0], [0]
    
    for i in range(0,len(transformations)):
      T = transformations[i]
      xs.append(T[0, 3])
      ys.append(T[1, 3])
      zs.append(T[2, 3])
      # Plot the XYZ arrows
      self._plot_arrows(self.ax, T,i)

    self.ax.plot(xs, ys, zs, marker='o')

    if not limits_const:
      x,y,z = self.model.get_cordinates()
      x = float(x)
      y = float(y)
      z = float(z)
      xmin = min([x,-x*0.4,self.arrow_length,-self.arrow_length])
      ymin = min([y,-y*0.4,self.arrow_length,-self.arrow_length])
      zmin = min([z,-z*0.4,self.arrow_length,-self.arrow_length])
      xmax = max([x,-x*0.4,self.arrow_length,-self.arrow_length])
      ymax = max([y,-y*0.4,self.arrow_length,-self.arrow_length])
      zmax = max([z,-z*0.4,self.arrow_length,-self.arrow_length])
    else:
      xmax = self.model.get_not_theretiacl_max_length()
      ymax = xmax
      zmax = xmax
      xmin = -xmax
      ymin = -xmax
      zmin = -xmax
      
    self.ax.set_xlim([xmin, xmax])
    self.ax.set_ylim([ymin, ymax])
    self.ax.set_zlim([zmin, zmax])
    self.ax.set_xlabel('X')
    self.ax.set_ylabel('Y')
    self.ax.set_zlabel('Z')

  def plot(self,transformations=None):  
    self.ax.clear()
    self.__plot(transformations,limits_const=False)
    plt.show()

  def _plot_arrows(self, ax, T,num=0):
    origin = T[:3, 3]
    x_dir = T[:3, 0]
    y_dir = T[:3, 1]
    z_dir = T[:3, 2]
    ax.quiver(*origin, *x_dir, color='r', length=self.arrow_length, normalize=True)
    ax.quiver(*origin, *y_dir, color='g', length=self.arrow_length, normalize=True)
    ax.quiver(*origin, *z_dir, color='b', length=self.arrow_length, normalize=True)
    ax.text(*(origin + x_dir * self.arrow_length), f'X{num}', color='r')
    ax.text(*(origin + y_dir * self.arrow_length), f'Y{num}', color='g')
    ax.text(*(origin + z_dir * self.arrow_length), f'Z{num}', color='b')

  def __play_plot(self, joints_values,frame_rate=0.5):
    for joint_set in joints_values:
      self.model.set_angles(*joint_set)
      self.__plot(None,limits_const=True)
      plt.pause(frame_rate)

  def play_plot(self, joints_values, play_in_loop=False, frame_rate=0.5):
    # matplotlib.use('qt5agg')   
    while True:
      self.__play_plot(joints_values,frame_rate)
      if not play_in_loop:
        break
    
    if not play_in_loop:
      plt.show()

    
    


if __name__=='__main__':
  # Example usage
  model = Kinematic6axisModel(145, 375, 375, 150)

  pos =  model.get_pos_equasion()
  rot = model.get_rotation_matrix_equasion()
  # sol = model.get_representation_angles()
  print(pos)
  print(rot)
  # print(sol)

  visualizer = Kinematic6axisVisaulization(model,arrow_length=150)
  h = np.pi/2
  # visualizer.set_angles(0,-h -0.3 ,h , h ,h,0)
  # visualizer.plot()
  positions = []
  for a in range(0,20):
    positions.append((0, 0, a*np.pi/20, a*np.pi/20, h, 0),)
  positions += positions[::-1]
      
  visualizer.play_plot(positions,play_in_loop=True,frame_rate=0.1)
  
  # model.set_angles(0,0,0,h,h,0)
  # visualizer.plot()
