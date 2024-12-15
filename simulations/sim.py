import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sympy import symbols, Matrix, sin, cos, pi, Symbol, eye
import sympy as sp
import time


class KinamticsMatrices:
  @staticmethod
  def rot_x(q):
    return np.array([
      [1, 0, 0, 0],
      [0, np.cos(q), -np.sin(q), 0],
      [0, np.sin(q), np.cos(q), 0],
      [0, 0, 0, 1]
    ])
  
  @staticmethod
  def rot_z(q):
    return np.array([
      [np.cos(q), -np.sin(q), 0, 0],
      [np.sin(q), np.cos(q), 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ])

  @staticmethod
  def trans_x(q):
    return np.array([
      [1, 0, 0, q],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ])
  
  @staticmethod
  def trans_z(q):
    return np.array([
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, q],
      [0, 0, 0, 1]
    ])
  
  @staticmethod
  def simplify_trigonometric(q):
    s = sin(q).trigsimp()
    c = cos(q).trigsimp()
    return s, c
  
  @staticmethod
  def Rx(q):
    s,c = KinamticsMatrices.simplify_trigonometric(q)
    a = Matrix([
        [1, 0,  0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1]
    ])
    return a

  @staticmethod
  def Rz(q):
    s,c = KinamticsMatrices.simplify_trigonometric(q)
    return Matrix([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])

  def Ry(q):
    s,c = KinamticsMatrices.simplify_trigonometric(q)
    return Matrix([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]
    ])

  @staticmethod
  def Tx(q):
    return Matrix([
        [1, 0, 0, q],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

  @staticmethod
  def Tz(q):
    return Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, q],
        [0, 0, 0, 1]
    ])

  @staticmethod
  def RollPitchYawToRotationMatrix(roll, pitch, yaw):
    a = KinamticsMatrices.Rz(roll) @ KinamticsMatrices.Ry(pitch) @ KinamticsMatrices.Rx(yaw)
    return a[:3,:3]
  
  @staticmethod
  def EulerZYZToRotationMatrix(alpha, beta, gamma):
    a = KinamticsMatrices.Rz(alpha) @ KinamticsMatrices.Ry(beta) @ KinamticsMatrices.Rz(gamma)
    return a[:3,:3]


class ForwardKinematic6axisModel:
  def __init__(self, link_legnth_1, link_legnth_2, link_legnth_3, link_legnth_4):
    self.link_legnth_1 = link_legnth_1
    self.link_legnth_2 = link_legnth_2
    self.link_legnth_3 = link_legnth_3
    self.link_legnth_4 = link_legnth_4
    self.q1 = Symbol('q1')
    self.q2 = Symbol('q2')
    self.q3 = Symbol('q3')
    self.q4 = Symbol('q4')
    self.q5 = Symbol('q5')
    self.q6 = Symbol('q6')

  def get_not_theretiacl_max_length(self):
    return self.link_legnth_2 + self.link_legnth_3

  def A_0_1(self,q):
    return KinamticsMatrices.rot_z(q)@KinamticsMatrices.trans_z(self.link_legnth_1)@KinamticsMatrices.rot_x(-np.pi/2)

  def A_1_2(self,q):
    return KinamticsMatrices.rot_z(q)@KinamticsMatrices.trans_x(self.link_legnth_2)
  
  def A_2_3(self,q):
    return KinamticsMatrices.rot_z(q-np.pi/2)@KinamticsMatrices.rot_x(-np.pi/2)
  
  def A_3_4(self,q):
    return KinamticsMatrices.rot_z(q)@KinamticsMatrices.trans_z(self.link_legnth_3)@KinamticsMatrices.rot_x(np.pi/2)
  
  def A_4_5(self,q):
    return KinamticsMatrices.rot_z(q)@KinamticsMatrices.rot_x(-np.pi/2)
  
  def A_5_6(self,q):
    return KinamticsMatrices.rot_z(q)@KinamticsMatrices.trans_z(self.link_legnth_4)

  def A_0_6(self,q1,q2,q3,q4,q5,q6):
    return self.A_0_1(q1) @ self.A_1_2(q2) @ self.A_2_3(q3) @ self.A_3_4(q4) @ self.A_4_5(q5) @ self.A_5_6(q6)
  
  def a_0_1(self):
    self.q1 = symbols('q1')
    self.l1 = symbols('l1')
    return KinamticsMatrices.Rz(self.q1)@KinamticsMatrices.Tz(self.l1)@KinamticsMatrices.Rx(-pi/2)
  
  def a_1_2(self):
    self.q2 = symbols('q2')
    self.l2 = symbols('l2')
    return KinamticsMatrices.Rz(self.q2)@KinamticsMatrices.Tx(self.l2)
  
  def a_2_3(self):
    self.q3 = symbols('q3')
    self.rz = self.q3 - pi/2
    return KinamticsMatrices.Rz(self.rz)@KinamticsMatrices.Rx(-pi/2)

  def a_3_4(self):
    self.q4 = symbols('q4')
    self.l3 = symbols('l3')
    return KinamticsMatrices.Rz(self.q4)@KinamticsMatrices.Tz(self.l3)@KinamticsMatrices.Rx(pi/2)

  def a_4_5(self):
    self.q5 = symbols('q5')
    return KinamticsMatrices.Rz(self.q5)@KinamticsMatrices.Rx(-pi/2)
  
  def a_5_6(self):
    self.q6 = symbols('q6')
    self.l4 = symbols('l4')
    return KinamticsMatrices.Rz(self.q6)@KinamticsMatrices.Tz(self.l4)
  
  def a_0_6(self):
    return self.a_0_1() @ self.a_1_2() @ self.a_2_3() @ self.a_3_4() @ self.a_4_5() @ self.a_5_6()

  def get_pos_equasion(self):
    return self.a_0_6()[:3,3]

  def get_rotation_matrix_equasion(self):
    return self.a_0_6()[:3,:3]


  # def get_jakobian(self):
  #   pos = self.get_pos_equasion()
  #   rot = self.get_rotation_matrix_equasion()
  #   q = Matrix([self.q1,self.q2,self.q3,self.q4,self.q5,self.q6])
  #   return pos.jacobian(q), rot.jacobian(q)

class Kinematic6axisVisaulization:
  def __init__(self, model:ForwardKinematic6axisModel,arrow_length=0.4):
    self.model = model
    self.q1 = 0
    self.q2 = 0
    self.q3 = 0
    self.q4 = 0
    self.q5 = 0
    self.q6 = 0
    self.arrow_length = arrow_length


    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(111, projection='3d')
    self.ax.set_xlim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_ylim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_zlim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_xlabel('X')
    self.ax.set_ylabel('Y')
    self.ax.set_zlabel('Z')


  def set_angles(self, q1,q2,q3,q4,q5,q6):
    self.q1 = q1
    self.q2 = q2
    self.q3 = q3
    self.q4 = q4
    self.q5 = q5
    self.q6 = q6

  def get_transformation_matrices(self, q1,q2,q3,q4,q5,q6):
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
    substitutes = {
      symbols('q1'): self.q1,
      symbols('q2'): self.q2,
      symbols('q3'): self.q3,
      symbols('q4'): self.q4,
      symbols('q5'): self.q5,
      symbols('q6'): self.q6,
      symbols('l1'): self.model.link_legnth_1,
      symbols('l2'): self.model.link_legnth_2,
      symbols('l3'): self.model.link_legnth_3,
      symbols('l4'): self.model.link_legnth_4
    }
    a01 = a01.subs(substitutes)
    a02 = a02.subs(substitutes)
    a03 = a03.subs(substitutes)
    a04 = a04.subs(substitutes)
    a05 = a05.subs(substitutes)
    a06 = a06.subs(substitutes)
    values = [a00,a01,a02,a03,a04,a05,a06]

    return values, transformations
  
  def __plot(self,transformations=None):
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    self.ax.clear()

    if transformations is None:
      transformations = self.get_transformation_matrices(self.q1,self.q2,self.q3,self.q4,self.q5,self.q6)
    xs, ys, zs = [0], [0], [0]
    
    for i in range(0,len(transformations)):
      T = transformations[i]
      xs.append(T[0, 3])
      ys.append(T[1, 3])
      zs.append(T[2, 3])
      # Plot the XYZ arrows
      self._plot_arrows(self.ax, T,i)

    self.ax.plot(xs, ys, zs, marker='o')
    self.ax.set_xlim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_ylim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_zlim([-self.model.get_not_theretiacl_max_length(), self.model.get_not_theretiacl_max_length()])
    self.ax.set_xlabel('X')
    self.ax.set_ylabel('Y')
    self.ax.set_zlabel('Z')
    # plt.show()

  def plot(self,transformations=None):  
    self.ax.clear()
    self.__plot(transformations)
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
      self.set_angles(*joint_set)
      self.__plot()
      plt.pause(frame_rate)

  def play_plot(self, joints_values, play_in_loop=False, frame_rate=0.5):
    while True:
      self.__play_plot(joints_values,frame_rate)
      if not play_in_loop:
        break
    
    if not play_in_loop:
      plt.show()

    
    


if __name__=='__main__':
  # Example usage
  model = ForwardKinematic6axisModel(145, 375, 375, 150)

  pos =  model.get_pos_equasion()
  rot = model.get_rotation_matrix_equasion()
  print(pos)
  print(rot)

  visualizer = Kinematic6axisVisaulization(model,arrow_length=40)
  h = np.pi/2
  # visualizer.set_angles(0,-h -0.3 ,h , h ,h,0)
  # visualizer.plot()
  positions = []
  for a in range(0,20):
    positions.append((0, 0, a*np.pi/20, a*np.pi/20, h, 0),)
  positions += positions[::-1]
      
  visualizer.play_plot(positions,play_in_loop=True,frame_rate=0.1)


  # simplified_equation = simplify(equasion)
  # print(simplified_equation)

  # # visualizer = Kinematic6axisVisaulization(model,arrow_length=40)
  # simplified_rational_equation = radsimp(simplified_equation)
  # print(simplified_rational_equation)
  # Substitute constants like sin(pi/2) with their numerical values

  # visaul = Kinematic6axisVisaulization(model,arrow_length=40)
  # visaul.set_angles(0,-h -0.3 ,h , h ,h,0)
  # transforms, symbolic = visaul.get_symbolic_transformation_matrices()

  # visaul.plot(transforms)




  # print(a01)
  # print(a02)
  # print(a03)
  # print(a04)
  # print(a05)
  # print(a06)


  # equasion = model.a_0_1() @ model.a_1_2() @ model.a_2_3() @ model.a_3_4() @ model.a_4_5() @ model.a_5_6()



  # print(equasion)
  # latex_equation = latex(equasion)
  # print(latex_equation)


  # a = symbols('a')
  # b = symbols('b')
  # c = symbols('c')

  # equ =  Matrix([ [ cos(a) * sin(b)  * sin(c) * sin(-pi/2), 1], [0, 0]]) 

  # simplified_equasion= equ.subs(sin(pi/2), 1).subs(cos(pi/2), 0).subs(sin(-pi/2), -1).subs(cos(-pi/2), 0)
  # print(equ)
  # print(simplified_equasion)

