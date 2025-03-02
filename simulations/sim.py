import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib
import copy

from sympy import symbols, Matrix, sin, cos, pi, Symbol, eye, acos ,sqrt, asin
import sympy
import time

def eqMatrix(a, b, epsilon=1e-5):
  return np.all(np.abs(a-b) < epsilon)

def eq(a, b, epsilon=1e-5):
  return abs(a-b) < epsilon

def beautify(a):
  if abs(a) < 1e-7:
    return 0
  return a
 
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
  def rot_y(q):
    return np.array([
      [np.cos(q), 0, np.sin(q), 0],
      [0, 1, 0, 0],
      [-np.sin(q), 0, np.cos(q), 0],
      [0,0,0,1]
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
  def Ty(q):
    return Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, q],
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
  def RollPitchYawToRotationMatrix(roll, pitch, yaw)-> Matrix:
    a = KinamticsMatrices.Rz(roll) @ KinamticsMatrices.Ry(pitch) @ KinamticsMatrices.Rx(yaw)
    return a[:3,:3]
  
  @staticmethod
  def EulerZYZToMatrix(alpha, beta, gamma) -> Matrix:
    a = KinamticsMatrices.Rz(alpha) @ KinamticsMatrices.Ry(beta) @ KinamticsMatrices.Rz(gamma)
    return a
  
  @staticmethod
  def EulerZYZToRotationMatrix(alpha, beta, gamma)-> Matrix:
    # a = KinamticsMatrices.Rz(alpha) @ KinamticsMatrices.Ry(beta) @ KinamticsMatrices.Rz(gamma)
    return KinamticsMatrices.EulerZYZToMatrix(alpha, beta, gamma)[:3,:3]


class Kinematic6axisModel:
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
    self.l1 = Symbol('l1')
    self.l2 = Symbol('l2')
    self.l3 = Symbol('l3')
    self.l4 = Symbol('l4')
    self.full_a06 = self.a_0_6()

    self.q1_rad = 0
    self.q2_rad = 0
    self.q3_rad = 0
    self.q4_rad = 0
    self.q5_rad = 0
    self.q6_rad = 0
    
    self.transfotmation_current_a06 = None
    self.pos_x = 900
    self.pos_y = 0
    self.pos_z = 145
    self.rot_roll = 0
    self.rot_pitch = 0
    self.rot_yaw = 0
    self.tcp_x = 0
    self.tcp_y = 0
    self.tcp_z = 0

    self.set_angles(0,0,0,0,0,0)

  def set_angles(self, q1,q2,q3,q4,q5,q6):
    self.q1_rad = q1
    self.q2_rad = q2
    self.q3_rad = q3
    self.q4_rad = q4
    self.q5_rad = q5
    self.q6_rad = q6
    self.transfotmation_current_a06 = copy.copy(self.full_a06)
    self.transfotmation_current_a06 = self.transfotmation_current_a06.subs({
      self.q1: q1,
      self.q2: q2,
      self.q3: q3,
      self.q4: q4,
      self.q5: q5,
      self.q6: q6,
      self.l1: self.link_legnth_1,
      self.l2: self.link_legnth_2,
      self.l3: self.link_legnth_3,
      self.l4: self.link_legnth_4
    })
  
  def set_tcp_vector(self, x,y,z):
    self.tcp_x = x
    self.tcp_y = y
    self.tcp_z = z

  def set_pos_and_rot(self, x,y,z, roll, pitch, yaw):
    self.pos_x = x
    self.pos_y = y
    self.pos_z = z
    self.rot_roll = roll
    self.rot_pitch = pitch
    self.rot_yaw = yaw

  def get_angles(self):
    return self.q1_rad, self.q2_rad, self.q3_rad, self.q4_rad, self.q5_rad, self.q6_rad

  def get_link_lengths(self):
    return self.link_legnth_1, self.link_legnth_2, self.link_legnth_3, self.link_legnth_4

  def get_transformation_matrix(self):
    return copy.copy(self.transfotmation_current_a06)

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
    # self.q1 = symbols('q1')
    # self.l1 = symbols('l1')
    return KinamticsMatrices.Rz(self.q1)@KinamticsMatrices.Tz(self.l1)@KinamticsMatrices.Rx(-pi/2)
  
  def a_1_2(self):
    # self.q2 = symbols('q2')
    # self.l2 = symbols('l2')
    return KinamticsMatrices.Rz(self.q2)@KinamticsMatrices.Tx(self.l2)
  
  def a_2_3(self):
    # self.q3 = symbols('q3')
    self.rz = self.q3 - pi/2
    return KinamticsMatrices.Rz(self.rz)@KinamticsMatrices.Rx(-pi/2)

  def a_3_4(self):
    # self.q4 = symbols('q4')
    # self.l3 = symbols('l3')
    return KinamticsMatrices.Rz(self.q4)@KinamticsMatrices.Tz(self.l3)@KinamticsMatrices.Rx(pi/2)

  def a_4_5(self):
    # self.q5 = symbols('q5')
    return KinamticsMatrices.Rz(self.q5)@KinamticsMatrices.Rx(-pi/2)
  
  def a_5_6(self):
    # self.q6 = symbols('q6')
    # self.l4 = symbols('l4')
    return KinamticsMatrices.Rz(self.q6)@KinamticsMatrices.Tz(self.l4)
  
  def a_0_6(self):
    return self.a_0_1() @ self.a_1_2() @ self.a_2_3() @ self.a_3_4() @ self.a_4_5() @ self.a_5_6()

  def get_pos_equasion(self):
    return self.a_0_6()[:3,3]

  def get_rotation_matrix_equasion(self):
    return self.a_0_6()[:3,:3]
  
  def get_defined_rotation_matrix(self):
    q1 = self.q1
    q2 = self.q2
    q3 = self.q3
    q4 = self.q4
    q5 = self.q5
    q6 = self.q6
    m = Matrix(
      [
        [
          ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), 
         -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), 
         -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3)
        ], 
        [
          ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), 
          -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), 
          -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3)
        ],
        [
          (-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), 
          -(-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), 
          -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)
        ]
      ]
      )
    return m
  

  def __euler_check(self,a,p,y, target_rot):
    eu = KinamticsMatrices.EulerZYZToRotationMatrix(a,p,y)
    if eqMatrix(eu, target_rot):
      return (a,p,y)
    return None

  def get_euler_zyz_angles_solutions(self):
    # Euler Z(a)Y(p)Z(y) angles a , p , y
    rot = self.transfotmation_current_a06[:3,:3]
    A = rot[0,0]
    B = rot[0,1]
    C = rot[0,2]
    D = rot[1,0]
    E = rot[1,1]
    F = rot[1,2]
    G = rot[2,0]
    H = rot[2,1]
    J = rot[2,2]

    #  first solution for P angle
    p1 = acos(J)
    sp1 = sin(p1)
    y12 = []
    a12 = []
    if sp1 <= 1e-5: # for p = 0 or p = pi |  Case for p = 0
      a1 = 0 # we set a to 0
      a2 = 0 #
      y1 = acos(A).evalf()
      y2 = 2*np.pi - y1
      # check y1 y2
      d_11 = sin(y1)
      d_12 = sin(y2)
      if eq(d_11, D):
        y12.append(y1)
        a12.append(a1)
      if eq(d_12, D):
        y12.append(y2)
        a12.append(a2)

    else:
      a1 = asin(F/sp1).evalf()
      a2 = np.pi - a1
      y1 = asin(H/sp1).evalf()
      y2 = np.pi - y1

      # check y1 y2
      g_11 = -sp1*cos(y1).evalf()
      g_12 = -sp1*cos(y2).evalf()

      if eq(g_11, G):
        y12.append(y1)
      if eq(g_12, G):
        y12.append(y2)

      # check a1 a2
      c_11 = sp1*cos(a1).evalf()
      c_12 = sp1*cos(a2).evalf()
      if eq(c_11, C):
        a12.append(a1)
      if eq(c_12, C):
        a12.append(a2)

    #  second solution for P angle
    p2 = 2 * np.pi - acos(J).evalf()
    sp2 = sin(p2).evalf()
    y34 = []
    a34 = []
    if sp2 <= 1e-5: # for p = 0 or p = pi |  Case for p = pi
      a3 = 0 # we set a to 0
      a4 = 0 #
      y3 = asin(D).evalf()
      y4 = np.pi - y3
      # check y3 y4
      d_21 = cos(y3).evalf()
      d_22 = cos(y4).evalf()
      if eq(d_21, E):
        y34.append(y3)
        a34.append(a3)
      if eq(d_22, E):
        y34.append(y4)
        a34.append(a4)

    else:
      a3 = asin(F/sp2).evalf()
      a4 = np.pi - a3
      y3 = asin(H/sp2).evalf()
      y4 = np.pi - y3

      # check y3 y4
      g_23 = -sp2*cos(y3).evalf()
      g_24 = -sp2*cos(y4).evalf()
      if eq(g_23, G):
        y34.append(y3)
      if eq(g_24, G):
        y34.append(y4)

      # check a3 a4
      c_23 = sp2*cos(a3).evalf()
      c_24 = sp2*cos(a4).evalf()
      a34 = []
      if eq(c_23, C):
        a34.append(a3)
      if eq(c_24, C):
        a34.append(a4)
    
    solutions = []
    for  a_ in a12:
      for y_ in y12:
        sol = self.__euler_check(a_, p1, y_,rot)
        if sol is not None:
          solutions.append(sol)

    for  a_ in a34:
      for y_ in y34:
        sol = self.__euler_check(a_, p2, y_,rot)
        if sol is not None:
          solutions.append(sol)


    return solutions
  
  def get_cordinates(self):
    self.transfotmation_current_a06
    x = self.transfotmation_current_a06[0,3].evalf()
    y = self.transfotmation_current_a06[1,3].evalf()
    z = self.transfotmation_current_a06[2,3].evalf()
    return x,y,z

  def _get_circle_intersections(self,x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    d=sympy.sqrt((x1-x0)**2 + (y1-y0)**2).evalf()
    # non intersecting
    if d > r0 + r1 :
      return {}
    # One circle within other
    if d < abs(r0-r1):
      return {}
    # coincident circles
    if d == 0 and r0 == r1:
      return {}
    else:
      a=(r0**2-r1**2+d**2)/(2*d)
      h=sympy.sqrt(r0**2-a**2).evalf()
      x2=x0+a*(x1-x0)/d   
      y2=y0+a*(y1-y0)/d   
      x3=x2+h*(y1-y0)/d     
      y3=y2-h*(x1-x0)/d 
      x4=x2-h*(y1-y0)/d
      y4=y2+h*(x1-x0)/d
      return x3, y3, x4, y4

  def __angle_for_q2(self,x,y):
    s2 = (y-self.link_legnth_1)/self.link_legnth_2
    c2 = x/self.link_legnth_2
    q2 = None
    q21 = sympy.asin(s2)
    q22 = np.pi - q21
    if eq(sympy.cos(q21).evalf(),c2):
      q2 = q21
    if eq(sympy.cos(q22).evalf(),c2):
      q2 = q22
    return -q2

  def __angle_between_vectors(self,vector1, vector2):
    # vector1 = sympy.Matrix([axis_x-x, axis_y-y,0])
    # vector2 = sympy.Matrix([x, y-model.link_legnth_1,0])
    dot_product = vector1.dot(vector2)
    magnitude1 = vector1.norm()
    magnitude2 = vector2.norm()
    cos_theta = dot_product / (magnitude1 * magnitude2)
    theta = sympy.acos(cos_theta)
    
    if vector2[0] == 0 and vector2[1] >= 0:
      return theta
    elif vector2[0] == 0 and vector2[1] < 0:
      return np.pi - theta
    a = (vector2[1])/(vector2[0])
    y_res = a*(vector1[0]) 
    y_res2 = -a*(vector1[0])
    angle = theta
    if vector1[1] <= y_res  and vector1[1] >= y_res2 or a >= 1e+7: # first quarter
      angle = theta
    elif vector1[1] < y_res  and vector1[1] < y_res2: # second quarter
      angle = np.pi - theta
    elif vector1[1] > y_res  and vector1[1] < y_res2: # third quarter
      angle = np.pi + theta
    elif vector1[1] > y_res  and vector1[1] > y_res2: # fourth quarter
      angle = 2*np.pi - theta
    return angle

  def get_q1q2q3_angles_for_pos(self):
    x = self.pos_x
    y = 0
    z = 520
    ra = 0
    rp = np.pi/2
    ry = np.pi
    # caluclate tool correction for final position
    zyz = KinamticsMatrices.EulerZYZToMatrix(ra,rp,ry).evalf()
    transfrom_tcp = zyz @ KinamticsMatrices.Tx(self.tcp_x) @ KinamticsMatrices.Ty(self.tcp_y) @ KinamticsMatrices.Tz(self.tcp_z + self.link_legnth_4) 
    tcpx = transfrom_tcp[0,3]
    tcpy = transfrom_tcp[1,3]
    tcpz = transfrom_tcp[2,3]
    axis_x = x - tcpx
    axis_y = y - tcpy
    axis_z = z - tcpz
    p = (axis_x**2 + axis_y**2)**0.5
    x1,y1,x2,y2 = self._get_circle_intersections( 0,self.link_legnth_1,self.link_legnth_2, p,axis_z,self.link_legnth_3)
    q21 = self.__angle_for_q2(x1,y1)
    q22 = self.__angle_for_q2(x2,y2)
    x1 = beautify(x1)
    y1 = beautify(y1)
    x2 = beautify(x2)
    y2 = beautify(y2)

    vector1 =  sympy.Matrix([p-x1, axis_z-y1,0])
    vector2 = sympy.Matrix([x1, y1-self.link_legnth_1,0])
    q31 = self.__angle_between_vectors(vector1,vector2)

    vector1 =  sympy.Matrix([p-x2, axis_z-y2,0])
    vector2 = sympy.Matrix([x2, y2-self.link_legnth_1,0])
    q32 = self.__angle_between_vectors(vector1,vector2)

    vector1 =  sympy.Matrix([axis_x, axis_y,0])
    vector2 = sympy.Matrix([2,0,0])
    q1 = self.__angle_between_vectors(vector1,vector2)
    
    solutions = [
      (q1,q21,q31),
      (q1,q22,q32)
    ]
    return solutions
  
  def get_inverse_kinematics_solutions(self, x,y,z, roll, pitch, yaw):
    return []

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
  
  def __plot(self,transformations=None, limits_const=False):
    self.ax.clear()

    if transformations is None:
      transformations = self.get_transformation_matrices()
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
