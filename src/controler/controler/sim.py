import numpy as np
import copy

from sympy import symbols, Matrix, sin, cos, pi, Symbol, eye, acos ,sqrt, asin
import sympy
import time

import sympy.vector

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
      return (sympy.re(a),sympy.re(p),sympy.re(y))
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
    if abs(sp1) <= 1e-5: # for p = 0 or p = pi |  Case for p = 0
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
    if abs(sp2) <= 1e-5: # for p = 0 or p = pi |  Case for p = pi
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
      return None, None, None, None
    # One circle within other
    if d < abs(r0-r1):
      return None, None, None, None
    # coincident circles
    if d == 0 and r0 == r1:
      return None, None, None, None
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
  
  def __angle_between_vectors2(self,v1, v2):
    ang = sympy.atan2(v1[0]*v2[1]-v2[0]*v1[1],v1[0]*v2[0]+v1[1]*v2[1])
    angle = sympy.Mod(ang, 2*np.pi).evalf()
    if (abs(sympy.im(angle)) > 1e-2):
      return None
    else:
      return sympy.re(angle)

  def get_q1q2q3_angles_for_pos(self):
    x = self.pos_x
    y = self.pos_y
    z = self.pos_z
    ra = self.rot_roll
    rp = self.rot_pitch
    ry = self.rot_yaw
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
    if x1 is None or x2 is None or y1 is None or y2 is None:
      return []
    
    q21 = self.__angle_for_q2(x1,y1)
    q22 = self.__angle_for_q2(x2,y2)
    x1 = beautify(x1)
    y1 = beautify(y1)
    x2 = beautify(x2)
    y2 = beautify(y2)

    vector1 =  sympy.Matrix([p-x1, axis_z-y1,0])
    vector2 = sympy.Matrix([x1, y1-self.link_legnth_1,0])
    q31 = self.__angle_between_vectors2(vector1,vector2)

    vector1 =  sympy.Matrix([p-x2, axis_z-y2,0])
    vector2 = sympy.Matrix([x2, y2-self.link_legnth_1,0])
    q32 = self.__angle_between_vectors2(vector1,vector2)

    vector1 = sympy.Matrix([1, 0, 0])  # unit vector along the x-axis
    vector2 =  sympy.Matrix([axis_x, axis_y,0])
    q1 = self.__angle_between_vectors2(vector1,vector2)
    # always returns two solutions
    return [ (q1,q21,q31), (q1,q22,q32) ]

  def __inverse_matrix_check(self,a,b,c,target_rot):
    a1 = self.a_3_4().subs({
      self.q4 : a,
      self.l3 : 0, 
      }).evalf()
    a2 = self.a_4_5().subs({
      self.q5 : b,
      }).evalf()
    a3 = self.a_5_6().subs({
      self.q6 : c,
      self.l4 : 0,
      }).evalf()
    eu = a1 @ a2 @ a3
    eu = eu[:3,:3]
    if eqMatrix(eu, target_rot,0.001):
      return (a,b,c)
    return None
  
  def get_inverse_kinematics_solutions(self):
    zyz = KinamticsMatrices.EulerZYZToMatrix(self.rot_roll,self.rot_pitch,self.rot_yaw).evalf()
    solutions = self.get_q1q2q3_angles_for_pos()
    if len(solutions) == 1:
      pass
    angles = []
    for s in solutions:
      # we will ingore translation on 4 axis and only take translation on z axis
      a_0_3 = self.a_0_1() @ self.a_1_2() @ self.a_2_3()
      a_0_3 = a_0_3.subs({
        self.q1 : s[0],
        self.q2 : s[1],
        self.q3 : s[2],
        self.l1 : self.link_legnth_1,
        self.l2 : self.link_legnth_2,
        self.l3 : self.link_legnth_3,
        }).evalf()
      a_0_3_inv = a_0_3.inv()
      if a_0_3_inv is None:
        continue
      comp = a_0_3_inv @ zyz
      rot = comp[:3,:3]
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
      # a = q4   p = q5  y = q6
      p1 = sympy.re(acos(J).evalf())
      sp1 = sin(p1).evalf()
      y12 = []
      a12 = []
      if abs(sp1) <= 1e-5: # for p = 0 or p = pi |  Case for p = 0
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
        a1 = asin(-F/sp1).evalf()
        a2 = np.pi - a1
        y1 = asin(-H/sp1).evalf()
        y2 = np.pi - y1

        # check y1 y2
        g_11 = sp1*cos(y1).evalf()
        g_12 = sp1*cos(y2).evalf()

        if eq(g_11, G,0.001):
          y12.append(y1)
        if eq(g_12, G,0.001):
          y12.append(y2)

        # check a1 a2
        c_11 = -sp1*cos(a1).evalf()
        c_12 = -sp1*cos(a2).evalf()
        if eq(c_11, C,0.001):
          a12.append(a1)
        if eq(c_12, C,0.001):
          a12.append(a2)

      #  second solution for P angle
      p2 = 2 * np.pi - sympy.re(acos(J).evalf())
      sp2 = sin(p2).evalf()
      y34 = []
      a34 = []
      if abs(sp2) <= 1e-5: # for p = 0 or p = pi |  Case for p = pi
        a3 = 0 # we set a to 0
        a4 = 0 #
        y3 = asin(D).evalf()
        y4 = np.pi - y3
        # check y3 y4
        d_21 = cos(y3).evalf()
        d_22 = cos(y4).evalf()
        if eq(d_21, E,0.001):
          y34.append(y3,)
          a34.append(a3)
        if eq(d_22, E,0.001):
          y34.append(y4)
          a34.append(a4)

      else:
        a3 = asin(-F/sp2).evalf()
        a4 = np.pi - a3
        y3 = asin(-H/sp2).evalf()
        y4 = np.pi - y3

        # check y3 y4
        g_23 = sp2*cos(y3).evalf()
        g_24 = sp2*cos(y4).evalf()
        if eq(g_23, G,0.001):
          y34.append(y3)
        if eq(g_24, G,0.001):
          y34.append(y4)

        # check a3 a4
        c_23 = -sp2*cos(a3).evalf()
        c_24 = -sp2*cos(a4).evalf()
        a34 = []
        if eq(c_23, C,0.001):
          a34.append(a3)
        if eq(c_24, C,0.001):
          a34.append(a4)
      
      solut = []
      for  a_ in a12:
        for y_ in y12:
          sol = self.__inverse_matrix_check(a_, p1, y_,rot)
          if sol is not None:
            solut.append(sol)

      for  a_ in a34:
        for y_ in y34:
          sol = self.__inverse_matrix_check(a_, p2, y_,rot)
          if sol is not None:
            solut.append(sol)

      for sa in solut:
        angles.append( (sympy.re(s[0]),sympy.re(s[1]),sympy.re(s[2]), sympy.re(sa[0]), sympy.re(sa[1]),sympy.re(sa[2])) )
    
    return angles
