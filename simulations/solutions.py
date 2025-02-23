from sympy import symbols, Matrix, sin, cos, pi, Symbol, eye, acos, asin, sqrt


q1, q2, q3, q4, q5, q6 = symbols('q1:7')
d1, a2, a3, d4, d5, d6 = symbols('d1 a2 a3 d4 d5 d6')

sol1 = (acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)),
 -acos(-(sin(q1)*sin(q4)*sin(q5) + sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))) + 2*pi,
 asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))) + pi)


sol2 = (acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)),
 -acos(-(sin(q1)*sin(q4)*sin(q5) + sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))) + 2*pi,
 -asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))))

sol5 = (acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)),
 acos((-sin(q1)*sin(q4)*sin(q5) - sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) + cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))),
 asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))) + pi)

sol6 =(acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)),
 acos((-sin(q1)*sin(q4)*sin(q5) - sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) + cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))),
 -asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))))


sol3 = (-acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)) + 2*pi,
 -acos((sin(q1)*sin(q4)*sin(q5) + sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))) + 2*pi,
 pi - asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))))


sol4 = (-acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)) + 2*pi,
 -acos((sin(q1)*sin(q4)*sin(q5) + sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))) + 2*pi,
 asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))))

sol7 = (-acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)) + 2*pi,
 acos((sin(q1)*sin(q4)*sin(q5) + sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))),
 pi - asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))))


sol8 = (-acos(-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5)) + 2*pi,
 acos((sin(q1)*sin(q4)*sin(q5) + sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - cos(q1)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))),
 asin((sin(q4)*cos(q6)*cos(q2 + q3) - sin(q5)*sin(q6)*sin(q2 + q3) + sin(q6)*cos(q4)*cos(q5)*cos(q2 + q3))/sqrt(-(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) - 1)*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5) + 1))))


















