from sympy import symbols, Matrix, sin, cos
from sympy import latex

# Step 1: Define symbolic variables for trigonometric arguments
x, y, z = symbols('x1 y z')
# Define the rotation matrix around the X-axis
theta = symbols('theta')


# Step 2: Create 4x4 matrices where each entry is a combination of sin and cos
A = Matrix([
    [sin(x), cos(x), sin(2*x)],
    [cos(y), sin(y), cos(2*y)],
    [sin(z), cos(z), sin(2*z)]
])

B = Matrix([
    [cos(x), sin(x), cos(2*x)],
    [sin(y), cos(y), sin(2*y)],
    [cos(z), sin(z), cos(2*z)]
])


# Step 3: Multiply the matrices (A * B * C)
result = A * B 

# Step 4: Display the resulting matrix
print("Resulting 4x4 Matrix with Sine and Cosine:")
print(result)
# Step 5: Convert the resulting matrix to LaTeX format
latex_result = latex(result)

# Step 6: Print the LaTeX formatted matrix
print("LaTeX formatted matrix:")
print(latex_result)