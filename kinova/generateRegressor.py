import sympy as sp

def derive_regressor_from_spatial_eqn():
    # 1. Define Kinematic Variables (Knowns)
    # --------------------------------------
    # Angular velocity (w), Linear velocity (v)
    wx, wy, wz = sp.symbols('wx wy wz', real=True)
    vx, vy, vz = sp.symbols('vx vy vz', real=True)
    
    # Angular accel (dw), Linear accel (dv)
    dwx, dwy, dwz = sp.symbols('dwx dwy dwz', real=True)
    dvx, dvy, dvz = sp.symbols('dvx dvy dvz', real=True)

    # 2. Define Inertial Parameters (Unknowns to Solve For)
    # -----------------------------------------------------
    # Mass (m)
    m = sp.symbols('m', real=True)
    # Center of Mass * Mass (h = m*c)
    hx, hy, hz = sp.symbols('hx hy hz', real=True)
    # Inertia Tensor (I) - Symmetric 3x3
    Ixx, Iyy, Izz = sp.symbols('Ixx Iyy Izz', real=True)
    Ixy, Ixz, Iyz = sp.symbols('Ixy Ixz Iyz', real=True)

    # The Parameter Vector (Theta)
    theta = sp.Matrix([m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz])

    # 3. Construct Spatial Vectors (6x1)
    # ----------------------------------
    V_a = sp.Matrix([wx, wy, wz, vx, vy, vz])      # Spatial Velocity
    dV_a = sp.Matrix([dwx, dwy, dwz, dvx, dvy, dvz]) # Spatial Accel

    # 4. Construct Helper Matrices (3x3)
    # ----------------------------------
    # Skew-symmetric operator [x]
    def skew(x, y, z):
        return sp.Matrix([[0, -z, y], [z, 0, -x], [-y, x, 0]])

    w_skew = skew(wx, wy, wz)
    v_skew = skew(vx, vy, vz)

    # 5. Construct The Big Matrices from Your Image
    # ---------------------------------------------
    # Adjoint Transpose: [ad_Va]^T
    # Top-left: -[w], Top-right: -[v], Bottom-right: -[w]
    ad_T = sp.zeros(6, 6)
    ad_T[0:3, 0:3] = -w_skew
    ad_T[0:3, 3:6] = -v_skew
    ad_T[3:6, 3:6] = -w_skew

    # Spatial Inertia Matrix: G_a
    # Top-left: Inertia (I_bar), Top-right: Skew(h)
    # Bottom-left: Skew(h)^T = -Skew(h), Bottom-right: m * Identity
    
    I_bar = sp.Matrix([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])
    h_skew = skew(hx, hy, hz)
    
    G_a = sp.zeros(6, 6)
    G_a[0:3, 0:3] = I_bar
    G_a[0:3, 3:6] = h_skew
    G_a[3:6, 0:3] = h_skew.T 
    G_a[3:6, 3:6] = m * sp.eye(3)

    # 6. The Equation from Your Image (Matrix Multiplication)
    # -----------------------------------------------------
    # F_a = G_a * dV_a - [ad_Va]^T * G_a * V_a
    # Note: SymPy handles the matrix multiplication automatically here.
    F_a = G_a * dV_a - ad_T * (G_a * V_a)

    # 7. THE MAGIC: Extract the Regressor
    # -----------------------------------
    # Since F_a is linear in theta, Jacobian returns the coefficient matrix Y.
    Y_regressor = F_a.jacobian(theta)

    print("Done! Extracted Regressor Matrix Y (Shape: 6x10)")
    sp.pprint(Y_regressor[:]) # Print first row as example

if __name__ == "__main__":
    derive_regressor_from_spatial_eqn()