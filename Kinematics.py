import sympy as sp
import numpy as np

def Rx(q):
    return sp.Matrix([[1, 0, 0, 0],
                      [0, sp.cos(q), -sp.sin(q), 0],
                      [0, sp.sin(q), sp.cos(q), 0],
                      [0, 0, 0, 1]])


def Ry(q):
    return sp.Matrix([[sp.cos(q), 0, sp.sin(q), 0],
                      [0, 1, 0, 0],
                      [-sp.sin(q), 0, sp.cos(q), 0],
                      [0, 0, 0, 1]])


def Rz(q):
    return sp.Matrix([[sp.cos(q), -sp.sin(q), 0, 0],
                      [sp.sin(q), sp.cos(q), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])


def Tx(d):
    return sp.Matrix([[1, 0, 0, d],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])


def Ty(d):
    return sp.Matrix([[1, 0, 0, 0],
                      [0, 1, 0, d],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])


def Tz(d):
    return sp.Matrix([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, d],
                      [0, 0, 0, 1]])


q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')
l1 = 128
l2 = 175.5
l3 = 157.1
l4 = 145
# l5 = 80
d1 = 320.5
d2 = 130.5
d3 = 60
d4 = 60

#TODO: clean the code and add comments
class Kinematics:
    def __init__(self, joints) -> None:
        self.joints = joints

    def forward_kinematics(self, q1m, q2m):
        T_base_0 = Rz(q1m) * Tz(l1) * Ty(l2) * Ry(q2m) * Tz(l3)
        T01 = Ry(q1) * Ty(l4)
        T12 = Rx(q2) * Tx(-d1)
        T23 = Rz(q3) * Tx(-d2)
        T34 = Rz(q4) * Tx(-d2)
        T45 = Rz(q5) * Tx(-d4) * Tz(d3)

        # Full transformation to end-effector
        T05 = (T_base_0 * T01 * T12 * T23 * T34 * T45)#.applyfunc(sp.simplify)
        return T05
    
    def compute_Jacobian(self, T_matr):
        J = []
        for p in T_matr:
            Ji = []
            for j in self.joints:
                Ji.append(sp.diff(p, j))  # .simplify())
            J.append(Ji)
        J = sp.Matrix(J)
        return J

    def inverse_Jacobian(self, J, qi):
        return sp.Matrix(np.linalg.pinv(J(float(qi[0]), float(qi[1]),
                                          float(qi[2]), float(qi[3]), float(qi[4]))))


    def compute_error(self, des_pos, f, qi):
        error = des_pos - sp.Matrix(f(float(qi[0]), float(qi[1]),
                                     float(qi[2]), float(qi[3]), float(qi[4])))
        error_sqrt = sp.sqrt(error.dot(error))
        # angle = sp.Matrix([q(float(qi[0]), float(qi[1]),
        #                      float(qi[2]), float(qi[3]), float(qi[4]))])
        # error = vect.row_insert(4, angle)
        # dist = sp.sqrt(vect.dot(vect)).evalf()
        # unit_dist = abs(angle[0])
        # return dist, unit_dist, error
        return error, error_sqrt

    def inverse_kinematics(self, Jac_full_lambd, f_full_lambd, des_pos, init_angle, dt=0.001, Kp=10, max_dist=0.1):
        dist_arr, vectors_arr = [], []
        qi = init_angle
        error, error_sqrt = self.compute_error(des_pos, f_full_lambd, qi)
        Jinv = self.inverse_Jacobian(Jac_full_lambd, qi)

        # print((Jinv*error).shape)
        # print((qi+Jinv*error).shape)
        # print(error.shape)
        # dist, unit_dist, 
        # error = sp.sqrt(error.dot(error))
        # print(error)
        # print(type(error), error)
        # dist_arr.append(dist)
        # vectors_arr.append(unit_dist)
        # while dist > max_dist or unit_dist > 10**(-5):
        while error_sqrt > max_dist:
            # print(error_sqrt, end='\r')
            Jinv = self.inverse_Jacobian(Jac_full_lambd, qi)
        #     print(qi.shape)
            qi = (qi + dt * Jinv * Kp * error).evalf()
        #     print(qi.shape)
            error, error_sqrt = self.compute_error(des_pos, f_full_lambd, qi)
            # error = sp.sqrt(error.dot(error))

        #     print(f"{dist = }, {unit_dist = }, {len(vectors_arr) = }", end='\r')
        #     # dist_arr.append(dist)
        #     # vectors_arr.append(unit_dist)

        return qi #, dist_arr, vectors_arr

    def holes_positions(self, tx_arr=[0], ty_arr=[0], tz_arr=[0], rz_arr=[0]):
        holes_pos_arr = []
        if len(tx_arr) == len(ty_arr) == len(tz_arr) == len(rz_arr):
            for tx, ty, tz, rz in zip(tx_arr, ty_arr, tz_arr, rz_arr):
                holes_pos_arr.append(Tx(tx)*Ty(ty)*Tz(tz)*Rz(rz))
        else:
            print("Put the same number of poistions for all arguments!")
        return holes_pos_arr


joints = [q1, q2, q3, q4, q5]
kin = Kinematics(joints)

q1m, q2m = np.radians([30, -30])
forw_kin = kin.forward_kinematics(q1m, q2m)

holes_arr = kin.holes_positions([-610], [225], [255], [sp.rad(-90)])
Jac_matr_pos = kin.compute_Jacobian(forw_kin[:3,3]) # Jacobian matrix for position
# Jacobian matrix for orientation
Jac_matr_ori = holes_arr[0][:3, 0].T * kin.compute_Jacobian(forw_kin[:3, 0])

Jac_full = Jac_matr_pos.row_insert(4, Jac_matr_ori)
Jac_full_lambd = sp.lambdify((q1, q2, q3, q4, q5), Jac_full)

des_pos = holes_arr[0][:3, 3].row_insert(4, sp.Matrix([1]))
# print()
f_full = forw_kin[:3, 3].row_insert(4, sp.Matrix(
    [holes_arr[0][:3, 0].dot(forw_kin[:3, 0])]))
f_full_lambd = sp.lambdify((q1, q2, q3, q4, q5), f_full)

init_angle = sp.Matrix([np.deg2rad(0.00001) for _ in range(len(joints))]).evalf()
q_sol_arr = kin.inverse_kinematics(Jac_full_lambd, f_full_lambd, des_pos, init_angle)

q_sol_arr = [q for q in q_sol_arr]

print("Solution in degrees:")
new_q_sol_arr = []
for a in q_sol_arr:
    a = a % (2 * sp.pi)    # force in range [0, 2 pi)
    if a > sp.pi:             # to [-pi, pi)
        a -= 2 * sp.pi
    print(sp.deg(a).evalf())
    new_q_sol_arr.append(a)
