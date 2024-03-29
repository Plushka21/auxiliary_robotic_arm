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


# q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')

l1_z = 0#128
l2_y = 0#175.5
l3_z = 0#214
l4_y = 0#92.321
d1_x = 35.687
d1_y = 0#41.1
d1_z = 0#15.247
d2_x = 230.5
d3_x = 130.5
d4_x = 130.5
d5_z = 10#36.4
d5_x = 110


# TODO: clean the code and add comments
class Kinematics:
    def __init__(self, joints) -> None:
        # joints used in manipulator
        self.joints = joints
        # current joints angles, initial values are 0

    def set_init_angles(self, angles=[0, 0, 0, 0, 0]):
        self.angles = sp.Matrix([np.deg2rad(angle)
                                for angle in angles]).evalf()

    def forward_kinematics(self, angles):
        q1m, q2m = np.radians(angles)
        q1, q2, q3, q4, q5 = self.joints
        T_base_0 = Rz(q1m) * Tz(l1_z) * Ty(l2_y) * \
            Ry(q2m) * Tz(l3_z) * Ty(l4_y)
        T01 = Ry(q1) * Tx(-d1_x) * Ty(d1_y) * Tz(d1_z)
        T12 = Rx(q2) * Tx(-d2_x)
        T23 = Rz(q3) * Tx(-d3_x)
        T34 = Rz(-q4) * Tx(-d4_x)
        T45 = Rz(q5) * Tz(d5_z) * Tx(-d5_x)

        # Full transformation to end-effector
        T05 = (T_base_0 * T01 * T12 * T23 * T34 * T45)
        return T05

    # Differentiate the matrix elements to compute Jacobian matrix
    def compute_Jacobian(self, T_matr):
        J = []
        for p in T_matr:
            Ji = []
            for j in self.joints:
                Ji.append(sp.diff(p, j))
            J.append(Ji)
        J = sp.Matrix(J)
        return J

    def inverse_Jacobian(self, J, qi):
        return sp.Matrix(np.linalg.pinv(J(float(qi[0]), float(qi[1]),
                                          float(qi[2]), float(qi[3]), float(qi[4]))))

    def full_Jacobian_function(self, pose, forw_kin):
        q1, q2, q3, q4, q5 = self.joints
        # Jacobian matrix for position
        Jac_matr_pos = self.compute_Jacobian(forw_kin[:3, 3])
        # Jacobian matrix for orientation
        Jac_matr_ori = pose[:3, 0].T * \
            self.compute_Jacobian(forw_kin[:3, 0])
        Jac_full = Jac_matr_pos.row_insert(4, Jac_matr_ori)
        Jac_full_lambd = sp.lambdify((q1, q2, q3, q4, q5), Jac_full)

        des_pos = pose[:3, 3].row_insert(4, sp.Matrix([1]))
        f_full = forw_kin[:3, 3].row_insert(
            4, sp.Matrix([pose[:3, 0].dot(forw_kin[:3, 0])]))
        f_full_lambd = sp.lambdify((q1, q2, q3, q4, q5), f_full)

        return des_pos, Jac_full_lambd, f_full_lambd

    def compute_error(self, des_f, cur_f, qi):
        cur_f = sp.Matrix(cur_f(float(qi[0]), float(qi[1]),
                                        float(qi[2]), float(qi[3]), float(qi[4])))
        error = des_f - cur_f
        pos_vect = sp.Matrix(error[:-1])
        error_sqrt = sp.sqrt(pos_vect.dot(pos_vect))
        # error_ori = sp.sqrt(error[:-1].dot(error[:-1]))
        # error_sqrt = sp.sqrt(error.dot(error))
        return error, error_sqrt, cur_f
    


    # For each given hole, add initial and final position to perform operation
    def targets_positions(self, holes_arr):
        all_targets = []
        for [tx, ty, tz, rz] in holes_arr:
            rz_rad = np.radians(rz)
            # Compute the position and orientation of given hole
            hole_forw_kin = Tx(tx)*Ty(ty)*Tz(tz)*Rz(rz_rad)
            # Compute extra pose
            # Manipulator reaches that pose, turns ON the screwdriver and starts parallel motion to the hole
            # When manipulator reaches hole, it turns OFF the screwdriver and moves back to the initial point
            # So for each given hole, we need 3 poses
            approach_hole_forw_kin = hole_forw_kin * Tx(25)
            all_targets.append([approach_hole_forw_kin, hole_forw_kin])
        return all_targets

    def inverse_kinematics(self, des_points_arr, master_arm_angles, dt=0.001, Kp=5, max_dist=0.1):
        all_targets = self.targets_positions(des_points_arr)
        print("Prepocessed holes data")
        print("Solving inverse kinematics")
        # print(all_des_points)
        all_angles_sol = []
        num_steps = len(all_targets) * 2
        cur_step = 1
        error_list = []
        for i, target in enumerate(all_targets):
            # For each given hole compute forward kinematics based on angles of master manipulator
            forw_kin = self.forward_kinematics(angles=master_arm_angles[i])

            hole_sol = []  # list of solution triplets for each hole
            for j, pose in enumerate(target):
                print(f"\nStep {cur_step}/{num_steps}")
                cur_step += 1
                qi = self.angles
                des_pos, Jac_full_lambd, f_full_lambd = self.full_Jacobian_function(
                    pose, forw_kin)
                print(self.angles)
                print(des_pos)
                error, error_sqrt, cur_f = self.compute_error(
                    des_pos, f_full_lambd, qi)
                local_error_list = [error_sqrt]
                while error_sqrt > max_dist:# or abs(error[-1]) > max_dist:
                    print(
                        f"error: {error_sqrt} approaches {max_dist}", end='\r')
                    J_inv = self.inverse_Jacobian(Jac_full_lambd, qi)

                    qi = (qi + dt *
                          J_inv * Kp * error).evalf()
                    error, error_sqrt, cur_f = self.compute_error(
                        des_pos, f_full_lambd, qi)
                    local_error_list.append(error_sqrt)

                # Scale each angle to range [0; 2pi)
                angles_scaled = []
                for q in qi:
                    q = q % (2 * np.pi)    # force in range [0, 2 pi)
                    if q > sp.pi:             # to [-pi, pi)
                        q -= 2 * np.pi
                    angles_scaled.append(q)

                if j == 0:
                    # Update current angles
                    self.angles = sp.Matrix(angles_scaled)

                # Save angles for current pose
                angles_scaled_deg = [sp.deg(a).evalf() for a in angles_scaled]
                # angles_scaled_deg[3] = -angles_scaled_deg[3]
                hole_sol.append(angles_scaled_deg)
                error_list.append(local_error_list)

            # Save solution triplet for current hole
            hole_sol.append(hole_sol[0])
            all_angles_sol.append(hole_sol)
        print("Finished computing Inverse Kinematics")
        return all_angles_sol, error_list



# # [-0.07723353933012334, -0.26, -0.26, -0.26, -0.18]
# q1, q2, q3, q4, q5 = sp.symbols('q1 q2 q3 q4 q5')
# joints = [q1, q2, q3, q4, q5]
# kin = Kinematics(joints)
# kin.set_init_angles()
# # kin.set_init_angles(angles=[-0.0, -0., -0., -0., -0.])
# des_points_arr = [[-600, 0, 10, 0]]  # , [-350+73.5, 350, 50, 90]]
# master_arm_angles = [
#     [np.rad2deg(-0.25204832), np.rad2deg(-2.10241713)+90]]  # ,
# # master_arm_angles = [[0, 0]]
# # master_arm_angles = [[np.rad2deg(-0.25204832), np.rad2deg(-2.10241713)+90]]
# all_targets, error_list = kin.inverse_kinematics(des_points_arr, master_arm_angles, Kp=10, max_dist=1)
# # print(all_targets)
# # print(kin.forward_kinematics(master_arm_angles[0]).subs(all_targets[0][1]))
# subs_values = {}
# for i, j in enumerate(joints):
#     subs_values[j] = all_targets[0][1][i]
# print(kin.forward_kinematics(master_arm_angles[0]).subs(subs_values).evalf())
# # # all_targets = kin.inverse_kinematics(des_points_arr, master_arm_angles, Kp=2.5)
# for target in all_targets:
#     for pose in target:
#         print(pose)
# holes_arr = [[-610, 225, 255, sp.rad(-90)], [-660, 225, 255, sp.rad(-90)]]
# q_sol_arr = kin.inverse_kinematics(
#     holes_arr, [[np.radians(30), np.radians(-30)], [np.radians(30), np.radians(-30)]])

# for q_sol in q_sol_arr:
#     print(q_sol, "\n")