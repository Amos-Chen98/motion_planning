import numpy as np
import pinocchio as pin
import xml.etree.ElementTree as ET


class DesLinkState():
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.yaw = 0
        self.vel_x = 0
        self.vel_y = 0
        self.omega_z = 0


class DesJointState():
    def __init__(self):
        self.pos = 0
        self.vel = 0


class Robot:

    def __init__(self, robot_urdf):
        self.init_pin_model(robot_urdf)

    def init_pin_model(self, robot_urdf):
        param = self.parse_robot_urdf(robot_urdf)
        self.config_dim = param['config_dim']
        self.rotor_num = param['rotor_num']
        self.joint_num = param['joint_num']
        self.rotor_names = ["rotor1", "rotor2", "rotor3", "rotor4"]
        self.m_f_rate = param['m_f_rate']
        self.rotor_directions = param['rotor_directions']
        self.rotor_thrust_max = param['rotor_thrust_max']

        self.param = param  # as a public attribute
        self.robot_model = pin.buildModelFromUrdf(robot_urdf, pin.JointModelPlanar())
        # Ref - Planar joint: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_2c-maths_2b-joints.html#autotoc_md54
        self.robot_data = self.robot_model.createData()

        # Joint indices
        self.rootlink_idx_q = self.robot_model.joints[self.robot_model.getJointId('root_joint')].idx_q
        self.joint1_idx_q = self.robot_model.joints[self.robot_model.getJointId('joint1')].idx_q
        self.joint2_idx_q = self.robot_model.joints[self.robot_model.getJointId('joint2')].idx_q
        self.joint3_idx_q = self.robot_model.joints[self.robot_model.getJointId('joint3')].idx_q
        self.gimbal1_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal1')].idx_q
        self.gimbal2_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal2')].idx_q
        self.gimbal3_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal3')].idx_q
        self.gimbal4_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal4')].idx_q

        self.rootlink_idx_v = self.robot_model.joints[self.robot_model.getJointId('root_joint')].idx_v
        self.joint1_idx_v = self.robot_model.joints[self.robot_model.getJointId('joint1')].idx_v
        self.joint2_idx_v = self.robot_model.joints[self.robot_model.getJointId('joint2')].idx_v
        self.joint3_idx_v = self.robot_model.joints[self.robot_model.getJointId('joint3')].idx_v

        self.selected_indices = [
            self.rootlink_idx_v,      # vx
            self.rootlink_idx_v+1,    # vy
            self.rootlink_idx_v+2,    # wz
            self.joint1_idx_v,        # joint1
            self.joint2_idx_v,        # joint2
            self.joint3_idx_v         # joint3
        ]

        self.rotor_frame_ids = [self.robot_model.getFrameId(name) for name in self.rotor_names]
        self.thrust_frame_ids = [self.robot_model.getFrameId(f"thrust{i+1}") for i in range(self.rotor_num)]
        self.thrust_frame_names = [f"thrust{i+1}" for i in range(self.rotor_num)]
        self.link1_frame_id = self.robot_model.getFrameId("link1")
        self.fc_frame_id = self.robot_model.getFrameId("fc")
        self.unit_thrust_vec = np.array([0.0, 0.0, 1.0])
        self.unit_thrust_vec_skew = self._skew(self.unit_thrust_vec)
        self.unit_thrust_array = np.tile([0.0, 0.0, 1.0], (self.rotor_num, 1))

    @staticmethod
    def parse_robot_urdf(robot_urdf):
        """
        Parse robot URDF file to extract configuration parameters.

        Args:
            robot_urdf (str): Path to the robot URDF file

        Returns:
            dict: Dictionary containing extracted robot parameters including:
                - config_dim: Configuration space dimension
                - joint_num: Number of revolute joints
                - rotor_num: Number of rotors/thrust links
                - rotor_directions: List of rotor rotation directions 
                  (+1: Counter-clockwise rotation when viewed from above,
                   -1: Clockwise rotation when viewed from above)
                - rotor_thrust_min: Minimum thrust force for all rotors [N]
                - rotor_thrust_max: Maximum thrust force for all rotors [N]
                - link_L: Link length
                - propeller_R: Propeller radius
        """
        param = {}

        try:
            # Parse the URDF XML file
            tree = ET.parse(robot_urdf)
            root = tree.getroot()

            # Count revolute joints (actuated joints)
            revolute_joints = root.findall(".//joint[@type='revolute']")
            joint_num = len(revolute_joints)

            # Count thrust links (rotors) by looking for links with names starting with 'thrust'
            all_links = root.findall(".//link")
            thrust_links = [link for link in all_links if link.get('name', '').startswith('thrust')]
            rotor_num = len(thrust_links)

            # Extract rotor rotation directions and thrust force boundaries from rotor joints
            rotor_directions = []
            rotor_thrust_min = 1.0  # Default min thrust: 1.0 N (assuming all rotors are the same)
            rotor_thrust_max = 35.5  # Default max thrust: 35.5 N (assuming all rotors are the same)
            rotor_joints = []

            # Find all rotor joints that connect to thrust links
            for i in range(1, rotor_num + 1):
                rotor_joint_name = f"rotor{i}"
                rotor_joint = root.find(f".//joint[@name='{rotor_joint_name}']")

                if rotor_joint is not None:
                    rotor_joints.append(rotor_joint)

                    # Extract rotation direction from axis
                    axis_elem = rotor_joint.find("axis")
                    if axis_elem is not None:
                        xyz = axis_elem.get("xyz", "0 0 1")
                        xyz_values = [float(x) for x in xyz.split()]
                        if len(xyz_values) >= 3:
                            # Extract z-component of axis to determine rotation direction
                            z_axis = xyz_values[2]
                            if z_axis > 0:
                                rotor_directions.append(1)  # Counter-clockwise rotation (when viewed from above)
                            else:
                                rotor_directions.append(-1)  # Clockwise rotation (when viewed from above)
                        else:
                            rotor_directions.append(1)  # Default to counter-clockwise
                    else:
                        rotor_directions.append(1)  # Default to counter-clockwise if no axis specified

                    # Extract thrust force boundaries from limit element (only from first rotor since all are the same)
                    if i == 1:  # Only get thrust boundaries from first rotor
                        limit_elem = rotor_joint.find("limit")
                        if limit_elem is not None:
                            rotor_thrust_min = float(limit_elem.get("lower", "1.0"))
                            rotor_thrust_max = float(limit_elem.get("upper", "35.5"))
                else:
                    rotor_directions.append(1)  # Default to counter-clockwise

            # Extract m_f_rate parameter
            m_f_rate = -0.0182  # Default value
            m_f_rate_elem = root.find(".//m_f_rate")
            if m_f_rate_elem is not None:
                m_f_rate_value = m_f_rate_elem.get("value")
                if m_f_rate_value is not None:
                    m_f_rate = float(m_f_rate_value)

            # Extract link length from first joint origin
            link_L = 0.6  # default value
            if revolute_joints:
                first_joint = revolute_joints[0]
                origin = first_joint.find("origin")
                if origin is not None:
                    xyz = origin.get("xyz", "0 0 0")
                    xyz_values = [float(x) for x in xyz.split()]
                    if len(xyz_values) >= 1:
                        link_L = abs(xyz_values[0])  # Take the x-component as link length

            # Extract propeller radius from collision geometry
            propeller_R = 0.2025  # default value
            collision_cylinders = root.findall(".//collision/geometry/cylinder")
            for cylinder in collision_cylinders:
                radius = cylinder.get("radius")
                if radius is not None:
                    radius_val = float(radius)
                    # Use the largest radius found (likely the propeller)
                    if radius_val > 0.1:  # Filter out small cylinders
                        propeller_R = radius_val
                        break

            # Calculate configuration dimension (2 for XY position + 1 for yaw + joint_num)
            config_dim = 3 + joint_num

            param = {
                'config_dim': config_dim,
                'joint_num': joint_num,
                'rotor_num': rotor_num,
                'rotor_directions': rotor_directions,
                'rotor_thrust_min': rotor_thrust_min,
                'rotor_thrust_max': rotor_thrust_max,
                'm_f_rate': m_f_rate,
                'link_L': link_L,
                'propeller_R': propeller_R
            }

        except Exception as e:
            print(f"Failed to parse URDF file {robot_urdf}: {e}")
            # Fall back to default values
            param = {
                'config_dim': 6,  # 2 for XY position + 1 for yaw + 3 joints (default)
                'joint_num': 3,
                'rotor_num': 4,
                'rotor_directions': [1, -1, 1, -1],  # Default pattern: CCW, CW, CCW, CW
                'rotor_thrust_min': 1.0,  # Default min thrust: 1.0 N (same for all rotors)
                'rotor_thrust_max': 35.5,  # Default max thrust: 35.5 N (same for all rotors)
                'm_f_rate': -0.0182,  # Default m_f_rate value
                'link_L': 0.6,
                'propeller_R': 0.2025
            }

        return param

    def get_des_states(self, des_pos_array: np.ndarray, des_vel_array=None):
        '''
        Return complete desired states including rootlink, joints, flight controller, and center of gravity
        '''
        state_cmd_len = len(des_pos_array)
        des_states = [{} for _ in range(state_cmd_len)]

        for i in range(state_cmd_len):
            config = des_pos_array[i]

            des_joint_states = [DesJointState() for _ in range(self.joint_num)]
            for j in range(self.joint_num):
                des_joint_states[j].pos = des_pos_array[i][3 + j]
            if des_vel_array is not None:
                for j in range(self.joint_num):
                    des_joint_states[j].vel = des_vel_array[i][3 + j]

            des_states[i] = {'joints': des_joint_states}

            robot_q = self.get_robot_q(config)
            self.forward_kinematics(robot_q)

            rootlink_state = DesLinkState()
            rootlink_state.pos_x = config[0]
            rootlink_state.pos_y = config[1]
            rootlink_state.yaw = config[2]
            if des_vel_array is not None:
                rootlink_state.vel_x = des_vel_array[i][0]
                rootlink_state.vel_y = des_vel_array[i][1]
                rootlink_state.omega_z = des_vel_array[i][2]
            des_states[i]['rootlink'] = rootlink_state

            fc_state = DesLinkState()
            fc_pos, fc_yaw = self._get_fc_pos_yaw()
            fc_state.pos_x = fc_pos[0]
            fc_state.pos_y = fc_pos[1]
            fc_state.yaw = fc_yaw

            # Calculate FC velocity if velocity array is provided
            if des_vel_array is not None:
                # Get FC Jacobian to compute velocities
                fc_jac = pin.getFrameJacobian(
                    self.robot_model, self.robot_data, self.fc_frame_id, pin.ReferenceFrame.WORLD
                )
                fc_jac_selected = fc_jac[:, self.selected_indices]  # shape: (6, n)
                config_vel = des_vel_array[i]
                fc_vel = fc_jac_selected @ config_vel
                # The Jacobian’s first three rows give the linear velocity, and the next three (3–5) give its angular velocity.
                # Ref: (search Spatial velocities in the page) https://docs.ros.org/en/rolling/p/pinocchio/doc/d-practical-exercises/1-directgeom.html
                fc_state.vel_x = fc_vel[0]
                fc_state.vel_y = fc_vel[1]
                fc_state.omega_z = fc_vel[5]  # Angular velocity around z-axis

            des_states[i]['fc'] = fc_state

            cog_state = DesLinkState()
            cog_pos = self._get_cog_pos()
            cog_state.pos_x = cog_pos[0]
            cog_state.pos_y = cog_pos[1]
            cog_state.yaw = fc_state.yaw

            # Calculate COG velocity if velocity array is provided
            if des_vel_array is not None:
                # Get COG Jacobian to compute velocities
                cog_jac = self._get_cog_jac()
                config_vel = des_vel_array[i]
                cog_vel = cog_jac @ config_vel
                cog_state.vel_x = cog_vel[0]
                cog_state.vel_y = cog_vel[1]
                cog_state.omega_z = fc_state.omega_z  # Same angular velocity as FC

            des_states[i]['cog'] = cog_state

        return des_states

    def forward_kinematics(self, robot_q):
        pin.forwardKinematics(self.robot_model, self.robot_data, robot_q)
        pin.updateFramePlacements(self.robot_model, self.robot_data)
        pin.computeJointJacobians(self.robot_model, self.robot_data, robot_q)
        # computeJointJacobians is required if getFrameJacobian is subsequently called.
        # Ref 1: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/jnrh2023/template/frame.html#Frame.getFrameJacobian1
        # Ref 2: https://github.com/stack-of-tasks/pinocchio/issues/1455#issuecomment-851437795

    def _get_fc_pos_yaw(self):
        """
        output: 3 D position of the flight controller in the world frame, yaw
        """
        fc_pose = self.robot_data.oMf[self.fc_frame_id]
        fc_pos = fc_pose.translation
        fc_rot = fc_pose.rotation
        fc_yaw = pin.rpy.matrixToRpy(fc_rot)[2]
        return fc_pos, fc_yaw

    def get_robot_q(self, pos: np.ndarray):
        input_dim = pos.shape[0]
        if input_dim == self.config_dim:
            gimbal_angles = np.zeros(self.rotor_num)
        elif input_dim == self.config_dim + self.rotor_num:
            gimbal_angles = pos[self.config_dim:]
        else:
            raise ValueError(f"Expected input dimension {self.config_dim} or {self.config_dim + self.rotor_num}, got {input_dim}")

        robot_q = np.zeros(self.robot_model.nq)
        robot_q[self.rootlink_idx_q] = pos[0]
        robot_q[self.rootlink_idx_q+1] = pos[1]
        robot_q[self.rootlink_idx_q+2] = np.cos(pos[2])
        robot_q[self.rootlink_idx_q+3] = np.sin(pos[2])
        robot_q[self.joint1_idx_q] = pos[3]
        robot_q[self.joint2_idx_q] = pos[4]
        robot_q[self.joint3_idx_q] = pos[5]
        robot_q[self.gimbal1_idx_q] = np.cos(gimbal_angles[0])
        robot_q[self.gimbal1_idx_q+1] = np.sin(gimbal_angles[0])
        robot_q[self.gimbal2_idx_q] = np.cos(gimbal_angles[1])
        robot_q[self.gimbal2_idx_q+1] = np.sin(gimbal_angles[1])
        robot_q[self.gimbal3_idx_q] = np.cos(gimbal_angles[2])
        robot_q[self.gimbal3_idx_q+1] = np.sin(gimbal_angles[2])
        robot_q[self.gimbal4_idx_q] = np.cos(gimbal_angles[3])
        robot_q[self.gimbal4_idx_q+1] = np.sin(gimbal_angles[3])

        return robot_q

    def get_rotor_pos_jac(self, config):
        robot_q = self.get_robot_q(config)
        self.forward_kinematics(robot_q)
        rotor_pos_array = np.zeros((self.rotor_num, 3))
        rotor_jac_array = np.zeros((self.rotor_num, 6, self.config_dim))
        for i, frame_id in enumerate(self.rotor_frame_ids):
            frame = self.robot_data.oMf[frame_id]
            rotor_pos_array[i] = frame.translation
            rotor_jac_full = pin.getFrameJacobian(
                self.robot_model,
                self.robot_data,
                frame_id,
                pin.ReferenceFrame.WORLD
            )
            rotor_jac_array[i] = rotor_jac_full[:, self.selected_indices]  # shape: (6, n)
        return rotor_pos_array, rotor_jac_array

    def get_joint_pos_vis(self, config):
        robot_q = self.get_robot_q(config)
        self.forward_kinematics(robot_q)
        joint_pos_array = np.zeros((self.joint_num, 3))
        frame_names = ["joint1", "joint2", "joint3"]
        for i, frame_name in enumerate(frame_names):
            frame_id = self.robot_model.getFrameId(frame_name)
            frame = self.robot_data.oMf[frame_id]
            joint_pos_array[i] = frame.translation
        return joint_pos_array

    # def get_fc_rp_dists_grad(self, config):
    #     """
    #     Used in trajectory optimization
    #     Return a tuple of (fc_rp_dists, grad_d2q_array)
    #     fc_rp_dists: feasible roll-pitch distances - length rotor_num
    #     grad_d2q_array: gradients of the distances with respect to the robot configuration
    #     """
    #     fc_rp_dists = np.zeros(self.rotor_num)
    #     grad_d2q_array = np.zeros((self.rotor_num, self.config_dim))
    #     v_array, grad_v2q = self._get_v_grad_v2q(config)
    #     for i in range(self.rotor_num):
    #         v_array[i][2] = 0
    #         grad_v2q[i][2] = np.zeros(self.config_dim)
    #     for i in range(self.rotor_num):
    #         v_i = v_array[i]
    #         v_i_norm = np.linalg.norm(v_i)
    #         v_i_unit_dir = v_i / v_i_norm
    #         d_v_i = grad_v2q[i]
    #         d_v_i_norm = v_i @ d_v_i / v_i_norm
    #         d_v_i_unit_dir = (d_v_i * v_i_norm - np.outer(v_i, d_v_i_norm)) / (v_i_norm ** 2)
    #         approx_dist = 0.0
    #         grad_d2q = np.zeros((1, self.config_dim))
    #         for j in range(self.rotor_num):
    #             if i == j:
    #                 continue
    #             v_j = v_array[j]
    #             d_v_j = grad_v2q[j]
    #             v_j_contribution = np.cross(v_j, v_i_unit_dir)[2]
    #             approx_dist += self._relu_approx(v_j_contribution)
    #             d_v_j_contribution = - self._skew(v_i_unit_dir) @ d_v_j + self._skew(v_j) @ d_v_i_unit_dir
    #             grad_d2q += self._sigmoid(v_j_contribution) * d_v_j_contribution[2, :]
    #         fc_rp_dists[i] = approx_dist
    #         grad_d2q_array[i] = grad_d2q
    #     return fc_rp_dists, grad_d2q_array

    # def _get_v_grad_v2q(self, config):
    #     """
    #     v: the generalized torque arm in 2D spaces, length rotor_num
    #     """
    #     v_array = np.zeros((self.rotor_num, 3))
    #     grad_v2q = np.zeros((self.rotor_num, 3, self.config_dim))
    #     rotor_pos_array, rotor_jac_array = self.get_rotor_pos_jac(config)
    #     u_skew = self.unit_thrust_vec_skew

    #     for i in range(self.rotor_num):
    #         p, grad_p2q = self.transform_between_frames(
    #             rotor_pos_array[i],
    #             'world',
    #             'cog',
    #             return_jacobian=True,
    #             src_jacobian=rotor_jac_array[i]
    #         )

    #         v_array[i] = np.cross(p, self.unit_thrust_vec)
    #         grad_v2q[i] = -u_skew @ grad_p2q
    #     return v_array, grad_v2q

    def get_fc_t_min(self, config) -> float:
        fc_t_array = self.get_fc_t_array(config, return_grad=False)
        return np.min(fc_t_array)

    def get_fc_t_array(self, config, return_grad=True):
        """
        if gimbal config is included in the input config, use the Hydrus_xi thrust model, otherwise, use the classic Hydrus model
        fc_t_min: minimum feasible control torque, len of fc_t_array: rotor_num * (rotor_num - 1)
        """
        unit_torque_array, unit_torque_grad = self._get_unit_torque_grad(config)

        # Pre-calculate the actual number of valid combinations: n*(n-1) for i,j pairs
        num_valid_pairs = self.rotor_num * (self.rotor_num - 1)
        fc_t_array = np.zeros(num_valid_pairs)

        if return_grad:
            fc_t_grad = np.zeros((num_valid_pairs, self.config_dim))

        index = int(0)
        for i in range(self.rotor_num):
            v_i = unit_torque_array[i]
            if return_grad:
                d_v_i = unit_torque_grad[i]

            for j in range(self.rotor_num):
                if i == j:
                    continue

                v_j = unit_torque_array[j]
                fc_t_ij = 0.0

                if return_grad:
                    d_v_j = unit_torque_grad[j]
                    fc_t_grad_ij = np.zeros((self.config_dim,))

                # Vectorized computation for all k values at once
                for k in range(self.rotor_num):
                    if k == i or k == j:
                        continue

                    v_k = unit_torque_array[k]
                    v_triple_product = self._calc_triple_product(v_i, v_j, v_k)
                    signed_torque = v_triple_product * self.rotor_thrust_max
                    # Use numpy maximum for potential vectorization benefits
                    fc_t_ij += np.maximum(0.0, signed_torque)

                    if return_grad:
                        d_v_k = unit_torque_grad[k]
                        d_v_triple_product = self._calc_triple_product_jac(v_i, v_j, v_k, d_v_i, d_v_j, d_v_k)
                        d_signed_torque = d_v_triple_product * self.rotor_thrust_max

                        if signed_torque > 0:
                            fc_t_grad_ij += d_signed_torque

                fc_t_array[index] = fc_t_ij
                if return_grad:
                    fc_t_grad[index] = fc_t_grad_ij
                index += 1

        if not return_grad:
            return fc_t_array
        else:
            return fc_t_array, fc_t_grad

    def _get_unit_torque_grad(self, config) -> tuple:
        """
        Return an array of shape (rotor_num, 3)
        torque[i] = unit_torque_array[i] * thrust[i]
        """
        input_config_dim = len(config)
        # Calculate u_array: shape (rotor_num, 3), each u is the unit vector along the rotor's direction
        if input_config_dim == self.config_dim + self.rotor_num:
            # use the Hydrus_xi thrust model
            unit_thrust_array, unit_thrust_cog_jac = self._get_rotor_unit_thrust_jac()
        elif input_config_dim == self.config_dim:
            # using the classic Hydrus model, all thrust vertically upward
            unit_thrust_array = self.unit_thrust_array
            unit_thrust_cog_jac = np.zeros((self.rotor_num, 3, self.config_dim))  # TODO: check if this is correct

            # test1, test2 = self._get_rotor_unit_thrust_jac() # For debug only

        unit_torque_array = np.zeros((self.rotor_num, 3))
        unit_torque_grad = np.zeros((self.rotor_num, 3, self.config_dim))
        rotor_pos_array, rotor_jac_array = self.get_rotor_pos_jac(config)

        for i in range(self.rotor_num):
            p, p_jac = self.transform_between_frames(rotor_pos_array[i, :],
                                                     'world',
                                                     'cog',
                                                     return_jacobian=True,
                                                     src_jacobian=rotor_jac_array[i])

            u = unit_thrust_array[i]
            u_jac = unit_thrust_cog_jac[i]
            rotor_direction = self.rotor_directions[i]
            unit_torque_array[i] = np.cross(p, u) + self.m_f_rate * rotor_direction * u

            # d(cross(p, u))/dq = cross(dp/dq, u) + cross(p, du/dq)
            #                   = -skew(u) @ dp/dq + skew(p) @ du/dq
            unit_torque_grad[i] = -self._skew(u) @ p_jac + self._skew(p) @ u_jac + self.m_f_rate * rotor_direction * u_jac

        return unit_torque_array, unit_torque_grad

    def _calc_triple_product(self, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        """
        Calculate the triple product: (a x b) · c / ||a x b||.
        Returns 0 if the norm of (a x b) is less than the threshold.
        Optimized for performance with early returns and reduced function calls.

        Args:
            a, b, c: NumPy vectors of shape (3,) or (3,1)

        Returns:
            Scalar value of the triple product
        """
        # Compute cross product
        axb = np.cross(a, b)

        # Calculate norm squared first to avoid sqrt if possible
        norm_squared = np.dot(axb, axb)
        if norm_squared < 1e-12:  # 1e-6 squared for early comparison
            return 0.0

        # Only compute sqrt when needed
        norm = np.sqrt(norm_squared)
        return np.dot(axb, c) / norm

    def _calc_triple_product_jac(self, a, b, c, d_a, d_b, d_c):
        """
        Calculate the Jacobian of the triple product: f = (a x b)T · c / ||a x b||.
        """
        """
        a,b,c: (3,) vectors
        d_a,d_b,d_c: (3, config_dim) Jacobians wrt q
        returns: (config_dim,) gradient wrt q

        Ref: https://proofwiki.org/wiki/Derivative_of_Scalar_Triple_Product_of_Vector-Valued_Functions
        """
        u = np.cross(a, b)  # (3,)
        s = np.linalg.norm(u)
        if s < 1e-12:
            return np.zeros(d_a.shape[1])

        t = np.dot(u, c)  # scalar

        du = np.cross(d_a.T, b) + np.cross(a, d_b.T)  # (N,3)
        ds = (du @ u) / s  # (N,)
        dt = (du @ c) + (u @ d_c)  # (N,)

        df = (dt * s - t * ds) / (s**2)  # (N,)
        return df

    def _skew(self, x):
        """
        Optimized _skew-symmetric matrix computation.
        Pre-extract components to avoid repeated indexing.
        """
        x0, x1, x2 = x[0], x[1], x[2]
        return np.array([[0, -x2, x1],
                         [x2, 0, -x0],
                         [-x1, x0, 0]])

    # def _relu_approx(self, x, epsilon=10):
    #     """
    #     Optimized ReLU approximation with numerical stability.
    #     Clamp extreme values to prevent overflow.
    #     """
    #     # Clamp to prevent overflow in exp
    #     x_eps = np.clip(x * epsilon, -500, 500)
    #     return np.log1p(np.exp(x_eps)) / epsilon

    # def _sigmoid(self, x, epsilon=10):
    #     """
    #     Gradient of _relu_approx - optimized _sigmoid with numerical stability.
    #     """
    #     # Clamp to prevent overflow
    #     x_eps = np.clip(-x * epsilon, -500, 500)
    #     return 1.0 / (1.0 + np.exp(x_eps))

    def get_cog_pos_jac(self, config):
        robot_q = self.get_robot_q(config)
        self.forward_kinematics(robot_q)
        cog_pos = self._get_cog_pos()
        cog_jac = self._get_cog_jac()
        return cog_pos, cog_jac

    def _get_cog_pos(self):
        """
        output: 3 D position of the center of gravity in the world frame
        """
        pin.centerOfMass(self.robot_model, self.robot_data)
        return self.robot_data.com[0]

    def _get_cog_jac(self, extended=False):
        cog_jac_v = pin.jacobianCenterOfMass(self.robot_model,
                                             self.robot_data)[:, self.selected_indices]

        if not extended:
            return cog_jac_v
        else:
            fc_jac_w = pin.getFrameJacobian(self.robot_model,
                                            self.robot_data,
                                            self.fc_frame_id,
                                            pin.ReferenceFrame.WORLD)[3:, self.selected_indices]
            cog_jac = np.vstack((cog_jac_v, fc_jac_w))  # shape: (6, n)
            return cog_jac

    def get_rootlink_pos_from_fc_pos(self, fc_pos: np.ndarray, joint_angles: np.ndarray) -> np.ndarray:
        t_fc = fc_pos[0:3]
        q_fc = fc_pos[3:7]
        qc = pin.Quaternion(q_fc)
        T_fc2world = pin.SE3(qc.toRotationMatrix(), t_fc)
        identity_state = np.hstack((np.zeros(3), joint_angles))
        q0 = self.get_robot_q(identity_state)
        self.forward_kinematics(q0)
        T_fc2root = self.robot_data.oMf[self.robot_model.getFrameId("fc")]
        T_root2fc = T_fc2root.inverse()
        T_root2world = T_fc2world * T_root2fc
        p_root = T_root2world.translation
        R_root = T_root2world.rotation
        yaw = pin.rpy.matrixToRpy(R_root)[2]
        return np.array([p_root[0], p_root[1], p_root[2], yaw])

    def _get_rotor_unit_thrust_jac(self) -> tuple:
        '''
        Return a tuple containing the unit thrust vectors in COG frame and their Jacobians.
        '''
        unit_thrust_cog = np.zeros((self.rotor_num, 3))
        unit_thrust_cog_jac = np.zeros((self.rotor_num, 3, self.config_dim))
        jac_in_thrust_frame = np.zeros((6, self.config_dim))

        for i, thrust_frame_name in enumerate(self.thrust_frame_names):
            unit_thrust_cog[i], unit_thrust_cog_jac[i] = self.transform_between_frames(
                self.unit_thrust_vec,
                thrust_frame_name,
                'cog',
                return_jacobian=True,
                src_jacobian=jac_in_thrust_frame
            )  # shape: (3,), (3, n)
            # print("norm of unit_thrust_cog[{}]: {}".format(i, np.linalg.norm(unit_thrust_cog[i])))

        # TODO: the norm of unit_thrust_cog is larger than 1. Check it!

        return unit_thrust_cog, unit_thrust_cog_jac

    def _get_cog_transform(self):
        cog_pos = self._get_cog_pos()
        T_fc2world = self.robot_data.oMf[self.fc_frame_id]
        R_fc2world = T_fc2world.rotation
        return pin.SE3(R_fc2world, cog_pos)

    def transform_between_frames(self, point: np.ndarray, src_frame: str, target_frame: str, return_jacobian=False, src_jacobian=None) -> tuple:
        """
        Transform a 3D point between coordinate frames, optionally computing Jacobian.

        Args:
            point: 3D point in the source frame
            src_frame: Source frame name ('world', 'cog', or frame name)
            target_frame: Target frame name ('world', 'cog', or frame name)
            return_jacobian: Whether to compute and return Jacobian
            src_jacobian: Jacobian matrix ∂(point_src)/∂q (only first 3 rows used). Please note that if the point is a fixed point in the source frame, the Jacobian will be zero.

        Returns:
            point_in_target: 3D point transformed to target frame
            point_jac_in_target: Jacobian matrix ∂(point_target)/∂q (if return_jacobian=True)
        """
        # Get transformation matrices for source and target frames
        T_src2world = self._get_frame_transform(src_frame)
        T_target2world = self._get_frame_transform(target_frame)

        # Compute point transformation
        T_world2target = T_target2world.inverse()
        point_in_target = T_world2target * T_src2world * point

        if not return_jacobian or src_jacobian is None:
            return point_in_target

        # Compute jacobian transformation
        point_jac_in_target = self._compute_transform_jacobian(
            point, src_frame, target_frame, src_jacobian,
            T_src2world, T_world2target, point_in_target
        )

        return point_in_target, point_jac_in_target

    def _get_frame_transform(self, frame_name: str) -> pin.SE3:
        """
        Get the transformation matrix from specified frame to world frame.

        Args:
            frame_name: Frame name ('world', 'cog', or frame name)

        Returns:
            Transformation matrix from frame to world
        """
        if frame_name == 'cog':
            return self._get_cog_transform()
        elif frame_name == 'world':
            return pin.SE3.Identity()
        else:
            frame_id = self.robot_model.getFrameId(frame_name)
            return self.robot_data.oMf[frame_id]

    def _compute_transform_jacobian(self, point: np.ndarray, src_frame: str, target_frame: str,
                                    src_jacobian: np.ndarray, T_src2world: pin.SE3,
                                    T_world2target: pin.SE3, point_in_target: np.ndarray) -> np.ndarray:
        """
        Compute the Jacobian of point transformation between frames.

        Args:
            point: Original 3D point in source frame
            src_frame: Source frame name
            target_frame: Target frame name
            src_jacobian: Input Jacobian matrix ∂(point_src)/∂q
            T_src2world: Transformation from source to world
            T_world2target: Transformation from world to target
            point_in_target: Transformed point in target frame

        Returns:
            Jacobian matrix ∂(point_target)/∂q
        """
        # First, transform jacobian to world frame
        if src_frame == 'world':
            # Point is already in world frame, jacobian is just src_jacobian
            point_jac_in_world = src_jacobian[:3]
        else:
            R_src2world = T_src2world.rotation

            # Get frame jacobian for source frame
            if src_frame == 'cog':
                src_frame_jac = self._get_cog_jac(extended=True)
            else:
                src_frame_id = self.robot_model.getFrameId(src_frame)
                src_frame_jac = pin.getFrameJacobian(
                    self.robot_model, self.robot_data, src_frame_id, pin.ReferenceFrame.WORLD
                )[:, self.selected_indices]

            src_frame_jac_v = src_frame_jac[:3]  # translational jacobian
            src_frame_jac_w = src_frame_jac[3:]  # rotational jacobian

            # point_in_world = R_src2world * point_in_src_frame + src_frame_origin
            point_jac_in_world = (R_src2world @ src_jacobian[:3] + src_frame_jac_v -
                                  self._skew(R_src2world @ point) @ src_frame_jac_w)

        # Then, transform from world frame to target frame
        if target_frame == 'world':
            # Target is world frame, no additional transformation needed
            point_jac_in_target = point_jac_in_world
        else:
            R_world2target = T_world2target.rotation

            # Get frame jacobian for target frame
            if target_frame == 'cog':
                target_frame_jac = self._get_cog_jac(extended=True)
            else:
                target_frame_id = self.robot_model.getFrameId(target_frame)
                target_frame_jac = pin.getFrameJacobian(
                    self.robot_model, self.robot_data, target_frame_id, pin.ReferenceFrame.WORLD
                )[:, self.selected_indices]

            target_frame_jac_v = target_frame_jac[:3]
            target_frame_jac_w = target_frame_jac[3:]

            # point_in_target = R_world2target * (point_in_world - target_frame_origin)
            point_jac_in_target = (R_world2target @ (point_jac_in_world - target_frame_jac_v) +
                                   self._skew(point_in_target) @ (R_world2target @ target_frame_jac_w))

        return point_jac_in_target
