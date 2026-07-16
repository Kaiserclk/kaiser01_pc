import os
import numpy as np
import PyKDL
from typing import List, Optional, Tuple
from dataclasses import dataclass
import xacro
from urdf_parser_py.urdf import URDF
import sys
import os
from kdl_parser.urdf import treeFromUrdfModel
from robot_util.file_path_manager import FilePathManager
file_manager = FilePathManager()
default_urdf = file_manager.GetFileAndCheckExist("robot_urdf")

@dataclass
class TcpPose:
    """End-effector pose result"""
    x: float
    y: float
    z: float
    roll: float   # degrees
    pitch: float  # degrees
    yaw: float    # degrees

    def __repr__(self):
        return (f"TcpPose(x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f}, "
                f"roll={self.roll:.2f}°, pitch={self.pitch:.2f}°, yaw={self.yaw:.2f}°)")


@dataclass
class KinResult:
    """Unified result wrapper for kinematics operations.
    On success: success=True, data=<result>, message=""
    On failure: success=False, data=None, message=<error description>
    On approximate: success=True, approximate=True, data=<best-effort result>,
                    message=<error info>, position_error=<distance in meters>
    On pitch sweep: success=True, adjusted_pitch=<actual pitch used>,
                    message=<info about adjusted pitch>
    """
    success: bool
    data: any  # TcpPose | np.ndarray | List[float] | None
    message: str = ""
    approximate: bool = False
    position_error: float = 0.0  # meters, only meaningful when approximate=True
    adjusted_pitch: Optional[float] = None  # degrees, set when solution found via pitch sweep

    def __repr__(self):
        if self.success and self.approximate:
            return (f"KinResult(success=True, approximate=True, "
                    f"position_error={self.position_error:.4f}m, data={self.data})")
        if self.success:
            extra = ""
            if self.adjusted_pitch is not None:
                extra = f", adjusted_pitch={self.adjusted_pitch:.1f}°"
            return f"KinResult(success=True, data={self.data}{extra})"
        return f"KinResult(success=False, message='{self.message}')"


class ArmKinematics:
    def __init__(self, urdf_path: Optional[str] = default_urdf,
                 base_link: str = "base_link", end_link: str = "gripper_tcp"):
        if urdf_path is None:
            raise ValueError("urdf_path is required")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"Model file not found: {urdf_path}")
        self._robot = self._load_robot_model(urdf_path)
        # Build KDL chain via kdl_parser
        self._chain = self._build_chain(self._robot, base_link, end_link)
        self._num_kdl_joints = self._chain.getNrOfJoints()

        # Dynamically extract joint names and limits from URDF
        self._joint_names, self._joint_limits_deg = self._extract_joint_limits(
            self._robot, base_link, end_link)
        self.NUM_JOINTS = len(self._joint_names)

        if self.NUM_JOINTS != self._num_kdl_joints:
            raise RuntimeError(
                f"Joint count mismatch: URDF chain has {self.NUM_JOINTS} active joints, "
                f"but KDL chain reports {self._num_kdl_joints} joints")
        # Setup kinematic solvers
        self._fk_solver = PyKDL.ChainFkSolverPos_recursive(self._chain)
        # LMA solver with task-space weight matrix L (6x1):
        # [pos_x, pos_y, pos_z, rot_x(roll), rot_y(pitch), rot_z(yaw)]
        # 5DOF arm can control position + roll + pitch, but NOT yaw
        # Give full weight to position and roll/pitch; yaw weight ≈ 0 so it
        # doesn't prevent convergence (solver ignores yaw dimension entirely)
        L = np.array([[1.0], [1.0], [1.0], [1e-15], [1.0], [1e-15]])
        # L = np.array([[1.0], [1.0], [1.0], [1.0], [1.0], [1e-15]])
        # L = np.array([[1.0], [1.0], [1.0], [1.0], [1.0], [1.0]])
        self._ik_solver = PyKDL.ChainIkSolverPos_LMA(
            self._chain,
            L=L,
            eps=1e-5,
            maxiter=1000,
            eps_joints=1e-15
        )
        # Jacobian solver
        self._jac_solver = PyKDL.ChainJntToJacSolver(self._chain)

    @staticmethod
    def _load_robot_model(urdf_path: str) -> URDF:
        doc = xacro.process_file(urdf_path)
        urdf_xml = doc.toxml()
        return URDF.from_xml_string(urdf_xml)

    @staticmethod
    def _build_chain(robot: URDF, base_link: str, end_link: str) -> PyKDL.Chain:
        import kdl_parser.urdf as _kp
        _orig = getattr(_kp, '_toKdlJoint', None)

        def _fixed_to_kdl_joint(jnt):
            if jnt.type == 'fixed':
                return PyKDL.Joint()
            origin = _kp._toKdlPose(jnt.origin)
            if jnt.type in ('revolute', 'continuous'):
                return PyKDL.Joint(
                    jnt.name, origin.p,
                    origin.M * PyKDL.Vector(*jnt.axis),
                    PyKDL.Joint.RotAxis)
            if jnt.type == 'prismatic':
                return PyKDL.Joint(
                    jnt.name, origin.p,
                    origin.M * PyKDL.Vector(*jnt.axis),
                    PyKDL.Joint.TransAxis)
            return PyKDL.Joint()

        _kp._toKdlJoint = _fixed_to_kdl_joint
        try:
            success, tree = treeFromUrdfModel(robot,quiet=True)
        finally:
            if _orig is not None:
                _kp._toKdlJoint = _orig

        if not success:
            raise RuntimeError("Failed to build KDL tree from URDF model")

        chain = tree.getChain(base_link, end_link)
        if chain.getNrOfJoints() == 0:
            raise RuntimeError(
                f"KDL chain from '{base_link}' to '{end_link}' has 0 joints. "
                f"Check that both link names exist in the URDF.")
        return chain

    @staticmethod
    def _extract_joint_limits(robot: URDF, base_link: str, end_link: str) -> Tuple[List[str], List[Tuple[float, float]]]:
        # Trace the kinematic chain from end_link back to base_link
        chain_joints = []
        current_link = end_link
        while current_link != base_link:
            found = False
            for j in robot.joints:
                if j.child == current_link:
                    chain_joints.append(j)
                    current_link = j.parent
                    found = True
                    break
            if not found:
                raise RuntimeError(
                    f"Cannot trace chain: no joint whose child is '{current_link}'")
        chain_joints.reverse()  # base -> end order

        # Collect active joints (revolute, continuous, prismatic)
        joint_names = []
        joint_limits_deg = []
        for j in chain_joints:
            if j.type in ("revolute", "continuous", "prismatic"):
                joint_names.append(j.name)
                if j.limit is not None:
                    lower_deg = np.degrees(j.limit.lower)
                    upper_deg = np.degrees(j.limit.upper)
                    joint_limits_deg.append((lower_deg, upper_deg))
                else:
                    # No limits defined in URDF (e.g. continuous joint)
                    joint_limits_deg.append((-360.0, 360.0))

        return joint_names, joint_limits_deg


    @staticmethod
    def _deg2rad(degrees: float) -> float:
        return degrees * np.pi / 180.0

    @staticmethod
    def _rad2deg(radians: float) -> float:
        return radians * 180.0 / np.pi


    def forward_kinematics(self, joint_angles_deg: List[float]) -> KinResult:
        """Compute forward kinematics (FK).

        Returns:
            KinResult with data=TcpPose on success, data=None on failure
        """

        try:
            q = PyKDL.JntArray(self._num_kdl_joints)
            for i in range(self.NUM_JOINTS):
                q[i] = self._deg2rad(joint_angles_deg[i])

            end_frame = PyKDL.Frame()
            ret = self._fk_solver.JntToCart(q, end_frame)
            if ret < 0:
                return KinResult(False, None, f"FK solver failed (code {ret})")

            x = end_frame.p.x()
            y = end_frame.p.y()
            z = end_frame.p.z()
            roll, pitch, yaw = end_frame.M.GetRPY()

            return KinResult(True, TcpPose(
                x=x, y=y, z=z,
                roll=self._rad2deg(roll),
                pitch=self._rad2deg(pitch),
                yaw=self._rad2deg(yaw)))
        except Exception as e:
            return KinResult(False, None, f"FK exception: {e}")

    def compute_jacobian(self, joint_angles_deg: List[float]) -> KinResult:
        """Compute the Jacobian matrix at the given joint configuration.
        Returns:
            KinResult with data=np.ndarray(6, NUM_JOINTS) on success, data=None on failure
        """

        try:
            q = PyKDL.JntArray(self._num_kdl_joints)
            for i in range(self.NUM_JOINTS):
                q[i] = self._deg2rad(joint_angles_deg[i])

            jac = PyKDL.Jacobian(self._num_kdl_joints)
            status = self._jac_solver.JntToJac(q, jac)
            if status < 0:
                return KinResult(False, None, f"Jacobian solver failed (code {status})")

            jac_matrix = np.zeros((6, self._num_kdl_joints))
            for i in range(6):
                for j in range(self._num_kdl_joints):
                    jac_matrix[i, j] = jac[i, j]
            return KinResult(True, jac_matrix)
        except Exception as e:
            return KinResult(False, None, f"Jacobian exception: {e}")

    def _solve_ik_once(
        self,
        target_frame: PyKDL.Frame,
        seed_angles_deg: Optional[List[float]] = None
    ) -> KinResult:
        """Single IK attempt with given seed. Returns KinResult."""
        q_seed = PyKDL.JntArray(self._num_kdl_joints)
        if seed_angles_deg is not None:
            for i in range(self.NUM_JOINTS):
                q_seed[i] = self._deg2rad(seed_angles_deg[i])

        q_result = PyKDL.JntArray(self._num_kdl_joints)
        ret = self._ik_solver.CartToJnt(q_seed, target_frame, q_result)

        if ret < 0:
            return KinResult(False, None, f"IK solver failed (code {ret})")

        result_deg = [self._rad2deg(q_result[i]) for i in range(self.NUM_JOINTS)]
        # Check joint limits
        for j_idx, angle in enumerate(result_deg):
            low, high = self._joint_limits_deg[j_idx]
            if angle < low - 1.0 or angle > high + 1.0:  # 1° tolerance
                return KinResult(False, None, f"Joint {j_idx} out of limits: {angle:.1f}")
        return KinResult(True, result_deg)

    def _solve_ik_approximate(
        self,
        target_frame: PyKDL.Frame,
        seed_angles_deg: Optional[List[float]] = None
    ) -> Optional[Tuple[List[float], float, float]]:
        """Get best-effort IK result even when solver doesn't converge.

        LMA solver always writes its best approximation into q_result,
        regardless of convergence status.

        Returns:
            (joint_angles_deg, position_error, orientation_error) or None
        """
        q_seed = PyKDL.JntArray(self._num_kdl_joints)
        if seed_angles_deg is not None:
            for i in range(self.NUM_JOINTS):
                q_seed[i] = self._deg2rad(seed_angles_deg[i])

        q_result = PyKDL.JntArray(self._num_kdl_joints)
        # Ignore return code - we want the best-effort result
        self._ik_solver.CartToJnt(q_seed, target_frame, q_result)

        result_deg = [self._rad2deg(q_result[i]) for i in range(self.NUM_JOINTS)]

        # Clamp to joint limits
        for j_idx in range(self.NUM_JOINTS):
            low, high = self._joint_limits_deg[j_idx]
            result_deg[j_idx] = max(low, min(result_deg[j_idx], high))

        # Compute position and orientation error via FK
        fk_result = self.forward_kinematics(result_deg)
        if not fk_result.success:
            return None

        target_pos = np.array([target_frame.p.x(), target_frame.p.y(), target_frame.p.z()])
        actual_pos = np.array([fk_result.data.x, fk_result.data.y, fk_result.data.z])
        pos_error = float(np.linalg.norm(target_pos - actual_pos))

        # Orientation error (roll + pitch only, ignore yaw)
        target_roll, target_pitch, _ = target_frame.M.GetRPY()
        actual_roll = self._deg2rad(fk_result.data.roll)
        actual_pitch = self._deg2rad(fk_result.data.pitch)
        orient_error = float(np.sqrt(
            (target_roll - actual_roll)**2 + (target_pitch - actual_pitch)**2))

        return result_deg, pos_error, orient_error

    def _generate_seeds(self, n: int = 10) -> List[List[float]]:
        """Generate diverse seed angle sets for multi-seed IK."""
        seeds = []
        # Seed 1: joint midpoints
        mid = [(lo + hi) / 2.0 for lo, hi in self._joint_limits_deg]
        seeds.append(mid)
        # Seed 2: zeros (if within limits)
        zeros = [max(lo, min(0.0, hi)) for lo, hi in self._joint_limits_deg]
        seeds.append(zeros)
        # Seed 3+: random within limits
        for _ in range(n - 2):
            s = [float(np.random.uniform(lo, hi))
                 for lo, hi in self._joint_limits_deg]
            seeds.append(s)
        return seeds

    def inverse_kinematics(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        target_roll: float = 0.0,
        target_pitch: float = 0.0,
        target_yaw: float = 0.0,
        seed_angles_deg: Optional[List[float]] = None,
        n_attempts: int = 10,
        allow_approximate: bool = True,
        pitch_search_space: Optional[float] = 10.0
    ) -> KinResult:
        """
        Compute inverse kinematics (IK) using LMA solver with multi-seed retry.
        Note: target_yaw is accepted for API compatibility but ignored internally,
        since this arm only has Roll and Pitch orientation changes at the end-effector.

        The solving strategy proceeds in three stages:
        1. Exact IK with user seed + multi-seed retry at the original target pitch.
        2. If stage 1 fails and pitch_search_space is set, sweep target pitch across
           [target_pitch - pitch_search_space, target_pitch + pitch_search_space]
           (1° step) and retry exact IK for each pitch candidate.
        3. If stage 2 also fails and allow_approximate is True, return the closest
           reachable configuration (approximate=True).

        Args:
            allow_approximate: If True, when all exact IK attempts fail,
                return the closest reachable configuration (approximate=True).
            pitch_search_space: Half-range (degrees) for the pitch sweep in
                stage 2. Set to None or 0 to disable the sweep.
        Returns:
            KinResult with data=List[float] (joint angles in degrees) on success.
            If approximate=True, the result is the closest reachable pose.
        """
        try:
            if seed_angles_deg is not None and len(seed_angles_deg) != self.NUM_JOINTS:
                return KinResult(
                    False, None,
                    f"Expected {self.NUM_JOINTS} seed angles, got {len(seed_angles_deg)}")

            # Yaw is ignored: arm cannot change yaw orientation
            target_frame = PyKDL.Frame(
                PyKDL.Rotation.RPY(
                    self._deg2rad(target_roll),
                    self._deg2rad(target_pitch),
                    0.0),
                PyKDL.Vector(target_x, target_y, target_z))

            # Stage 1: First attempt with user-provided seed
            result = self._solve_ik_once(target_frame, seed_angles_deg)
            if result.success:
                return result

            # Stage 1: Multi-seed retry at original pitch
            seeds = self._generate_seeds(n_attempts)
            for seed in seeds:
                result = self._solve_ik_once(target_frame, seed)
                if result.success:
                    return result

            # Stage 2: Pitch sweep – try varying target pitch within search space
            if pitch_search_space is not None and pitch_search_space > 0:
                pitch_step = 1.0  # degree
                n_steps = int(pitch_search_space / pitch_step)
                # Build candidate list: alternate above/below to try
                pitch_candidates = []
                for i in range(1, n_steps + 1):
                    pitch_candidates.append(target_pitch + i * pitch_step)
                    pitch_candidates.append(target_pitch - i * pitch_step)

                all_seeds_for_sweep = [seed_angles_deg] + seeds
                for try_pitch in pitch_candidates:
                    search_frame = PyKDL.Frame(
                        PyKDL.Rotation.RPY(
                            self._deg2rad(target_roll),
                            self._deg2rad(try_pitch),
                            0.0),
                        PyKDL.Vector(target_x, target_y, target_z))
                    for seed in all_seeds_for_sweep:
                        result = self._solve_ik_once(search_frame, seed)
                        if result.success:
                            result.adjusted_pitch = try_pitch
                            result.message = (
                                f"Exact solution found via pitch sweep "
                                f"(adjusted pitch: {try_pitch:.1f}°)")
                            return result

            # Stage 3: All exact attempts failed – try approximate solution
            if allow_approximate:
                best_approx = None
                best_cost = float('inf')
                best_pos_error = float('inf')
                all_seeds = [seed_angles_deg] + seeds
                for seed in all_seeds:
                    approx = self._solve_ik_approximate(target_frame, seed)
                    if approx is not None:
                        angles, pos_err, orient_err = approx
                        # Combined cost: position (m) + orientation (rad)
                        # Weight orientation error to be comparable to position
                        cost = pos_err + 0.1 * orient_err
                        if cost < best_cost:
                            best_cost = cost
                            best_pos_error = pos_err
                            best_approx = angles
                if best_approx is not None:
                    return KinResult(
                        success=True, data=best_approx,
                        message=f"Approximate solution (position error: {best_pos_error:.4f}m)",
                        approximate=True, position_error=best_pos_error)

            return KinResult(False, None,
                             f"IK failed after {n_attempts + 1} attempts "
                             f"(target unreachable)")
        except Exception as e:
            return KinResult(False, None, f"IK exception: {e}")



    def get_jacobian_pinv(self, joint_angles_deg: List[float],
                          damping: float = 0.01) -> KinResult:
        """Compute the damped pseudo-inverse of the Jacobian matrix.

        Useful for resolved-rate motion control (velocity-level IK):
            dq = J_pinv @ dx
        where dx is the desired end-effector twist [vx, vy, vz, wx, wy, wz].

        Uses damped least-squares (Levenberg-Marquardt):
            J_pinv = J^T @ (J @ J^T + λ²I)^(-1)
        to handle singularities gracefully.

        Args:
            joint_angles_deg: Current joint angles in degrees.
            damping: Damping factor λ for singularity robustness.
                     Larger → more stable near singularities, less accurate.
        Returns:
            KinResult with data=np.ndarray(NUM_JOINTS, 6) pseudoinverse.
        """
        jac_result = self.compute_jacobian(joint_angles_deg)
        if not jac_result.success:
            return KinResult(False, None, f"Jacobian computation failed: {jac_result.message}")

        J = jac_result.data  # (6, N)
        n = self.NUM_JOINTS

        # Damped pseudo-inverse: J^T @ (J@J^T + λ²I)^{-1}
        JJt = J @ J.T  # (6, 6)
        damping_mat = (damping ** 2) * np.eye(6)
        try:
            inv_part = np.linalg.inv(JJt + damping_mat)  # (6, 6)
            J_pinv = J.T @ inv_part  # (N, 6)
            return KinResult(True, J_pinv)
        except np.linalg.LinAlgError as e:
            return KinResult(False, None, f"Pseudo-inverse singular: {e}")

    def compute_manipulability(self, joint_angles_deg: List[float]) -> KinResult:
        """Compute the Yoshikawa manipulability measure.

        μ = sqrt(det(J @ J^T))

        This scalar measure indicates how far the arm is from singularity:
        - μ > 0: arm is well-conditioned (larger = more dexterous)
        - μ ≈ 0: arm is near or at a singularity

        Args:
            joint_angles_deg: Current joint angles in degrees.
        Returns:
            KinResult with data=float (manipulability measure).
        """
        jac_result = self.compute_jacobian(joint_angles_deg)
        if not jac_result.success:
            return KinResult(False, None, f"Jacobian computation failed: {jac_result.message}")

        J = jac_result.data  # (6, N)
        # For a non-square Jacobian, manipulability is sqrt(det(J @ J^T))
        JJt = J @ J.T  # (6, 6)
        det_val = np.linalg.det(JJt)
        mu = np.sqrt(max(det_val, 0.0))
        return KinResult(True, mu)

    def get_joint_names(self) -> List[str]:
        """Get active joint names in chain order."""
        return self._joint_names.copy()

    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """Get joint limits in degrees."""
        return self._joint_limits_deg.copy()

