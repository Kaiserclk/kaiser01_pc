import PyKDL
from urdf_parser_py.urdf import URDF
import urdf_parser_py.urdf as urdf

import subprocess


# ====================== 工具函数 ======================
def urdf_to_kdl_chain(robot_urdf, base_link, end_link):
    """
    从 URDF 构建 KDL Chain，正确处理所有关节类型（含 fixed）
    修复说明：
      1. fixed 关节不再跳过，保留其平移/旋转偏移
      2. revolute/prismatic 关节的 xyz 偏移放入 Joint origin（而非 f_tip），
         避免偏移量随关节角度一起旋转导致正解出错
    """
    chain = PyKDL.Chain()

    try:
        chain_segments = robot_urdf.get_chain(base_link, end_link, links=False, joints=True)
    except:
        raise Exception("URDF 连杆链查找失败，请检查 base_link / tip_link 名称")

    for jnt_name in chain_segments:
        jnt = robot_urdf.joint_map[jnt_name]

        xyz = list(jnt.origin.xyz) if (jnt.origin and jnt.origin.xyz is not None) else [0.0, 0.0, 0.0]
        rpy = list(jnt.origin.rpy) if (jnt.origin and jnt.origin.rpy is not None) else [0.0, 0.0, 0.0]

        F_origin = PyKDL.Frame(
            PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
            PyKDL.Vector(xyz[0], xyz[1], xyz[2])
        )

        if jnt.type == "fixed":
            # fixed 关节：完整保留变换，不贡献自由度
            kdl_joint = PyKDL.Joint(jnt_name, PyKDL.Joint.Fixed)
            chain.addSegment(PyKDL.Segment(jnt_name, kdl_joint, F_origin))

        elif jnt.type in ("revolute", "continuous"):
            ax = list(jnt.axis) if jnt.axis is not None else [0.0, 0.0, 1.0]
            kdl_axis = PyKDL.Vector(ax[0], ax[1], ax[2])
            # Joint origin 作为旋转轴通过的点，f_tip 使用相同的 F_origin
            # 这样 T(-origin) 与 f_tip 的 T(origin) 抵消，等效于 URDF 的 T(xyz)*R(axis,q)
            # 与 kdl_parser_py 实现一致
            kdl_joint = PyKDL.Joint(jnt_name,
                                    PyKDL.Vector(xyz[0], xyz[1], xyz[2]),
                                    kdl_axis,
                                    PyKDL.Joint.RotAxis)
            chain.addSegment(PyKDL.Segment(jnt_name, kdl_joint, F_origin))

        elif jnt.type == "prismatic":
            ax = list(jnt.axis) if jnt.axis is not None else [0.0, 0.0, 1.0]
            kdl_axis = PyKDL.Vector(ax[0], ax[1], ax[2])
            kdl_joint = PyKDL.Joint(jnt_name,
                                    PyKDL.Vector(xyz[0], xyz[1], xyz[2]),
                                    kdl_axis,
                                    PyKDL.Joint.TransAxis)
            chain.addSegment(PyKDL.Segment(jnt_name, kdl_joint, F_origin))

    return chain

# ====================== 正运动学 FK ======================
def forward_kine(chain, q_list):
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    joints = PyKDL.JntArray(len(q_list))
    for i, q in enumerate(q_list):
        joints[i] = q

    end_frame = PyKDL.Frame()
    fk_solver.JntToCart(joints, end_frame)

    x = end_frame.p[0]
    y = end_frame.p[1]
    z = end_frame.p[2]
    roll, pitch, yaw = end_frame.M.GetRPY()

    return (x,y,z), (roll,pitch,yaw)

# ====================== 逆运动学 IK ======================
def inverse_kine(chain, target_pos, target_rpy, seed_q=None):
    nj = chain.getNrOfJoints()
    
    # 种子角度（默认全0）
    if seed_q is None:
        seed_q = [0.0]*nj

    # 目标位姿
    target_frame = PyKDL.Frame(
        PyKDL.Rotation.RPY(*target_rpy),
        PyKDL.Vector(*target_pos)
    )

    # IK 求解器（LMA 最稳定）
    ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)
    q_out = PyKDL.JntArray(nj)
    q_init = PyKDL.JntArray(nj)
    for i in range(nj):
        q_init[i] = seed_q[i]

    # 求解
    ret = ik_solver.CartToJnt(q_init, target_frame, q_out)

    if ret < 0:
        return False, None
    
    q_result = [q_out[i] for i in range(nj)]
    return True, q_result

# ====================== 测试主函数 ======================
if __name__ == "__main__":
    URDF_PATH = "/root/work_ws/src/kai01_description/urdf/arm.urdf.xacro"   # 你的URDF路径
    BASE_LINK = "arm_base_link"
    TIP_LINK  = "gripper"

    # 加载URDF（先用 xacro 展开）
    result = subprocess.run(
        ["xacro", URDF_PATH],
        capture_output=True, text=True, check=True
    )
    robot = urdf.URDF.from_xml_string(result.stdout)
    chain = urdf_to_kdl_chain(robot, BASE_LINK, TIP_LINK)
    NJOINTS = chain.getNrOfJoints()
    print(f"✅ 机械臂关节数量：{NJOINTS}")

    # # 正解测试
    test_q = [-0.9, 1.57, 1.57, 0.0]   # 输入关节角度 [arm_joint2, arm_joint3, arm_joint4, arm_joint5]
    pos, rpy = forward_kine(chain, test_q)
    print(f"\n📌 正解结果：")
    print(f"位置：{pos}")
    print(f"姿态RPY：{rpy}")

    # # 逆解测试
    success, ik_q = inverse_kine(chain, pos, rpy, seed_q=test_q)
    if success:
        print(f"\n✅ 逆解成功：")
        print(ik_q)
    else:
        print("\n❌ 逆解失败")