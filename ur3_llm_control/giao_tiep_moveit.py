"""Lop mong boc cac dich vu cua MoveIt 2 de dung tu Python.

Humble khong co binding MoveGroupInterface cho Python, nen node nay goi thang
cac service va action ma move_group cong bo:

    /plan_kinematic_path     (GetMotionPlan)      - lap ke hoach trong khong gian khop
    /compute_cartesian_path  (GetCartesianPath)   - noi cac diem bang duong thang
    /execute_trajectory      (ExecuteTrajectory)  - gui quy dao xuong controller
    /apply_planning_scene    (ApplyPlanningScene) - them vat can, gan/tha vat

Moi lan lap ke hoach deu bat tranh va cham, nen ban than MoveIt dam bao robot
khong vuot gioi han khop, khong tu va cham va khong dam vao ban.
"""
from __future__ import annotations

import math
import random
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    PlanningOptions,
    RobotState,
    WorkspaceParameters,
)
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
)
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

TEN_KHOP_UR = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class LoiMoveIt(Exception):
    """MoveIt tu choi lap ke hoach hoac thuc thi."""


class LoiThucThi(LoiMoveIt):
    """Controller huy quy dao giua chung (thuong do sai so bam vuot nguong).

    Day la loi tam thoi: lap lai ke hoach tu vi tri hien tai roi chay lai
    thuong se qua duoc, khac han voi truong hop khong the lap duoc ke hoach.
    """


def quy_ve_gan_nhat(gia_tri: float, moc: float) -> float:
    """Doi gia tri khop sang nghiem tuong duong gan moc nhat.

    Cac khop co tam quay +-2*pi (ro nhat la wrist_3) co nhieu gia tri khop ung
    voi CUNG MOT tu the vat ly, cach nhau dung 2*pi. Bo giai IK co the tra ve
    nghiem lech mot vong so voi vi tri hien tai; khi do controller thay sai so
    6.283 rad va lap tuc huy quy dao voi loi PATH_TOLERANCE_VIOLATED du tu the
    dich hoan toan dung.
    """
    return gia_tri + 2.0 * math.pi * round((moc - gia_tri) / (2.0 * math.pi))


def quy_co_tay_ve_giua(khop: Sequence[float]) -> List[float]:
    """Xoay wrist_3 them boi so cua 90 do de no nam trong [-45, 45] do.

    Tay kep hai ngon doi xung va khoi vuong: xoay tay kep 90 hay 180 do quanh
    truc thang dung van kep duoc y het. Loi dung dieu do, luon giu wrist_3 gan
    0 thay vi de no troi dan ve gan +-pi. Neu de troi, co luc quy dao va
    controller hieu wrist_3 lech nhau dung mot vong 2*pi va huy quy dao
    (PATH_TOLERANCE_VIOLATED, sai so -6.283 rad).
    """
    ket_qua = [float(k) for k in khop]
    goc = math.pi / 2.0
    ket_qua[5] -= goc * round(ket_qua[5] / goc)
    return ket_qua


def tao_pose(xyz: Sequence[float], quat_xyzw: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
    pose.orientation = Quaternion(
        x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
        z=float(quat_xyzw[2]), w=float(quat_xyzw[3]))
    return pose


class GiaoTiepMoveIt:
    def __init__(self, node: Node, nhom_hoach_dinh: str = "ur_manipulator",
                 khung: str = "base_link", link_cong_tac: str = "tool0") -> None:
        self._node = node
        self._nhom = nhom_hoach_dinh
        self._khung = khung
        self._link = link_cong_tac
        self._nhom_callback = ReentrantCallbackGroup()

        self._khop_hien_tai: Optional[JointState] = None
        node.create_subscription(
            JointState, "/joint_states", self._nhan_joint_state, 10,
            callback_group=self._nhom_callback)

        self._sv_khop = node.create_client(
            GetMotionPlan, "/plan_kinematic_path", callback_group=self._nhom_callback)
        self._sv_cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self._nhom_callback)
        self._sv_scene = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=self._nhom_callback)
        self._sv_ik = node.create_client(
            GetPositionIK, "/compute_ik", callback_group=self._nhom_callback)
        self._sv_fk = node.create_client(
            GetPositionFK, "/compute_fk", callback_group=self._nhom_callback)
        self._ac_thuc_thi = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory", callback_group=self._nhom_callback)
        self._ac_move = ActionClient(
            node, MoveGroup, "/move_action", callback_group=self._nhom_callback)

    # ------------------------------------------------------------ ket noi
    def cho_san_sang(self, thoi_gian_cho: float = 60.0) -> None:
        cac_dich_vu = [
            ("/plan_kinematic_path", self._sv_khop),
            ("/compute_cartesian_path", self._sv_cartesian),
            ("/apply_planning_scene", self._sv_scene),
            ("/compute_ik", self._sv_ik),
            ("/compute_fk", self._sv_fk),
        ]
        for ten, dich_vu in cac_dich_vu:
            if not dich_vu.wait_for_service(timeout_sec=thoi_gian_cho):
                raise LoiMoveIt(f"khong thay service {ten}; move_group da chay chua?")
        if not self._ac_thuc_thi.wait_for_server(timeout_sec=thoi_gian_cho):
            raise LoiMoveIt("khong thay action /execute_trajectory")
        if not self._ac_move.wait_for_server(timeout_sec=thoi_gian_cho):
            raise LoiMoveIt("khong thay action /move_action")
        for _ in range(int(thoi_gian_cho * 10)):
            if self._khop_hien_tai is not None:
                self.go_xoan_co_tay()
                return
            rclpy.spin_once(self._node, timeout_sec=0.1)
        raise LoiMoveIt("khong nhan duoc /joint_states")

    def go_xoan_co_tay(self) -> None:
        """Dua wrist_3 cua controller ve [-pi, pi] neu no dang bi xoan vong.

        Voi UR3e, wrist_3 khai bao khong gioi han vi tri nen MoveIt coi no la
        khop quay vo han va luon quy goc ve [-pi, pi]; controller thi giu goc
        tho. Neu wrist_3 that dang o, vi du, 5.56 rad thi MoveIt thay -0.72 rad,
        moi quy dao no gui xuong deu lech controller dung 2*pi va bi huy ngay.
        Luc do chi co cach gui thang cho controller mot quy dao quay nguoc lai
        mot vong (cung mot tu the vat ly) roi moi dung MoveIt tiep.
        """
        self._lam_moi_khop()
        khop = self.khop_hien_tai()
        w = khop[5]
        if abs(w) <= math.pi:
            return
        dich = list(khop)
        dich[5] = math.atan2(math.sin(w), math.cos(w))
        self._node.get_logger().warn(
            f"wrist_3 dang xoan ({w:.2f} rad), quay ve {dich[5]:.2f} rad truoc khi lap ke hoach")

        muc_tieu = FollowJointTrajectory.Goal()
        muc_tieu.trajectory.joint_names = list(TEN_KHOP_UR)
        diem = JointTrajectoryPoint()
        diem.positions = dich
        diem.velocities = [0.0] * 6
        # Quay mot vong o toc do vua phai (~1.5 rad/s)
        diem.time_from_start = Duration(seconds=max(2.0, abs(w - dich[5]) / 1.5)).to_msg()
        muc_tieu.trajectory.points = [diem]

        client = ActionClient(self._node, FollowJointTrajectory,
                              "/joint_trajectory_controller/follow_joint_trajectory",
                              callback_group=self._nhom_callback)
        try:
            if not client.wait_for_server(timeout_sec=10.0):
                raise LoiMoveIt("khong thay action cua joint_trajectory_controller")
            tuong_lai = client.send_goal_async(muc_tieu)
            rclpy.spin_until_future_complete(self._node, tuong_lai, timeout_sec=10.0)
            tay_cam = tuong_lai.result()
            if tay_cam is None or not tay_cam.accepted:
                raise LoiMoveIt("controller tu choi lenh go xoan wrist_3")
            tuong_lai_kq = tay_cam.get_result_async()
            rclpy.spin_until_future_complete(self._node, tuong_lai_kq, timeout_sec=30.0)
            ket_qua = tuong_lai_kq.result()
            if (ket_qua is None
                    or ket_qua.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                raise LoiMoveIt("go xoan wrist_3 khong thanh cong")
        finally:
            client.destroy()
        self._lam_moi_khop()

    def _nhan_joint_state(self, tin: JointState) -> None:
        self._khop_hien_tai = tin

    def khop_hien_tai(self) -> List[float]:
        if self._khop_hien_tai is None:
            raise LoiMoveIt("chua co /joint_states")
        bang = dict(zip(self._khop_hien_tai.name, self._khop_hien_tai.position))
        try:
            return [float(bang[ten]) for ten in TEN_KHOP_UR]
        except KeyError as loi:
            raise LoiMoveIt(f"/joint_states thieu khop {loi}") from loi

    def _lam_moi_khop(self, so_vong: int = 12) -> None:
        """Quay executor vai vong de /joint_states duoc cap nhat.

        Node nay chi spin trong luc cho service, nen gia tri cache co the cu
        hang giay sau mot lan robot vua di chuyen.
        """
        for _ in range(so_vong):
            rclpy.spin_once(self._node, timeout_sec=0.02)

    def _trang_thai_bat_dau(self) -> RobotState:
        """Tra ve trang thai rong dang "diff".

        Nhu vay move_group se dung trang thai HIEN TAI ma chinh no dang theo
        doi, thay vi trang thai cache trong node nay. Trang thai cache co the cu
        hang giay, khien quy dao bat dau lech khoi vi tri that va controller
        bao PATH_TOLERANCE_VIOLATED roi huy quy dao.
        """
        trang_thai = RobotState()
        trang_thai.is_diff = True
        return trang_thai

    def _trang_thai_hien_tai_day_du(self) -> RobotState:
        """Ban co ghi ro gia tri khop, chi dung lam hat giong cho IK."""
        self._lam_moi_khop()
        trang_thai = RobotState()
        if self._khop_hien_tai is not None:
            trang_thai.joint_state = self._khop_hien_tai
        trang_thai.is_diff = False
        return trang_thai

    def _goi_dong_bo(self, dich_vu, yeu_cau, thoi_gian_cho: float = 30.0):
        tuong_lai = dich_vu.call_async(yeu_cau)
        rclpy.spin_until_future_complete(self._node, tuong_lai, timeout_sec=thoi_gian_cho)
        if not tuong_lai.done():
            raise LoiMoveIt("service khong phan hoi kip")
        return tuong_lai.result()

    # ------------------------------------------------------------- FK / IK
    def huong_tool_hien_tai(self) -> Quaternion:
        """Huong hien tai cua tool0 trong he khung quy chieu (qua /compute_fk)."""
        yeu_cau = GetPositionFK.Request()
        yeu_cau.header = Header(frame_id=self._khung)
        yeu_cau.fk_link_names = [self._link]
        yeu_cau.robot_state = self._trang_thai_hien_tai_day_du()
        ket_qua = self._goi_dong_bo(self._sv_fk, yeu_cau, thoi_gian_cho=10.0)
        if (ket_qua is None or ket_qua.error_code.val != MoveItErrorCodes.SUCCESS
                or not ket_qua.pose_stamped):
            raise LoiMoveIt("khong tinh duoc FK cua tool0")
        return ket_qua.pose_stamped[0].pose.orientation

    def giai_ik(self, tu_the: Pose, so_lan_thu: int = 8,
                nhieu_hat_giong: float = 0.0,
                hat_giong: Optional[Sequence[float]] = None) -> List[float]:
        """Tim tu the khop dat toi mot pose cua tool0.

        MoveIt tra ve nghiem khong va cham va nam trong gioi han khop. Goi lai
        nhieu lan vi bo giai so co the roi vao nhanh nghiem xau o lan dau.
        """
        loi_cuoi = "khong ro"
        for _ in range(so_lan_thu):
            yeu_cau = GetPositionIK.Request()
            ik: PositionIKRequest = yeu_cau.ik_request
            ik.group_name = self._nhom
            ik.robot_state = self._trang_thai_hien_tai_day_du()
            if hat_giong is not None:
                trang_thai = RobotState()
                trang_thai.joint_state.name = list(TEN_KHOP_UR)
                trang_thai.joint_state.position = [float(k) for k in hat_giong]
                ik.robot_state = trang_thai
            elif nhieu_hat_giong > 0.0:
                # Xao tron hat giong de bo giai IK roi vao nhanh nghiem khac
                trang_thai = RobotState()
                trang_thai.joint_state.name = list(TEN_KHOP_UR)
                goc = self.khop_hien_tai()
                trang_thai.joint_state.position = [
                    g + random.uniform(-nhieu_hat_giong, nhieu_hat_giong) for g in goc]
                ik.robot_state = trang_thai
            ik.avoid_collisions = True
            ik.ik_link_name = self._link
            ik.pose_stamped.header.frame_id = self._khung
            ik.pose_stamped.pose = tu_the
            ik.timeout.sec = 1

            ket_qua = self._goi_dong_bo(self._sv_ik, yeu_cau, thoi_gian_cho=10.0)
            if ket_qua is None:
                loi_cuoi = "service /compute_ik khong tra ve"
                continue
            ma = ket_qua.error_code.val
            if ma != MoveItErrorCodes.SUCCESS:
                loi_cuoi = f"ma loi {ma}"
                continue
            bang = dict(zip(ket_qua.solution.joint_state.name,
                            ket_qua.solution.joint_state.position))
            if all(ten in bang for ten in TEN_KHOP_UR):
                nghiem = [float(bang[ten]) for ten in TEN_KHOP_UR]
                # Quy ve nhanh nghiem gan tu the hien tai nhat
                goc = self.khop_hien_tai()
                return [quy_ve_gan_nhat(gt, mc) for gt, mc in zip(nghiem, goc)]
            loi_cuoi = "nghiem thieu khop"

        x, y, z = tu_the.position.x, tu_the.position.y, tu_the.position.z
        raise LoiMoveIt(f"khong giai duoc IK cho diem ({x:.3f}, {y:.3f}, {z:.3f}): {loi_cuoi}")

    # ------------------------------------------------------- lap ke hoach
    def lap_ke_hoach_khop(self, khop_dich: Sequence[float], he_so_van_toc: float,
                          he_so_gia_toc: float, so_lan_thu: int = 5):
        """Lap ke hoach toi mot tu the khop. Dung cho home va cac buoc chuyen lon."""
        self._lam_moi_khop()
        goc = self.khop_hien_tai()
        khop_dich = [quy_ve_gan_nhat(float(gt), mc) for gt, mc in zip(khop_dich, goc)]

        rang_buoc = Constraints()
        for ten, gia_tri in zip(TEN_KHOP_UR, khop_dich):
            rb = JointConstraint()
            rb.joint_name = ten
            rb.position = float(gia_tri)
            rb.tolerance_above = 0.001
            rb.tolerance_below = 0.001
            rb.weight = 1.0
            rang_buoc.joint_constraints.append(rb)

        yeu_cau = GetMotionPlan.Request()
        req: MotionPlanRequest = yeu_cau.motion_plan_request
        req.group_name = self._nhom
        req.start_state = self._trang_thai_bat_dau()
        req.goal_constraints = [rang_buoc]
        req.num_planning_attempts = so_lan_thu
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = float(he_so_van_toc)
        req.max_acceleration_scaling_factor = float(he_so_gia_toc)
        req.workspace_parameters = WorkspaceParameters(
            header=Header(frame_id=self._khung))
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0

        self._lam_moi_khop()
        ket_qua = self._goi_dong_bo(self._sv_khop, yeu_cau, thoi_gian_cho=40.0)
        if ket_qua is None:
            raise LoiMoveIt("service lap ke hoach khong tra ve")
        ma = ket_qua.motion_plan_response.error_code.val
        if ma != MoveItErrorCodes.SUCCESS:
            raise LoiMoveIt(f"lap ke hoach khop that bai, ma loi {ma}")
        return ket_qua.motion_plan_response.trajectory

    def lap_ke_hoach_tu_the(self, tu_the: Pose, he_so_van_toc: float,
                            he_so_gia_toc: float, dung_sai_vi_tri: float = 0.005,
                            dung_sai_huong: float = 0.05, so_lan_thu: int = 12):
        """Lap ke hoach toi mot tu the cua tool0, khong chi dinh truoc nghiem khop.

        Khac voi cach goi IK roi lap ke hoach toi day khop cu the: o day MoveIt
        duoc tu chon nhanh nghiem nao vua toi duoc vua khong va cham. Nho vay
        khong con bi ket vi IK tra ve mot nhanh khuyu gap nguoc ma OMPL khong
        di toi duoc.
        """
        rang_buoc = Constraints()

        rb_vi_tri = PositionConstraint()
        rb_vi_tri.header = Header(frame_id=self._khung)
        rb_vi_tri.link_name = self._link
        rb_vi_tri.weight = 1.0
        vung = BoundingVolume()
        hinh = SolidPrimitive()
        hinh.type = SolidPrimitive.SPHERE
        hinh.dimensions = [float(dung_sai_vi_tri)]
        vung.primitives = [hinh]
        vung.primitive_poses = [tao_pose(
            (tu_the.position.x, tu_the.position.y, tu_the.position.z), (0.0, 0.0, 0.0, 1.0))]
        rb_vi_tri.constraint_region = vung
        rang_buoc.position_constraints = [rb_vi_tri]

        rb_huong = OrientationConstraint()
        rb_huong.header = Header(frame_id=self._khung)
        rb_huong.link_name = self._link
        rb_huong.orientation = tu_the.orientation
        rb_huong.absolute_x_axis_tolerance = float(dung_sai_huong)
        rb_huong.absolute_y_axis_tolerance = float(dung_sai_huong)
        rb_huong.absolute_z_axis_tolerance = float(dung_sai_huong)
        rb_huong.weight = 1.0
        rang_buoc.orientation_constraints = [rb_huong]

        yeu_cau = GetMotionPlan.Request()
        req: MotionPlanRequest = yeu_cau.motion_plan_request
        req.group_name = self._nhom
        req.start_state = self._trang_thai_bat_dau()
        req.goal_constraints = [rang_buoc]
        req.num_planning_attempts = int(so_lan_thu)
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = float(he_so_van_toc)
        req.max_acceleration_scaling_factor = float(he_so_gia_toc)
        req.workspace_parameters = WorkspaceParameters(header=Header(frame_id=self._khung))
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0

        ket_qua = self._goi_dong_bo(self._sv_khop, yeu_cau, thoi_gian_cho=60.0)
        if ket_qua is None:
            raise LoiMoveIt("service lap ke hoach khong tra ve")
        ma = ket_qua.motion_plan_response.error_code.val
        if ma != MoveItErrorCodes.SUCCESS:
            x, y, z = tu_the.position.x, tu_the.position.y, tu_the.position.z
            raise LoiMoveIt(f"khong lap duoc ke hoach toi ({x:.3f}, {y:.3f}, {z:.3f}), ma loi {ma}")
        return ket_qua.motion_plan_response.trajectory

    def lap_ke_hoach_cartesian(self, cac_diem: Sequence[Pose], buoc: float,
                               ty_le_toi_thieu: float,
                               tranh_va_cham: bool = True,
                               he_so_van_toc: float = 0.1,
                               he_so_gia_toc: float = 0.1) -> Tuple[object, float]:
        """Noi cac diem bang duong thang. Tra ve (quy_dao, ty_le_dat_duoc)."""
        self.go_xoan_co_tay()
        yeu_cau = GetCartesianPath.Request()
        yeu_cau.header = Header(frame_id=self._khung)
        yeu_cau.start_state = self._trang_thai_bat_dau()
        yeu_cau.group_name = self._nhom
        yeu_cau.link_name = self._link
        yeu_cau.waypoints = list(cac_diem)
        yeu_cau.max_step = float(buoc)
        yeu_cau.jump_threshold = 0.0
        # Chan buoc nhay khop lon: neu khong, duong thang co the di qua mot
        # diem ky di va khien mot khop quay gan tron vong trong mot buoc.
        yeu_cau.revolute_jump_threshold = 0.5
        yeu_cau.avoid_collisions = bool(tranh_va_cham)
        # Khong dat hai he so nay thi quy dao duoc dinh thi o TOC DO TOI DA,
        # controller khong bam kip va huy voi loi PATH_TOLERANCE_VIOLATED.
        yeu_cau.max_velocity_scaling_factor = float(he_so_van_toc)
        yeu_cau.max_acceleration_scaling_factor = float(he_so_gia_toc)

        self._lam_moi_khop()
        ket_qua = self._goi_dong_bo(self._sv_cartesian, yeu_cau, thoi_gian_cho=40.0)
        if ket_qua is None:
            raise LoiMoveIt("service cartesian khong tra ve")
        ty_le = float(ket_qua.fraction)
        if ty_le < ty_le_toi_thieu:
            raise LoiMoveIt(f"duong cartesian chi dat {ty_le * 100:.1f}% "
                            f"(toi thieu {ty_le_toi_thieu * 100:.0f}%)")
        return ket_qua.solution, ty_le

    def ke_hoach_toi_tu_the(self, tu_the: Pose, he_so_van_toc: float,
                            he_so_gia_toc: float, so_vong: int = 6):
        """Tim mot quy dao toi tu the, thu lan luot nhieu chien luoc.

        Vong 1 dung nghiem IK tu tu the hien tai (thuong la nghiem "tu nhien"
        nhat). Cac vong sau xao tron hat giong de lay nhanh nghiem khac. Neu
        van khong duoc thi ha xuong lap ke hoach theo rang buoc tu the, noi
        long dan dung sai. Cach nay on dinh hon han viec chi thu mot lan.
        """
        cac_loi: List[str] = []
        for vong in range(so_vong):
            try:
                khop = self.giai_ik(
                    tu_the, so_lan_thu=4,
                    nhieu_hat_giong=0.0 if vong == 0 else 0.35 * vong)
                return self.lap_ke_hoach_khop(khop, he_so_van_toc, he_so_gia_toc)
            except LoiMoveIt as loi:
                cac_loi.append(f"vong {vong + 1}: {loi}")

        for dung_sai in (0.01, 0.02):
            try:
                return self.lap_ke_hoach_tu_the(
                    tu_the, he_so_van_toc, he_so_gia_toc,
                    dung_sai_vi_tri=dung_sai, dung_sai_huong=0.1)
            except LoiMoveIt as loi:
                cac_loi.append(f"rang buoc tu the (dung sai {dung_sai}): {loi}")

        x, y, z = tu_the.position.x, tu_the.position.y, tu_the.position.z
        raise LoiMoveIt(f"khong toi duoc ({x:.3f}, {y:.3f}, {z:.3f}) sau {so_vong} vong; "
                        f"{cac_loi[-1] if cac_loi else ''}")

    # ---------------------------------------------------------- thuc thi
    def thuc_thi(self, quy_dao, thoi_gian_cho: float = 120.0) -> None:
        muc_tieu = ExecuteTrajectory.Goal()
        muc_tieu.trajectory = quy_dao

        tuong_lai = self._ac_thuc_thi.send_goal_async(muc_tieu)
        rclpy.spin_until_future_complete(self._node, tuong_lai, timeout_sec=20.0)
        tay_cam = tuong_lai.result()
        if tay_cam is None or not tay_cam.accepted:
            raise LoiMoveIt("action /execute_trajectory tu choi quy dao")

        tuong_lai_kq = tay_cam.get_result_async()
        rclpy.spin_until_future_complete(self._node, tuong_lai_kq, timeout_sec=thoi_gian_cho)
        ket_qua = tuong_lai_kq.result()
        if ket_qua is None:
            raise LoiMoveIt("thuc thi quy dao khong ket thuc kip")
        ma = ket_qua.result.error_code.val
        if ma != MoveItErrorCodes.SUCCESS:
            raise LoiThucThi(f"thuc thi that bai, ma loi {ma}")

    def di_toi_tu_the(self, tu_the: Pose, he_so_van_toc: float, he_so_gia_toc: float,
                      so_lan_thu: int = 3) -> None:
        """Lap ke hoach roi thuc thi, thu lai neu controller huy giua chung.

        Moi lan thu deu lap ke hoach LAI tu vi tri hien tai, vi sau mot lan bi
        huy robot da nam o cho khac so voi luc lap ke hoach dau tien.
        """
        loi_cuoi: Optional[Exception] = None
        for lan in range(so_lan_thu):
            quy_dao = self.ke_hoach_toi_tu_the(tu_the, he_so_van_toc, he_so_gia_toc)
            try:
                self.thuc_thi(quy_dao)
                return
            except LoiThucThi as loi:
                loi_cuoi = loi
                self._node.get_logger().warn(
                    f"thuc thi bi huy (lan {lan + 1}/{so_lan_thu}), lap ke hoach lai: {loi}")
                for _ in range(10):
                    rclpy.spin_once(self._node, timeout_sec=0.1)
        raise LoiMoveIt(f"thuc thi that bai sau {so_lan_thu} lan: {loi_cuoi}")

    def di_theo_duong_thang(self, cac_diem: Sequence[Pose], buoc: float,
                            ty_le_toi_thieu: float, so_lan_thu: int = 3,
                            tranh_va_cham: bool = True,
                            he_so_van_toc: float = 0.1,
                            he_so_gia_toc: float = 0.1) -> float:
        """Nhu tren nhung cho doan di thang."""
        loi_cuoi: Optional[Exception] = None
        for lan in range(so_lan_thu):
            quy_dao, ty_le = self.lap_ke_hoach_cartesian(
                cac_diem, buoc, ty_le_toi_thieu, tranh_va_cham=tranh_va_cham,
                he_so_van_toc=he_so_van_toc, he_so_gia_toc=he_so_gia_toc)
            try:
                self.thuc_thi(quy_dao)
                return ty_le
            except LoiThucThi as loi:
                loi_cuoi = loi
                self._node.get_logger().warn(
                    f"doan thang bi huy (lan {lan + 1}/{so_lan_thu}), lap lai: {loi}")
                for _ in range(10):
                    rclpy.spin_once(self._node, timeout_sec=0.1)
        raise LoiMoveIt(f"doan thang that bai sau {so_lan_thu} lan: {loi_cuoi}")

    # ----------------------------------------- lap ke hoach va chay mot lan
    def _yeu_cau_toi_khop(self, khop_dich: Sequence[float], he_so_van_toc: float,
                          he_so_gia_toc: float, so_lan_thu: int) -> MotionPlanRequest:
        rang_buoc = Constraints()
        for ten, gia_tri in zip(TEN_KHOP_UR, khop_dich):
            rb = JointConstraint()
            rb.joint_name = ten
            rb.position = float(gia_tri)
            rb.tolerance_above = 0.02
            rb.tolerance_below = 0.02
            rb.weight = 1.0
            rang_buoc.joint_constraints.append(rb)

        req = MotionPlanRequest()
        req.group_name = self._nhom
        req.start_state.is_diff = True          # dung trang thai hien tai cua move_group
        req.goal_constraints = [rang_buoc]
        req.num_planning_attempts = int(so_lan_thu)
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = float(he_so_van_toc)
        req.max_acceleration_scaling_factor = float(he_so_gia_toc)
        req.workspace_parameters = WorkspaceParameters(header=Header(frame_id=self._khung))
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        return req

    def di_toi_khop(self, khop_dich: Sequence[float], he_so_van_toc: float,
                    he_so_gia_toc: float, so_lan_thu: int = 8,
                    thoi_gian_cho: float = 180.0, so_vong_goi: int = 4) -> None:
        """Goi /move_action, thu lai ca lan goi neu that bai.

        OMPL la bo lap ke hoach ngau nhien: cung mot bai toan co the that bai
        lan nay nhung thanh cong lan sau voi hat giong khac. Goi lai vai lan
        re hon nhieu so voi viec doi bo cuc workcell.
        """
        loi_cuoi: Optional[Exception] = None
        for _ in range(max(1, so_vong_goi)):
            try:
                self._goi_move_action(khop_dich, he_so_van_toc, he_so_gia_toc,
                                      so_lan_thu, thoi_gian_cho)
                return
            except LoiThucThi as loi:
                loi_cuoi = loi
                for _ in range(10):
                    rclpy.spin_once(self._node, timeout_sec=0.1)
        raise LoiMoveIt(f"/move_action that bai sau {so_vong_goi} lan goi: {loi_cuoi}")

    def _goi_move_action(self, khop_dich: Sequence[float], he_so_van_toc: float,
                         he_so_gia_toc: float, so_lan_thu: int,
                         thoi_gian_cho: float) -> None:
        """Lap ke hoach VA thuc thi trong mot lan goi /move_action.

        Goi mot lan nhu vay dam bao quy dao luon bat dau tu trang thai that
        cua robot ngay tai thoi diem lap ke hoach, va de MoveIt tu dinh thi
        gian cho quy dao. Cach tach lam hai buoc (lap ke hoach roi thuc thi
        rieng) de bi lech trang thai giua hai buoc, khien controller huy quy
        dao voi loi PATH_TOLERANCE_VIOLATED.
        """
        self.go_xoan_co_tay()
        goc = self.khop_hien_tai()
        dich = [quy_ve_gan_nhat(float(gt), mc) for gt, mc in zip(khop_dich, goc)]
        # wrist_3 thi giu dung gia tri yeu cau: moi dich deu da duoc
        # quy_co_tay_ve_giua dua ve gan 0, quy lai theo tu the hien tai se keo
        # no troi tro lai gan +-2*pi.
        dich[5] = float(khop_dich[5])
        # Sau khi quy ve nghiem gan nhat, gia tri co the vuot +-2*pi. MoveIt se
        # cat ve bien roi lap ke hoach toi MOT DICH KHAC han cai ta yeu cau,
        # kem theo canh bao "constrained to be above the maximum bounds".
        gioi_han = 2.0 * math.pi - 1e-3
        dich = [k - math.copysign(2.0 * math.pi, k) if abs(k) > gioi_han else k
                for k in dich]

        muc_tieu = MoveGroup.Goal()
        muc_tieu.request = self._yeu_cau_toi_khop(dich, he_so_van_toc, he_so_gia_toc, so_lan_thu)
        tuy_chon = PlanningOptions()
        tuy_chon.plan_only = False
        tuy_chon.replan = True
        tuy_chon.replan_attempts = 3
        tuy_chon.replan_delay = 0.3
        muc_tieu.planning_options = tuy_chon

        tuong_lai = self._ac_move.send_goal_async(muc_tieu)
        rclpy.spin_until_future_complete(self._node, tuong_lai, timeout_sec=20.0)
        tay_cam = tuong_lai.result()
        if tay_cam is None or not tay_cam.accepted:
            raise LoiMoveIt("action /move_action tu choi muc tieu")

        tuong_lai_kq = tay_cam.get_result_async()
        rclpy.spin_until_future_complete(self._node, tuong_lai_kq, timeout_sec=thoi_gian_cho)
        ket_qua = tuong_lai_kq.result()
        if ket_qua is None:
            raise LoiMoveIt("/move_action khong ket thuc kip")
        ma = ket_qua.result.error_code.val
        if ma != MoveItErrorCodes.SUCCESS:
            raise LoiThucThi(f"/move_action that bai, ma loi {ma}")

    def _cac_hat_giong(self, tu_the: Pose, goc: Sequence[float]) -> List[Optional[List[float]]]:
        """Sinh cac hat giong IK theo huong cua diem dich.

        Quan trong nhat la goc xoay vai: dat truoc shoulder_pan = atan2(y, x)
        de bo giai IK bam vao nghiem "vuon ra phia truoc". Neu de hat giong
        tu do, KDL hay tra ve nghiem shoulder_pan = pi, tuc la tay may vuon
        nguoc ra sau lung va phai quet 180 do qua mat ban.
        """
        pan = math.atan2(tu_the.position.y, tu_the.position.x)
        # Khong dat gia thiet nao ve nhanh nghiem: de bo giai tu chon truoc,
        # roi thu them vai hat giong khac de co nhieu ung vien lua chon.
        return [
            None,                                             # de bo giai tu chon
            list(goc),                                        # tu the hien tai
            [pan - math.pi, -1.5708, 1.2217, -1.2217, -1.5708, 0.0],
            [pan + math.pi, -1.2217, 1.8326, -2.1817, -1.5708, 0.0],
            [pan, -1.5708, 1.2217, -1.2217, -1.5708, 0.0],
        ]

    def di_toi_diem_nhanh(self, tu_the: Pose, he_so_van_toc: float,
                          he_so_gia_toc: float) -> None:
        """Duong nhanh: mot lan IK lay hat giong la tu the hien tai, roi di.

        Hat giong la tu the hien tai nen nghiem giu nguyen nhanh khuyu/co tay
        dang dung, quy dao ngan va OMPL tim ra gan nhu tuc thi. Chi khi duong
        nay hong moi quay ve di_toi_diem (thu nhieu nhanh nghiem, cham hon).
        """
        try:
            khop = quy_co_tay_ve_giua(self.giai_ik(tu_the, so_lan_thu=2))
            self.di_toi_khop(khop, he_so_van_toc, he_so_gia_toc,
                             so_lan_thu=4, so_vong_goi=2)
            return
        except LoiMoveIt as loi:
            self._node.get_logger().info(f"duong nhanh khong duoc ({loi}), thu cac nhanh khac")
        self.di_toi_diem(tu_the, he_so_van_toc, he_so_gia_toc)

    def di_toi_diem(self, tu_the: Pose, he_so_van_toc: float, he_so_gia_toc: float) -> None:
        """Di toi mot tu the: thu lan luot cac nghiem IK, gan nhat truoc.

        Cac hat giong khac nhau cho ra cac nhanh nghiem khac nhau (khuyu gap
        len hay gap xuong, vai vuon truoc hay lui sau). Sap xep theo khoang
        cach khop de uu tien tu the it phai di chuyen nhat.

        Day la duong du phong khi di_toi_diem_nhanh khong di duoc: thu het moi
        ung vien tim duoc, moi lan cho OMPL toi 15 giay, vi mot so doan can
        nhieu thoi gian tim duong. O day chon do tin cay hon toc do.
        """
        self._lam_moi_khop()
        goc = self.khop_hien_tai()
        ung_vien: List[Tuple[float, List[float]]] = []
        da_thay = set()

        for hat in self._cac_hat_giong(tu_the, goc):
            try:
                nghiem = quy_co_tay_ve_giua(
                    self.giai_ik(tu_the, so_lan_thu=3, hat_giong=hat))
            except LoiMoveIt:
                continue
            # Chi loai nghiem nam ngoai gioi han khop; khong loc theo goc xoay
            # vai, vi he base_link cua UR lech 180 do so voi huong "truoc mat"
            # nen gia thiet pan ~ atan2(y, x) khong dung.
            if any(abs(k) > 2.0 * math.pi for k in nghiem):
                continue
            khoa = tuple(round(k * 1000) for k in nghiem)
            if khoa in da_thay:
                continue
            da_thay.add(khoa)
            khoang_cach = math.sqrt(sum((a - b) ** 2 for a, b in zip(nghiem, goc)))
            ung_vien.append((khoang_cach, nghiem))

        if not ung_vien:
            x, y, z = tu_the.position.x, tu_the.position.y, tu_the.position.z
            raise LoiMoveIt(f"khong co nghiem IT nao cho ({x:.3f}, {y:.3f}, {z:.3f})")

        loi_cuoi: Optional[Exception] = None
        for _, nghiem in sorted(ung_vien, key=lambda k: k[0]):
            try:
                self.di_toi_khop(nghiem, he_so_van_toc, he_so_gia_toc)
                return
            except LoiMoveIt as loi:
                loi_cuoi = loi
        raise LoiMoveIt(f"thu het {len(ung_vien)} nghiem IK deu khong di duoc: {loi_cuoi}")

    # ------------------------------------------------------ planning scene
    def _ap_dung_scene(self, scene: PlanningScene, bat_buoc: bool = True) -> None:
        yeu_cau = ApplyPlanningScene.Request()
        yeu_cau.scene = scene
        ket_qua = self._goi_dong_bo(self._sv_scene, yeu_cau, thoi_gian_cho=15.0)
        if ket_qua is None or not ket_qua.success:
            if not bat_buoc:
                # Don dep scene: go mot vat chua ton tai la chuyen binh thuong
                return
            raise LoiMoveIt("khong cap nhat duoc planning scene")

    def dat_lai_scene(self, cac_ten: Sequence[str]) -> None:
        """Go moi vat dang gan va xoa moi vat can cu truoc khi nap scene moi.

        move_group song lau hon node dieu khien, nen neu mot lan chay truoc bi
        dung giua chung thi khoi van con dang gan vao tool0. Khong don dep thi
        lan chay sau se thay moi duong di deu va cham.
        """
        go = []
        for ten in cac_ten:
            vat = AttachedCollisionObject()
            vat.link_name = self._link
            vat.object.id = ten
            vat.object.operation = CollisionObject.REMOVE
            go.append(vat)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = go
        self._ap_dung_scene(scene, bat_buoc=False)

        xoa = []
        for ten in cac_ten:
            vat = CollisionObject()
            vat.header = Header(frame_id=self._khung)
            vat.id = ten
            vat.operation = CollisionObject.REMOVE
            xoa.append(vat)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = xoa
        self._ap_dung_scene(scene, bat_buoc=False)

    def them_hop(self, ten: str, tam: Sequence[float], kich_thuoc: Sequence[float]) -> None:
        vat = CollisionObject()
        vat.header = Header(frame_id=self._khung)
        vat.id = ten
        hinh = SolidPrimitive()
        hinh.type = SolidPrimitive.BOX
        hinh.dimensions = [float(k) for k in kich_thuoc]
        vat.primitives = [hinh]
        # Vi tri dat vao object.pose, con primitive nam ngay tam object.
        # Neu dat vi tri vao primitive_poses thi lenh MOVE sau nay (chi sua
        # object.pose) se cong don hai vi tri, lam vat can nhay sang cho khac.
        vat.pose = tao_pose(tam, (0.0, 0.0, 0.0, 1.0))
        vat.primitive_poses = [tao_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))]
        vat.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [vat]
        self._ap_dung_scene(scene)

    def xoa_hop(self, ten: str) -> None:
        vat = CollisionObject()
        vat.header = Header(frame_id=self._khung)
        vat.id = ten
        vat.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [vat]
        self._ap_dung_scene(scene, bat_buoc=False)

    def gan_vat(self, ten: str, canh: float, lech_z: float,
                cac_link_cham: Sequence[str] = ()) -> None:
        """Gan mot khoi vao tool0 voi hinh hoc khai bao ro rang.

        Khong muon lai vat can dang nam trong the gioi: vat do dang tiep xuc
        mat ban nen vua gan vao la MoveIt bao va cham ngay. Thay vao do ta xoa
        no khoi the gioi roi tao mot khoi moi gan duoi tool0, hoi nho hon khoi
        that mot chut de tru sai so gap.
        """
        self.xoa_hop(ten)

        gan = AttachedCollisionObject()
        gan.link_name = self._link
        gan.object.header.frame_id = self._link
        gan.object.id = ten
        hinh = SolidPrimitive()
        hinh.type = SolidPrimitive.BOX
        hinh.dimensions = [float(canh) * 0.9] * 3
        gan.object.primitives = [hinh]
        gan.object.primitive_poses = [tao_pose((0.0, 0.0, float(lech_z)), (0.0, 0.0, 0.0, 1.0))]
        gan.object.operation = CollisionObject.ADD
        gan.touch_links = list(cac_link_cham)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [gan]
        self._ap_dung_scene(scene)

    def tha_vat(self, ten: str) -> None:
        """Go khoi khoi tool0 VA xoa han no khoi the gioi.

        MoveIt hieu lenh REMOVE tren mot attached object la "go ra roi tra ve
        the gioi", va no tra ve ngay tai vi tri hien tai cua dau kep. Khi do
        robot lap tuc bi coi la dang va cham voi chinh khoi vua tha, khien moi
        ke hoach tiep theo deu that bai. Vi vay phai xoa not ban trong the
        gioi; vat can that se duoc them lai sau khi tay may rut len.
        """
        go = AttachedCollisionObject()
        go.link_name = self._link
        go.object.id = ten
        go.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [go]
        self._ap_dung_scene(scene)

        self.xoa_hop(ten)

    def doi_vi_tri_hop(self, ten: str, tam: Sequence[float]) -> None:
        """Dat lai vi tri mot vat can da co trong scene."""
        vat = CollisionObject()
        vat.header = Header(frame_id=self._khung)
        vat.id = ten
        vat.pose = tao_pose(tam, (0.0, 0.0, 0.0, 1.0))
        vat.operation = CollisionObject.MOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [vat]
        self._ap_dung_scene(scene)
