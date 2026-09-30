"""Dieu khien tay kep hai ngon qua tay_kep_controller.

Hai ngon la hai khop truot doi xung (khop_ngon_trai, khop_ngon_phai), duoc
mot JointTrajectoryController rieng dieu khien. Moi lenh mo/dong gui mot quy
dao mot diem roi CHO controller bao xong, nen skill chi di tiep khi ngon da
thuc su toi vi tri.

Khoi 45 mm khong duoc giu bang ma sat trong Gazebo (rat de truot, vang). Ngon
dong toi khe 46 mm de om sat khoi, con viec "giu" vat do MoveIt dam nhan qua
AttachedCollisionObject va dong_bo_gazebo cho khoi bam theo tay kep.
"""
from __future__ import annotations

import time
from typing import Optional, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

ACTION_TAY_KEP = "/tay_kep_controller/follow_joint_trajectory"
KHOP_NGON = ("khop_ngon_trai", "khop_ngon_phai")
VI_TRI_MO = 0.0375     # khe 75 mm
VI_TRI_DONG = 0.0230   # khe 46 mm, sat khoi 45 mm


class LoiTayKep(Exception):
    """Controller tay kep khong nhan lenh hoac khong chay xong."""


class TayKep:
    def __init__(self, node: Node, thoi_gian_dong: float = 0.4,
                 thoi_gian_cho: float = 5.0) -> None:
        self._node = node
        self._thoi_gian_dong = thoi_gian_dong
        self._thoi_gian_cho = thoi_gian_cho
        self._client = ActionClient(node, FollowJointTrajectory, ACTION_TAY_KEP,
                                    callback_group=ReentrantCallbackGroup())

    def cho_san_sang(self, thoi_gian_cho: float = 60.0) -> None:
        if not self._client.wait_for_server(timeout_sec=thoi_gian_cho):
            raise LoiTayKep(f"khong thay action {ACTION_TAY_KEP}; tay_kep_controller da bat chua?")

    def mo(self) -> None:
        self._gui((VI_TRI_MO, VI_TRI_MO), "mo")

    def dong(self) -> None:
        self._gui((VI_TRI_DONG, VI_TRI_DONG), "dong")

    def _gui(self, vi_tri: Sequence[float], nhan: str, so_lan_thu: int = 3) -> None:
        """Gui lenh va cho xong; thu lai neu controller tu choi.

        Thinh thoang controller tu choi mot goal den ngay sau goal truoc (vi
        du luc no vua bat lai sau khi mo phong khoi dong). Gui lai sau 0.3 s
        la qua, khong can bao loi ca skill.
        """
        loi_cuoi: Optional[LoiTayKep] = None
        for _ in range(so_lan_thu):
            try:
                self._gui_mot_lan(vi_tri, nhan)
                return
            except LoiTayKep as loi:
                loi_cuoi = loi
                self._node.get_logger().warn(f"{loi}, gui lai")
                time.sleep(0.3)
        assert loi_cuoi is not None
        raise loi_cuoi

    def _gui_mot_lan(self, vi_tri: Sequence[float], nhan: str) -> None:
        if not self._client.wait_for_server(timeout_sec=self._thoi_gian_cho):
            raise LoiTayKep(f"khong thay action {ACTION_TAY_KEP}")

        muc_tieu = FollowJointTrajectory.Goal()
        muc_tieu.trajectory.joint_names = list(KHOP_NGON)
        diem = JointTrajectoryPoint()
        diem.positions = [float(k) for k in vi_tri]
        diem.velocities = [0.0, 0.0]
        diem.time_from_start = Duration(seconds=self._thoi_gian_dong).to_msg()
        muc_tieu.trajectory.points = [diem]
        muc_tieu.goal_time_tolerance = Duration(seconds=1.0).to_msg()

        tuong_lai = self._client.send_goal_async(muc_tieu)
        rclpy.spin_until_future_complete(self._node, tuong_lai, timeout_sec=self._thoi_gian_cho)
        tay_cam = tuong_lai.result()
        if tay_cam is None or not tay_cam.accepted:
            raise LoiTayKep(f"controller tu choi lenh {nhan} kep")

        tuong_lai_kq = tay_cam.get_result_async()
        rclpy.spin_until_future_complete(self._node, tuong_lai_kq,
                                         timeout_sec=self._thoi_gian_cho)
        ket_qua = tuong_lai_kq.result()
        if ket_qua is None:
            tay_cam.cancel_goal_async()
            raise LoiTayKep(f"lenh {nhan} kep khong xong sau {self._thoi_gian_cho:.0f} s")
        if (ket_qua.status != GoalStatus.STATUS_SUCCEEDED
                or ket_qua.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            raise LoiTayKep(f"lenh {nhan} kep that bai: {ket_qua.result.error_string} "
                            f"(ma {ket_qua.result.error_code})")
