"""Ve workcell va ke hoach dang thuc thi len RViz.

Muc dich la de nguoi xem video hieu duoc chuong trinh dang lam gi: ba vung
dat duoc to mau, vat the duoc danh dau, va ke hoach hien tai duoc in thanh
chu ngay trong khung nhin, kem mui ten chi buoc dang chay.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from ur3_llm_control.mo_hinh_workcell import Workcell

MAU_VAT = {
    "red_cube": (0.85, 0.10, 0.10),
    "yellow_cube": (0.95, 0.80, 0.05),
    "blue_cube": (0.10, 0.25, 0.85),
}
MAU_VUNG = {
    "zone_a": (0.90, 0.25, 0.25),
    "zone_b": (0.25, 0.35, 0.90),
    "zone_c": (0.95, 0.85, 0.25),
}


def _mau(rgb: Sequence[float], a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


class HienThiRviz:
    def __init__(self, node: Node, workcell: Workcell,
                 chu_de: str = "/workcell_markers") -> None:
        self._node = node
        self._wc = workcell
        # Transient local de RViz mo sau van nhan duoc marker cu
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = node.create_publisher(MarkerArray, chu_de, qos)
        self._ke_hoach: List[str] = []
        self._buoc_hien_tai: Optional[int] = None

    # ------------------------------------------------------------- cap nhat
    def dat_ke_hoach(self, cac_dong: Sequence[str]) -> None:
        self._ke_hoach = list(cac_dong)
        self._buoc_hien_tai = None
        self.ve()

    def dat_buoc(self, chi_so: Optional[int]) -> None:
        self._buoc_hien_tai = chi_so
        self.ve()

    # ---------------------------------------------------------------- ve
    def _header(self) -> Header:
        return Header(frame_id=self._wc.khung, stamp=self._node.get_clock().now().to_msg())

    def _khung_marker(self, ns: str, chi_so: int, kieu: int) -> Marker:
        m = Marker()
        m.header = self._header()
        m.ns = ns
        m.id = chi_so
        m.type = kieu
        m.action = Marker.ADD
        m.lifetime = Duration(sec=0)
        m.pose.orientation.w = 1.0
        return m

    def ve(self) -> None:
        mang = MarkerArray()
        chi_so = 0

        # Ban thao tac: khoi go tu san len mat ban (trong MoveIt ban chi la
        # tam mong, nen ve rieng cho nguoi xem thay ro)
        ban = self._khung_marker("ban", chi_so, Marker.CUBE)
        chi_so += 1
        cao = self._wc.cao_mat_ban
        ban.pose.position = Point(x=self._wc.tam_ban[0], y=self._wc.tam_ban[1], z=cao / 2.0)
        ban.scale = Vector3(x=self._wc.kich_thuoc_ban[0], y=self._wc.kich_thuoc_ban[1], z=cao)
        ban.color = _mau((0.72, 0.60, 0.42), 1.0)
        mang.markers.append(ban)

        # Ba vung dat
        for ten, tam in self._wc.tam_vung.items():
            m = self._khung_marker("vung_dat", chi_so, Marker.CUBE)
            chi_so += 1
            m.pose.position = Point(x=tam[0], y=tam[1], z=tam[2] + 0.002)
            m.scale = Vector3(x=self._wc.canh_vung, y=self._wc.canh_vung, z=0.004)
            m.color = _mau(MAU_VUNG[ten], 0.55)
            mang.markers.append(m)

            chu = self._khung_marker("ten_vung", chi_so, Marker.TEXT_VIEW_FACING)
            chi_so += 1
            chu.pose.position = Point(x=tam[0], y=tam[1], z=tam[2] + 0.05)
            chu.scale = Vector3(x=0.0, y=0.0, z=0.028)
            chu.color = _mau((0.1, 0.1, 0.1), 0.95)
            chu.text = ten.replace("zone_", "ZONE-").upper()
            mang.markers.append(chu)

        # Vi tri vat the theo trang thai chuong trinh dang giu
        for ten, vi_tri in self._wc.vi_tri_vat.items():
            m = self._khung_marker("vat_the", chi_so, Marker.CUBE)
            chi_so += 1
            m.pose.position = Point(x=vi_tri[0], y=vi_tri[1], z=vi_tri[2])
            canh = self._wc.canh_vat
            m.scale = Vector3(x=canh, y=canh, z=canh)
            dang_cam = self._wc.dang_cam == ten
            # Vat dang kep da duoc Planning Scene ve gan tren tay kep, nen an
            # marker o vi tri cu di de khong thanh "bong ma" tren ban.
            m.color = _mau(MAU_VAT[ten], 0.0 if dang_cam else 0.9)
            mang.markers.append(m)

        # Ke hoach dang chay, in thanh nhieu dong chu phia tren workcell
        for thu_tu, dong in enumerate(self._ke_hoach):
            chu = self._khung_marker("ke_hoach", chi_so, Marker.TEXT_VIEW_FACING)
            chi_so += 1
            chu.pose.position = Point(x=0.12, y=-0.42, z=0.46 - 0.05 * thu_tu)
            chu.scale = Vector3(x=0.0, y=0.0, z=0.035)
            dang_chay = self._buoc_hien_tai == thu_tu
            chu.color = _mau((1.0, 0.45, 0.0) if dang_chay else (0.15, 0.15, 0.15), 1.0)
            chu.text = ("-> " if dang_chay else "   ") + dong
            mang.markers.append(chu)

        self._pub.publish(mang)
