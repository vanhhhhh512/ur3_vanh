"""Dong bo vi tri vat the giua ke hoach cua MoveIt va mo phong Gazebo.

Khi tay kep dong lai, vat duoc gan vao tay kep trong planning scene cua
MoveIt (nen MoveIt van tranh va cham cho ca vat dang cam), dong thoi vi tri
cua vat trong Gazebo duoc cap nhat bam theo diem kep o tan so cao de nhin
thay vat nam giua hai ngon va di theo tay may. Khoi 45 mm khong duoc giu bang
ma sat vi trong Gazebo no rat de truot hoac bi vang khoi ngon.

Ignition Fortress cong bo service /world/<ten>/set_pose. Goi qua CLI `ign`
la cach on dinh nhat tren ban Fortress di kem ROS 2 Humble. Moi lan goi mat
vai chuc ms, nen viec nay chay trong mot luong rieng: neu dat trong timer cua
node, no chan luon executor va moi lenh cho MoveIt deu cham han di.
"""
from __future__ import annotations

import subprocess
import threading
import time
from typing import Optional, Sequence

from rclpy.node import Node


class DongBoGazebo:
    def __init__(self, node: Node, ten_world: str = "ur3_workcell",
                 tan_so: float = 40.0, timeout_ms: int = 300) -> None:
        self._node = node
        self._world = ten_world
        self._timeout_ms = timeout_ms
        self._chu_ky = 1.0 / tan_so
        self._vat_dang_bam: Optional[str] = None
        self._lech_z = 0.0
        self._tf = None  # gan tu ben ngoai de tranh phu thuoc vong
        self._khoa = threading.Lock()
        self._luong = threading.Thread(target=self._vong_lap, daemon=True)
        self._luong.start()

    def gan_nguon_tu_the(self, ham_lay_tu_the) -> None:
        """ham_lay_tu_the() -> (x, y, z) cua tool0 trong he base_link, hoac None."""
        self._tf = ham_lay_tu_the

    # ------------------------------------------------------------- dieu khien
    def bam_theo(self, ten_vat: Optional[str], lech_z: float = 0.0) -> None:
        """Bat che do bam theo tool0 cho mot vat, hoac tat khi truyen None."""
        with self._khoa:
            self._vat_dang_bam = ten_vat
            self._lech_z = lech_z

    def dat_vi_tri(self, ten_vat: str, xyz: Sequence[float]) -> bool:
        """Dat vat ve mot vi tri co dinh trong the gioi Gazebo."""
        with self._khoa:
            return self._goi_set_pose(ten_vat, xyz)

    # ----------------------------------------------------------------- noi bo
    def _vong_lap(self) -> None:
        while True:
            bat_dau = time.monotonic()
            with self._khoa:
                ten, lech = self._vat_dang_bam, self._lech_z
                if ten is not None and self._tf is not None:
                    tu_the = self._tf()
                    if tu_the is not None:
                        x, y, z = tu_the
                        self._goi_set_pose(ten, (x, y, z + lech))
            con_lai = self._chu_ky - (time.monotonic() - bat_dau)
            if con_lai > 0:
                time.sleep(con_lai)

    def _goi_set_pose(self, ten_vat: str, xyz: Sequence[float]) -> bool:
        yeu_cau = (f'name: "{ten_vat}", position: '
                   f'{{x: {xyz[0]:.5f}, y: {xyz[1]:.5f}, z: {xyz[2]:.5f}}}, '
                   f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
        lenh = [
            "ign", "service", "-s", f"/world/{self._world}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", str(self._timeout_ms),
            "--req", yeu_cau,
        ]
        try:
            ket_qua = subprocess.run(lenh, capture_output=True, text=True, timeout=2.0)
        except (subprocess.TimeoutExpired, FileNotFoundError) as loi:
            self._node.get_logger().warn(f"set_pose {ten_vat} that bai: {loi}")
            return False
        if "true" not in ket_qua.stdout.lower():
            self._node.get_logger().debug(
                f"set_pose {ten_vat}: {ket_qua.stdout.strip()} {ket_qua.stderr.strip()}")
            return False
        return True
