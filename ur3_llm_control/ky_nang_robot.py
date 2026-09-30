"""Ba robot skill cong khai: home, pick, place.

Day la tang duy nhat duoc phep dieu khien robot. LLM khong bao gio goi thang
vao MoveIt: no chi chon ten skill va tham so, con moi quy dao deu do cac ham
o day dung nen roi giao cho MoveIt lap ke hoach co tranh va cham.

Robot dung tay kep song song hai ngon (urdf/tay_kep_hai_ngon.xacro):
    pick : mo kep -> toi tren vat -> ha thang -> dong kep -> gan vat -> nhac len
    place: mang thang toi tren vung -> ha thang -> mo kep -> tha vat -> rut len

Moi skill tra ve mot ma trang thai trong danh_muc (SUCCESS, FAILED,
INVALID_OBJECT, INVALID_ZONE, PLANNING_FAILED) kem mot dong giai thich.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

from geometry_msgs.msg import Pose

from ur3_llm_control.danh_muc import (
    LAP_KE_HOACH_THAT_BAI,
    THANH_CONG,
    THAT_BAI,
    VAT_THE_KHONG_HOP_LE,
    VUNG_KHONG_HOP_LE,
)
from ur3_llm_control.giao_tiep_moveit import GiaoTiepMoveIt, LoiMoveIt, tao_pose
from ur3_llm_control.mo_hinh_workcell import Workcell
from ur3_llm_control.tay_kep import LoiTayKep, TayKep

# Cac link duoc phep cham vao vat dang kep: hai ngon om sat khoi, than kep va
# co tay nam ngay tren no.
LINK_DUOC_CHAM = ("tool0", "flange", "wrist_3_link",
                  "tay_kep_than", "ngon_trai", "ngon_phai", "tay_kep_tcp")


@dataclass
class KetQuaKyNang:
    trang_thai: str
    thong_diep: str = ""

    @property
    def thanh_cong(self) -> bool:
        return self.trang_thai == THANH_CONG


class KyNangRobot:
    def __init__(self, moveit: GiaoTiepMoveIt, workcell: Workcell,
                 tay_kep: Optional[TayKep] = None,
                 ghi_log: Optional[Callable[[str], None]] = None,
                 dong_bo_gazebo: Optional[Callable[[str, Sequence[float]], None]] = None,
                 bam_theo_tay: Optional[Callable[[Optional[str]], None]] = None) -> None:
        self._moveit = moveit
        self._wc = workcell
        self._kep = tay_kep
        self._log = ghi_log or (lambda _: None)
        self._dong_bo = dong_bo_gazebo          # dat lai vi tri vat trong Gazebo
        self._bam_theo = bam_theo_tay           # bat/tat che do vat bam theo tay kep

    # ------------------------------------------------------------ tien ich
    def _tu_the(self, xyz: Sequence[float]) -> Pose:
        return tao_pose(xyz, self._wc.chuyen_dong.quaternion_gap)

    def _buoc(self, nhan: str, bat_dau: float) -> None:
        self._log(f"    {nhan}: done ({time.monotonic() - bat_dau:.1f} s)")

    def _di_thang(self, diem: Sequence[float], nhan: str) -> None:
        """Ha xuong hoac nhac len theo duong thang dung.

        Doan nay chay voi tranh va cham tat. Luc ha, hai ngon kep om hai ben
        khoi; luc nhac, khoi vua gan vao tay kep van dang cham mat ban. Bat kiem
        va cham thi ca hai truong hop deu bi coi la va cham va tra ve 0%. Doan
        chi dai vai centimet theo phuong thang dung, ngay tren vat/vung da
        biet chac la trong, va van bi rang buoc boi gioi han khop.
        """
        cd = self._wc.chuyen_dong
        bat_dau = time.monotonic()
        # Giu nguyen huong tay kep hien tai (co the da xoay boi so 90 do so voi
        # quaternion_gap, xem quy_co_tay_ve_giua) de wrist_3 dung yen.
        dich = self._tu_the(diem)
        dich.orientation = self._moveit.huong_tool_hien_tai()
        self._moveit.di_theo_duong_thang(
            [dich], cd.buoc_cartesian, cd.ty_le_cartesian_toi_thieu,
            tranh_va_cham=False,
            he_so_van_toc=cd.he_so_van_toc_thang, he_so_gia_toc=cd.he_so_gia_toc_thang)
        self._buoc(nhan, bat_dau)

    def _di_toi(self, xyz: Sequence[float], nhan: str) -> None:
        """Di tu do (co tranh va cham) toi mot diem treo tren vat hoac vung."""
        cd = self._wc.chuyen_dong
        bat_dau = time.monotonic()
        try:
            self._moveit.di_toi_diem_nhanh(self._tu_the(xyz), cd.he_so_van_toc, cd.he_so_gia_toc)
        except LoiMoveIt as loi:
            # Khong di thang duoc thi ghe qua tu the trung chuyen roi thu lai.
            self._log(f"    {nhan}: blocked, routing via transit pose ({loi})")
            self._moveit.di_toi_khop(cd.tu_the_trung_chuyen,
                                     cd.he_so_van_toc, cd.he_so_gia_toc)
            self._moveit.di_toi_diem_nhanh(self._tu_the(xyz), cd.he_so_van_toc, cd.he_so_gia_toc)
        self._buoc(nhan, bat_dau)

    def _mo_kep(self) -> None:
        if self._kep is not None:
            self._kep.mo()
            self._log("    gripper open")

    def _dong_kep(self) -> None:
        if self._kep is not None:
            self._kep.dong()
            self._log("    gripper closed")

    def _diem_treo(self, diem: Sequence[float]) -> List[float]:
        return [diem[0], diem[1], diem[2] + self._wc.chuyen_dong.cao_tiep_can]

    # --------------------------------------------------------------- home
    def home(self) -> KetQuaKyNang:
        if self._wc.dang_cam is not None:
            return KetQuaKyNang(THAT_BAI, f"tay may con dang cam {self._wc.dang_cam}")
        cd = self._wc.chuyen_dong
        bat_dau = time.monotonic()
        try:
            self._moveit.di_toi_khop(cd.tu_the_home, cd.he_so_van_toc, cd.he_so_gia_toc)
        except LoiMoveIt as loi:
            return KetQuaKyNang(LAP_KE_HOACH_THAT_BAI, str(loi))
        self._buoc("go home", bat_dau)
        return KetQuaKyNang(THANH_CONG)

    # --------------------------------------------------------------- pick
    def pick(self, ten_vat: str) -> KetQuaKyNang:
        if not self._wc.co_vat(ten_vat):
            return KetQuaKyNang(VAT_THE_KHONG_HOP_LE, f"{ten_vat} khong co trong workcell")
        if self._wc.dang_cam is not None:
            return KetQuaKyNang(THAT_BAI, f"tay may dang cam {self._wc.dang_cam}")

        diem_gap = self._wc.diem_gap(ten_vat)
        treo = self._diem_treo(diem_gap)
        try:
            self._mo_kep()
            self._di_toi(treo, f"approach above {ten_vat}")
            # Xoa vat can truoc khi ha: hai ngon kep se om hai ben khoi
            self._moveit.xoa_hop(ten_vat)
            self._di_thang(diem_gap, "descend to grasp")

            self._dong_kep()
            # Tam khoi nam o diem kep, cach tool0 dai_tcp theo truc +z cua tool0
            self._moveit.gan_vat(ten_vat, self._wc.canh_vat,
                                 self._wc.chuyen_dong.dai_tcp + self._wc.chuyen_dong.khe_ho_gap,
                                 LINK_DUOC_CHAM)
            # Ghi nhan truoc, vi bam_theo ve lai marker theo trang thai workcell
            self._wc.ghi_nhan_gap(ten_vat)
            if self._bam_theo is not None:
                self._bam_theo(ten_vat)
            self._log(f"    grasped {ten_vat}")

            self._di_thang(treo, "lift object")
        except (LoiMoveIt, LoiTayKep) as loi:
            return KetQuaKyNang(LAP_KE_HOACH_THAT_BAI, str(loi))
        return KetQuaKyNang(THANH_CONG)

    # -------------------------------------------------------------- place
    def place(self, ten_vat: str, ten_vung: str) -> KetQuaKyNang:
        if not self._wc.co_vat(ten_vat):
            return KetQuaKyNang(VAT_THE_KHONG_HOP_LE, f"{ten_vat} khong co trong workcell")
        if not self._wc.co_vung(ten_vung):
            return KetQuaKyNang(VUNG_KHONG_HOP_LE, f"{ten_vung} khong co trong workcell")
        if self._wc.dang_cam != ten_vat:
            return KetQuaKyNang(THAT_BAI,
                                f"tay may dang cam {self._wc.dang_cam or 'khong gi ca'}, "
                                f"khong phai {ten_vat}")
        chu_cu = self._wc.vung_bi_chiem(ten_vung)
        if chu_cu is not None and chu_cu != ten_vat:
            return KetQuaKyNang(THAT_BAI, f"{ten_vung} da bi {chu_cu} chiem")

        diem_tha = self._wc.diem_tha(ten_vung)
        treo = self._diem_treo(diem_tha)
        try:
            self._di_toi(treo, f"carry {ten_vat} to {ten_vung}")
            self._di_thang(diem_tha, "descend to release")

            self._mo_kep()
            # Go khoi khoi tay kep roi dat lai vao the gioi
            self._moveit.tha_vat(ten_vat)
            self._wc.ghi_nhan_tha(ten_vat, ten_vung)
            if self._bam_theo is not None:
                self._bam_theo(None)
            if self._dong_bo is not None:
                self._dong_bo(ten_vat, self._wc.vi_tri_vat[ten_vat])
            self._log(f"    released {ten_vat} at {ten_vung}")

            self._di_thang(treo, "retreat")
            # Chi dua vat can tro lai the gioi SAU khi tay kep da rut len,
            # neu khong tu the hien tai bi coi la dang va cham voi chinh no.
            self._moveit.them_hop(ten_vat, self._wc.vi_tri_vat[ten_vat],
                                  (self._wc.canh_vat,) * 3)
        except (LoiMoveIt, LoiTayKep) as loi:
            return KetQuaKyNang(LAP_KE_HOACH_THAT_BAI, str(loi))
        return KetQuaKyNang(THANH_CONG)
