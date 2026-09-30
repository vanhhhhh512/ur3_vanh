"""Doc scene.yaml va giu trang thai that cua workcell.

Trang thai nay la nguon su that cho validator va cho cac skill: vat nao con
tren ban, vat nao dang o vung nao, tay may dang cam gi. LLM khong duoc ghi
vao day; no chi de xuat ke hoach, con trang thai do chuong trinh tu quan ly.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

from ur3_llm_control.danh_muc import VAT_THE, VUNG_DAT


class LoiCauHinhScene(Exception):
    """scene.yaml thieu truong hoac sai gia tri."""


@dataclass(frozen=True)
class ThamSoChuyenDong:
    quaternion_gap: Tuple[float, float, float, float]
    dai_tcp: float
    cao_tiep_can: float
    cao_di_chuyen: float
    khe_ho_gap: float
    buoc_cartesian: float
    ty_le_cartesian_toi_thieu: float
    he_so_van_toc: float
    he_so_gia_toc: float
    he_so_van_toc_thang: float
    he_so_gia_toc_thang: float
    tu_the_home: Tuple[float, ...]
    tu_the_trung_chuyen: Tuple[float, ...]


@dataclass
class Workcell:
    khung: str
    link_cong_tac: str
    tam_ban: Tuple[float, float, float]
    kich_thuoc_ban: Tuple[float, float, float]
    cao_mat_ban: float
    canh_vat: float
    vi_tri_vat: Dict[str, List[float]]
    canh_vung: float
    tam_vung: Dict[str, List[float]]
    chuyen_dong: ThamSoChuyenDong
    dang_cam: Optional[str] = None
    vat_trong_vung: Dict[str, str] = field(default_factory=dict)

    # ------------------------------------------------------------- truy van
    def co_vat(self, ten: str) -> bool:
        return ten in self.vi_tri_vat

    def co_vung(self, ten: str) -> bool:
        return ten in self.tam_vung

    def danh_sach_vat(self) -> Tuple[str, ...]:
        return tuple(self.vi_tri_vat)

    def diem_gap(self, ten_vat: str) -> List[float]:
        """Vi tri tool0 khi gap: tam kep trung tam khoi, tool0 cao hon dai_tcp."""
        x, y, z = self.vi_tri_vat[ten_vat]
        cd = self.chuyen_dong
        return [x, y, z + cd.dai_tcp + cd.khe_ho_gap]

    def diem_tha(self, ten_vung: str) -> List[float]:
        """Vi tri tool0 khi tha: khoi vua cham mat vung."""
        x, y, z = self.vi_tri_vat_trong_vung(ten_vung)
        cd = self.chuyen_dong
        return [x, y, z + cd.dai_tcp + cd.khe_ho_gap]

    def vi_tri_vat_trong_vung(self, ten_vung: str) -> List[float]:
        x, y, z = self.tam_vung[ten_vung]
        return [x, y, z + self.canh_vat / 2.0]

    # ---------------------------------------------------------- cap nhat
    def ghi_nhan_gap(self, ten_vat: str) -> None:
        self.dang_cam = ten_vat
        for vung, vat in list(self.vat_trong_vung.items()):
            if vat == ten_vat:
                del self.vat_trong_vung[vung]

    def ghi_nhan_tha(self, ten_vat: str, ten_vung: str) -> None:
        self.vi_tri_vat[ten_vat] = self.vi_tri_vat_trong_vung(ten_vung)
        self.vat_trong_vung[ten_vung] = ten_vat
        self.dang_cam = None

    def vung_bi_chiem(self, ten_vung: str) -> Optional[str]:
        return self.vat_trong_vung.get(ten_vung)

    def tom_tat(self) -> str:
        dong = [f"gripper: {self.dang_cam or 'empty'}"]
        for vat in sorted(self.vi_tri_vat):
            x, y, z = self.vi_tri_vat[vat]
            o_vung = next((v for v, t in self.vat_trong_vung.items() if t == vat), "-")
            dong.append(f"  {vat:12s} ({x:+.3f}, {y:+.3f}, {z:+.3f})  zone: {o_vung}")
        return "\n".join(dong)


def _bo_ba(gia_tri, ten: str) -> Tuple[float, float, float]:
    if not isinstance(gia_tri, (list, tuple)) or len(gia_tri) != 3:
        raise LoiCauHinhScene(f"{ten} phai la danh sach 3 so")
    return tuple(float(k) for k in gia_tri)  # type: ignore[return-value]


def doc_workcell(duong_dan: str) -> Workcell:
    with open(duong_dan, "r", encoding="utf-8") as tep:
        tai_lieu = yaml.safe_load(tep) or {}

    try:
        ban = tai_lieu["ban_thao_tac"]
        vat = tai_lieu["vat_the"]
        vung = tai_lieu["vung_dat"]
        cd = tai_lieu["chuyen_dong"]
    except KeyError as loi:
        raise LoiCauHinhScene(f"scene.yaml thieu muc {loi}") from loi

    vi_tri_vat = {str(k): [float(x) for x in v] for k, v in vat["vi_tri_ban_dau"].items()}
    if set(vi_tri_vat) != set(VAT_THE):
        raise LoiCauHinhScene(f"scene.yaml phai khai dung ba vat the {VAT_THE}")

    tam_vung = {str(k): [float(x) for x in v] for k, v in vung["tam"].items()}
    if set(tam_vung) != set(VUNG_DAT):
        raise LoiCauHinhScene(f"scene.yaml phai khai dung ba vung {VUNG_DAT}")

    chuyen_dong = ThamSoChuyenDong(
        quaternion_gap=tuple(float(k) for k in cd["quaternion_gap"]),  # type: ignore[arg-type]
        dai_tcp=float(cd["dai_tcp"]),
        cao_tiep_can=float(cd["cao_tiep_can"]),
        cao_di_chuyen=float(cd["cao_di_chuyen"]),
        khe_ho_gap=float(cd.get("khe_ho_gap", 0.005)),
        buoc_cartesian=float(cd["buoc_cartesian"]),
        ty_le_cartesian_toi_thieu=float(cd["ty_le_cartesian_toi_thieu"]),
        he_so_van_toc=float(cd["he_so_van_toc"]),
        he_so_gia_toc=float(cd["he_so_gia_toc"]),
        he_so_van_toc_thang=float(cd.get("he_so_van_toc_thang", cd["he_so_van_toc"])),
        he_so_gia_toc_thang=float(cd.get("he_so_gia_toc_thang", cd["he_so_gia_toc"])),
        tu_the_home=tuple(float(k) for k in cd["tu_the_home"]),
        tu_the_trung_chuyen=tuple(float(k) for k in cd.get(
            "tu_the_trung_chuyen", cd["tu_the_home"])),
    )

    return Workcell(
        khung=str(tai_lieu.get("khung_quy_chieu", "base_link")),
        link_cong_tac=str(tai_lieu.get("link_cong_tac", "tool0")),
        tam_ban=_bo_ba(ban["tam"], "ban_thao_tac.tam"),
        kich_thuoc_ban=_bo_ba(ban["kich_thuoc"], "ban_thao_tac.kich_thuoc"),
        cao_mat_ban=float(ban.get("cao_mat_ban", 0.15)),
        canh_vat=float(vat["canh"]),
        vi_tri_vat=vi_tri_vat,
        canh_vung=float(vung["canh"]),
        tam_vung=tam_vung,
        chuyen_dong=chuyen_dong,
    )
