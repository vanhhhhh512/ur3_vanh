"""Sinh nhiem vu ca nhan tu ma sinh vien.

De bai: P = XX mod 6, voi XX la hai chu so cuoi cua MSSV. Moi gia tri P ung
voi mot cach gan vat the cho ba vung. Module nay khong viet cung ket qua cua
mot sinh vien nao ma tra cuu bang quy uoc trong config/student_config.yaml,
nen doi MSSV la doi nhiem vu ngay.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping

import yaml

from ur3_llm_control.danh_muc import VAT_THE, VUNG_DAT


class LoiCauHinhSinhVien(Exception):
    """Cau hinh sinh vien thieu truong hoac sai gia tri."""


@dataclass(frozen=True)
class NhiemVuCaNhan:
    ten_sinh_vien: str
    ma_sinh_vien: str
    hai_chu_so_cuoi: int
    p: int
    phan_cong: Dict[str, str]        # zone -> object

    def ban_giao_theo_thu_tu(self):
        """Tra ve [(zone, object), ...] theo dung thu tu zone_a, zone_b, zone_c."""
        return [(vung, self.phan_cong[vung]) for vung in VUNG_DAT]

    def mo_ta_bang(self) -> str:
        """Bang phan cong dang van ban, dung de in ra terminal va gui cho LLM."""
        dong = [f"{vung} -> {self.phan_cong[vung]}" for vung in VUNG_DAT]
        return "\n".join(dong)


def _lay_hai_chu_so_cuoi(ma_sinh_vien: str) -> int:
    chu_so = "".join(k for k in ma_sinh_vien if k.isdigit())
    if len(chu_so) < 2:
        raise LoiCauHinhSinhVien(f"ma sinh vien phai co it nhat hai chu so: {ma_sinh_vien!r}")
    return int(chu_so[-2:])


def doc_nhiem_vu(duong_dan_cau_hinh: str) -> NhiemVuCaNhan:
    """Doc student_config.yaml va tinh ra nhiem vu ca nhan."""
    with open(duong_dan_cau_hinh, "r", encoding="utf-8") as tep:
        tai_lieu = yaml.safe_load(tep) or {}

    ten = str(tai_lieu.get("student_name", "")).strip()
    ma = str(tai_lieu.get("student_id", "")).strip()
    if not ten or not ma:
        raise LoiCauHinhSinhVien("thieu student_name hoac student_id")

    bang = tai_lieu.get("phan_cong_theo_p")
    if not isinstance(bang, Mapping):
        raise LoiCauHinhSinhVien("thieu bang phan_cong_theo_p")

    hai_chu_so = _lay_hai_chu_so_cuoi(ma)
    p = hai_chu_so % 6

    phan_cong_tho = bang.get(p)
    if not isinstance(phan_cong_tho, Mapping):
        raise LoiCauHinhSinhVien(f"bang phan_cong_theo_p thieu truong hop P = {p}")

    phan_cong = {str(k): str(v) for k, v in phan_cong_tho.items()}
    if set(phan_cong) != set(VUNG_DAT):
        raise LoiCauHinhSinhVien(f"P = {p} phai phan cong dung ba vung {VUNG_DAT}")
    if sorted(phan_cong.values()) != sorted(VAT_THE):
        raise LoiCauHinhSinhVien(f"P = {p} phai dung moi vat the dung mot lan")

    return NhiemVuCaNhan(ten, ma, hai_chu_so, p, phan_cong)
