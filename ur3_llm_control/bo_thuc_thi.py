"""Chay tung buoc cua ke hoach da qua kiem tra va in bao cao ra terminal.

Bo thuc thi la ranh gioi cuoi cung: no chi biet ba skill cong khai va tu choi
bat cu thu gi khac, ke ca khi validator da bo sot. Thuc thi dung lai ngay o
buoc dau tien that bai, khong chay tiep cac buoc con lai.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

from ur3_llm_control.danh_muc import THANH_CONG, THAT_BAI
from ur3_llm_control.ky_nang_robot import KetQuaKyNang, KyNangRobot


def mo_ta_buoc(buoc: Dict[str, str]) -> str:
    """Dang chuoi de in: pick(red_cube), place(red_cube, zone_b), home()."""
    skill = buoc["skill"]
    if skill == "home":
        return "home()"
    if skill == "pick":
        return f"pick({buoc['object']})"
    return f"place({buoc['object']}, {buoc['zone']})"


@dataclass
class KetQuaThucThi:
    thanh_cong: bool
    cac_dong: List[str] = field(default_factory=list)
    buoc_hong: Optional[str] = None
    trang_thai_hong: Optional[str] = None


class BoThucThi:
    def __init__(self, ky_nang: KyNangRobot,
                 ghi_log: Optional[Callable[[str], None]] = None,
                 cho_nut_bam: Optional[Callable[[str], None]] = None) -> None:
        self._ky_nang = ky_nang
        self._log = ghi_log or print
        self._cho_nut = cho_nut_bam

    def _goi_skill(self, buoc: Dict[str, str]) -> KetQuaKyNang:
        skill = buoc["skill"]
        if skill == "home":
            return self._ky_nang.home()
        if skill == "pick":
            return self._ky_nang.pick(buoc["object"])
        if skill == "place":
            return self._ky_nang.place(buoc["object"], buoc["zone"])
        return KetQuaKyNang(THAT_BAI, f"skill {skill!r} khong duoc ho tro o tang thuc thi")

    def chay(self, cac_buoc: Sequence[Dict[str, str]]) -> KetQuaThucThi:
        ket_qua = KetQuaThucThi(thanh_cong=True)
        rong = max((len(mo_ta_buoc(b)) for b in cac_buoc), default=10)

        self._log("")
        self._log("EXECUTION:")
        for buoc in cac_buoc:
            nhan = mo_ta_buoc(buoc)
            if self._cho_nut is not None:
                self._cho_nut(nhan)

            kq = self._goi_skill(buoc)
            dau_cham = "." * max(1, rong + 4 - len(nhan))
            dong = f"  {nhan} {dau_cham} {kq.trang_thai}"
            if kq.thong_diep:
                dong += f"   ({kq.thong_diep})"
            self._log(dong)
            ket_qua.cac_dong.append(dong)

            if kq.trang_thai != THANH_CONG:
                ket_qua.thanh_cong = False
                ket_qua.buoc_hong = nhan
                ket_qua.trang_thai_hong = kq.trang_thai
                self._log("")
                self._log(f"TASK FAILED at {nhan} ({kq.trang_thai})")
                return ket_qua

        self._log("")
        self._log("TASK SUCCESS")
        return ket_qua
