"""Kiem tra ke hoach do LLM sinh ra truoc khi cho phep robot chay.

Nguyen tac fail-closed: chi nhung ke hoach vuot qua TOAN BO cac phep kiem tra
duoi day moi duoc thuc thi. Bat ky dau hieu bat thuong nao cung lam ca ke
hoach bi tu choi, thay vi bo qua mot buoc roi chay tiep.

LLM la nguon KHONG dang tin. Module nay khong tin bat cu gia tri nao no tra
ve, ke ca khi JSON dung dinh dang.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Sequence

from ur3_llm_control.danh_muc import SKILL_CONG_KHAI, VAT_THE, VUNG_DAT

SO_BUOC_TOI_DA = 24


@dataclass
class KetQuaKiemTra:
    hop_le: bool
    cac_buoc: List[Dict[str, str]] = field(default_factory=list)
    ly_do: List[str] = field(default_factory=list)

    def bao_cao(self) -> str:
        if self.hop_le:
            return f"ke hoach hop le, {len(self.cac_buoc)} buoc"
        return "tu choi: " + "; ".join(self.ly_do)


def _loi(ly_do: str) -> KetQuaKiemTra:
    return KetQuaKiemTra(hop_le=False, ly_do=[ly_do])


def kiem_tra(ke_hoach_tho: Any, vat_the_tren_ban: Sequence[str] = VAT_THE) -> KetQuaKiemTra:
    """Kiem tra ke hoach tho lay tu LLM.

    vat_the_tren_ban cho phep goi han theo trang thai that cua workcell: neu
    mot vat khong ton tai trong scene thi ke hoach nhac den no cung bi tu choi.
    """
    if not isinstance(ke_hoach_tho, dict):
        return _loi("ket qua cua LLM khong phai mot doi tuong JSON")

    if "plan" not in ke_hoach_tho:
        return _loi("thieu truong 'plan'")

    cac_buoc_tho = ke_hoach_tho["plan"]
    if not isinstance(cac_buoc_tho, list):
        return _loi("truong 'plan' phai la mot danh sach")

    if not cac_buoc_tho:
        return _loi("LLM tra ve ke hoach rong, khong du thong tin de thuc thi")

    if len(cac_buoc_tho) > SO_BUOC_TOI_DA:
        return _loi(f"ke hoach dai bat thuong ({len(cac_buoc_tho)} buoc, toi da {SO_BUOC_TOI_DA})")

    ly_do: List[str] = []
    cac_buoc: List[Dict[str, str]] = []

    # Mo phong trang thai gap/tha de bat cac ke hoach sai logic
    dang_cam: str | None = None
    vung_da_chiem: Dict[str, str] = {}

    for chi_so, buoc_tho in enumerate(cac_buoc_tho, start=1):
        nhan = f"buoc {chi_so}"

        if not isinstance(buoc_tho, dict):
            ly_do.append(f"{nhan}: khong phai doi tuong JSON")
            continue

        skill = buoc_tho.get("skill")
        if not isinstance(skill, str) or skill not in SKILL_CONG_KHAI:
            ly_do.append(f"{nhan}: skill {skill!r} khong nam trong danh sach cho phep")
            continue

        truong_bat_buoc = SKILL_CONG_KHAI[skill]
        thua = set(buoc_tho) - {"skill"} - set(truong_bat_buoc)
        if thua:
            ly_do.append(f"{nhan}: skill {skill!r} co tham so la {sorted(thua)}")
            continue

        thieu = [t for t in truong_bat_buoc if t not in buoc_tho]
        if thieu:
            ly_do.append(f"{nhan}: skill {skill!r} thieu tham so {thieu}")
            continue

        buoc: Dict[str, str] = {"skill": skill}

        if "object" in truong_bat_buoc:
            vat = buoc_tho.get("object")
            if not isinstance(vat, str) or vat not in VAT_THE:
                ly_do.append(f"{nhan}: object {vat!r} khong hop le")
                continue
            if vat not in vat_the_tren_ban:
                ly_do.append(f"{nhan}: object {vat!r} khong co trong workcell")
                continue
            buoc["object"] = vat

        if "zone" in truong_bat_buoc:
            vung = buoc_tho.get("zone")
            if not isinstance(vung, str) or vung not in VUNG_DAT:
                ly_do.append(f"{nhan}: zone {vung!r} khong hop le")
                continue
            buoc["zone"] = vung

        # Kiem tra tinh nhat quan cua chuoi thao tac
        if skill == "pick":
            if dang_cam is not None:
                ly_do.append(f"{nhan}: pick khi tay may dang cam {dang_cam}")
                continue
            dang_cam = buoc["object"]
        elif skill == "place":
            if dang_cam is None:
                ly_do.append(f"{nhan}: place khi tay may dang trong")
                continue
            if dang_cam != buoc["object"]:
                ly_do.append(f"{nhan}: place {buoc['object']} nhung tay may dang cam {dang_cam}")
                continue
            chu_cu = vung_da_chiem.get(buoc["zone"])
            if chu_cu is not None:
                ly_do.append(f"{nhan}: {buoc['zone']} da bi {chu_cu} chiem")
                continue
            vung_da_chiem[buoc["zone"]] = buoc["object"]
            dang_cam = None
        elif skill == "home":
            if dang_cam is not None:
                ly_do.append(f"{nhan}: home khi tay may con cam {dang_cam}")
                continue

        cac_buoc.append(buoc)

    if dang_cam is not None:
        ly_do.append(f"ke hoach ket thuc khi tay may con cam {dang_cam}")

    if ly_do:
        return KetQuaKiemTra(hop_le=False, ly_do=ly_do)

    return KetQuaKiemTra(hop_le=True, cac_buoc=cac_buoc)
