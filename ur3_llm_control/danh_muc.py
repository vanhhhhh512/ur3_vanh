"""Danh muc cac gia tri hop le dung chung cho ca he thong.

Tat ca cac module khac deu tham chieu ve day, nho vay chi co mot noi duy nhat
dinh nghia "the nao la mot skill/object/zone hop le". Validator va prompt gui
cho LLM cung lay tu day nen khong bao gio lech nhau.
"""
from __future__ import annotations

# Ba skill cong khai ma LLM duoc phep chon, kem so tham so bat buoc cua moi cai
SKILL_CONG_KHAI = {
    "home": (),
    "pick": ("object",),
    "place": ("object", "zone"),
}

VAT_THE = ("red_cube", "yellow_cube", "blue_cube")
VUNG_DAT = ("zone_a", "zone_b", "zone_c")

# Trang thai tra ve cua moi skill, dung dung ten trong de bai
THANH_CONG = "SUCCESS"
THAT_BAI = "FAILED"
VAT_THE_KHONG_HOP_LE = "INVALID_OBJECT"
VUNG_KHONG_HOP_LE = "INVALID_ZONE"
LAP_KE_HOACH_THAT_BAI = "PLANNING_FAILED"
