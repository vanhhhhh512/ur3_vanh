"""Goi LLM qua 9Router de bien cau lenh ngon ngu tu nhien thanh ke hoach JSON.

9Router la mot gateway chay cuc bo, mo endpoint tuong thich OpenAI Chat
Completions o cong 20128. Nho vay module nay chi can SDK openai va khong bi
rang buoc vao mot nha cung cap LLM cu the nao.

Ranh gioi an toan: LLM CHI duoc chon va sap xep skill. Moi gia tri no tra ve
deu di qua kiem_tra_ke_hoach truoc khi robot nhuc nhich. Module nay khong tu
sua chua ke hoach sai, cung khong doan y nguoi dung.
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional

import yaml

NHAN_BANG_PHAN_CONG = "ASSIGNMENT TABLE"


class LoiCauHinhLLM(Exception):
    """Thieu bien moi truong hoac truong cau hinh bat buoc."""


class LoiGoiLLM(Exception):
    """Goi LLM that bai hoac ket qua khong phai JSON doc duoc."""


@dataclass(frozen=True)
class CauHinhLLM:
    base_url: str
    api_key: str
    model: str
    timeout_giay: float
    nhiet_do: float
    so_lan_thu_lai: int

    def che_khoa(self) -> str:
        """Chuoi mo ta an toan de in ra log, khong lo khoa."""
        duoi = self.api_key[-4:] if len(self.api_key) >= 4 else "????"
        return f"{self.base_url} | model={self.model} | key=***{duoi}"


def doc_cau_hinh(duong_dan: str, moi_truong: Optional[Mapping[str, str]] = None) -> CauHinhLLM:
    moi_truong = os.environ if moi_truong is None else moi_truong
    with open(duong_dan, "r", encoding="utf-8") as tep:
        tai_lieu = yaml.safe_load(tep) or {}

    def _bat_buoc(khoa: str) -> str:
        gia_tri = tai_lieu.get(khoa)
        if not isinstance(gia_tri, str) or not gia_tri.strip():
            raise LoiCauHinhLLM(f"config LLM thieu truong {khoa!r}")
        return gia_tri.strip()

    ten_bien_url = _bat_buoc("bien_base_url")
    ten_bien_key = _bat_buoc("bien_api_key")
    ten_bien_model = _bat_buoc("bien_model")

    base_url = moi_truong.get(ten_bien_url) or _bat_buoc("base_url_mac_dinh")
    model = moi_truong.get(ten_bien_model) or _bat_buoc("model_mac_dinh")

    api_key = moi_truong.get(ten_bien_key, "")
    if not api_key.strip():
        raise LoiCauHinhLLM(
            f"chua dat bien moi truong {ten_bien_key}. Lay API key trong dashboard "
            f"9Router tai {base_url.rsplit('/v1', 1)[0]} roi export bien nay.")

    return CauHinhLLM(
        base_url=base_url,
        api_key=api_key.strip(),
        model=model,
        timeout_giay=float(tai_lieu.get("timeout_giay", 45.0)),
        nhiet_do=float(tai_lieu.get("nhiet_do", 0.0)),
        so_lan_thu_lai=int(tai_lieu.get("so_lan_thu_lai", 2)),
    )


def _tao_client_openai(**tham_so: Any) -> Any:
    try:
        from openai import OpenAI
    except ImportError as loi:  # pragma: no cover - phu thuoc moi truong
        raise LoiGoiLLM("chua cai SDK openai: python3 -m pip install 'openai>=1.0,<2.0'") from loi
    return OpenAI(**tham_so)


def _bo_rao_markdown(van_ban: str) -> str:
    """Mot so model boc JSON trong ```json ... ```; go rao truoc khi parse."""
    van_ban = van_ban.strip()
    if van_ban.startswith("```"):
        van_ban = re.sub(r"^```[a-zA-Z]*\s*", "", van_ban)
        van_ban = re.sub(r"\s*```$", "", van_ban)
    return van_ban.strip()


def _trich_json(van_ban: str) -> Any:
    van_ban = _bo_rao_markdown(van_ban)
    try:
        return json.loads(van_ban)
    except json.JSONDecodeError:
        pass
    # Model co the kem loi giai thich; lay khoi ngoac ngoai cung dau tien
    mo = van_ban.find("{")
    dong = van_ban.rfind("}")
    if mo == -1 or dong <= mo:
        raise LoiGoiLLM(f"ket qua cua LLM khong chua JSON: {van_ban[:200]!r}")
    return json.loads(van_ban[mo:dong + 1])


class BoLapKeHoachLLM:
    """Boc mot lan goi Chat Completions thanh mot ham duy nhat."""

    def __init__(self, cau_hinh: CauHinhLLM, system_prompt: str,
                 tao_client: Optional[Callable[..., Any]] = None) -> None:
        self._cau_hinh = cau_hinh
        self._system_prompt = system_prompt
        self._tao_client = tao_client or _tao_client_openai
        self._client: Any = None

    @property
    def cau_hinh(self) -> CauHinhLLM:
        return self._cau_hinh

    def _lay_client(self) -> Any:
        if self._client is None:
            self._client = self._tao_client(
                base_url=self._cau_hinh.base_url,
                api_key=self._cau_hinh.api_key,
                timeout=self._cau_hinh.timeout_giay,
            )
        return self._client

    def _dung_tin_nhan(self, cau_lenh: str, bang_phan_cong: Optional[str]) -> List[Dict[str, str]]:
        """Dung hoi thoai gui cho LLM.

        Bang phan cong duoc noi thang vao cuoi system prompt chu khong tach
        thanh mot system message rieng: cac model nho hay bo qua system message
        thu hai, dan toi viec coi yeu cau "sap xep theo MSSV" la thieu thong
        tin va tra ve ke hoach rong.
        """
        noi_dung = self._system_prompt
        if bang_phan_cong:
            # Bang nay do chuong trinh tinh tu MSSV, khong phai do LLM suy ra.
            noi_dung = (f"{noi_dung}\n\n{NHAN_BANG_PHAN_CONG} (the real one for "
                        f"this session, use these exact objects):\n{bang_phan_cong}\n")
        return [
            {"role": "system", "content": noi_dung},
            {"role": "user", "content": cau_lenh},
        ]

    def lap_ke_hoach(self, cau_lenh: str, bang_phan_cong: Optional[str] = None) -> Dict[str, Any]:
        """Tra ve ke hoach tho dang dict. Chua duoc kiem tra, chua duoc tin."""
        if not cau_lenh or not cau_lenh.strip():
            raise LoiGoiLLM("cau lenh rong")

        tin_nhan = self._dung_tin_nhan(cau_lenh.strip(), bang_phan_cong)
        client = self._lay_client()
        loi_cuoi: Optional[Exception] = None

        for _ in range(max(1, self._cau_hinh.so_lan_thu_lai + 1)):
            try:
                phan_hoi = client.chat.completions.create(
                    model=self._cau_hinh.model,
                    messages=tin_nhan,
                    temperature=self._cau_hinh.nhiet_do,
                )
                noi_dung = phan_hoi.choices[0].message.content or ""
                return _trich_json(noi_dung)
            except Exception as loi:  # bat rong: loi mang, loi parse, loi quota
                loi_cuoi = loi

        raise LoiGoiLLM(f"goi LLM that bai sau {self._cau_hinh.so_lan_thu_lai + 1} lan: {loi_cuoi}")
