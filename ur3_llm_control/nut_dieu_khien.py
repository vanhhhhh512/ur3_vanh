"""Node chinh: nhan cau lenh ngon ngu tu nhien va dieu khien UR3e.

Luong xu ly, dung theo so do cua de bai:

    Cau lenh nguoi dung
      -> LLM Planner (9Router)        bo_lap_ke_hoach_llm.py
      -> JSON Plan (chua dang tin)
      -> Plan Validator               kiem_tra_ke_hoach.py
      -> Skill Executor               bo_thuc_thi.py
      -> Robot Skills                 ky_nang_robot.py
      -> MoveIt 2                     giao_tiep_moveit.py
      -> UR3e trong Gazebo

Node co hai che do chay:
  * mot cau lenh roi thoat        --lenh "dua khoi do vao vung B"
  * vong lap tuong tac de quay demo (mac dinh), go cau lenh truc tiep o terminal
"""
from __future__ import annotations

import argparse
import os
import queue
import sys
import threading
from typing import List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener

from ur3_llm_control.bo_lap_ke_hoach_llm import (
    BoLapKeHoachLLM,
    LoiCauHinhLLM,
    LoiGoiLLM,
    doc_cau_hinh,
)
from ur3_llm_control.bo_thuc_thi import BoThucThi, mo_ta_buoc
from ur3_llm_control.dong_bo_gazebo import DongBoGazebo
from ur3_llm_control.giao_tiep_moveit import GiaoTiepMoveIt, LoiMoveIt
from ur3_llm_control.hien_thi_rviz import HienThiRviz
from ur3_llm_control.kiem_tra_ke_hoach import kiem_tra
from ur3_llm_control.ky_nang_robot import KyNangRobot
from ur3_llm_control.mo_hinh_workcell import doc_workcell
from ur3_llm_control.nhiem_vu_sinh_vien import doc_nhiem_vu
from ur3_llm_control.tay_kep import LoiTayKep, TayKep

NGAN = "=" * 68


class NutDieuKhien(Node):
    def __init__(self, thu_muc_cau_hinh: str, thu_muc_prompt: str,
                 ten_world: str, cho_nut_bam: bool) -> None:
        super().__init__("nut_dieu_khien_llm")
        self._nhom_cb = ReentrantCallbackGroup()

        self.workcell = doc_workcell(os.path.join(thu_muc_cau_hinh, "scene.yaml"))
        self.nhiem_vu = doc_nhiem_vu(os.path.join(thu_muc_cau_hinh, "student_config.yaml"))
        self._duong_dan_llm = os.path.join(thu_muc_cau_hinh, "llm.yaml")
        with open(os.path.join(thu_muc_prompt, "planner_system_prompt.txt"),
                  "r", encoding="utf-8") as tep:
            self._system_prompt = tep.read()

        self.moveit = GiaoTiepMoveIt(self, khung=self.workcell.khung,
                                     link_cong_tac=self.workcell.link_cong_tac)
        self.hien_thi = HienThiRviz(self, self.workcell)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self.gazebo = DongBoGazebo(self, ten_world=ten_world)
        self.gazebo.gan_nguon_tu_the(self._tu_the_tool0)

        self.tay_kep = TayKep(self)

        self.ky_nang = KyNangRobot(
            self.moveit, self.workcell, tay_kep=self.tay_kep,
            ghi_log=lambda dong: print(dong, flush=True),
            dong_bo_gazebo=lambda ten, xyz: self.gazebo.dat_vi_tri(ten, xyz),
            bam_theo_tay=self._bat_bam_theo)

        self._cho_nut_bam = cho_nut_bam
        self._hang_doi_nut: "queue.Queue[int]" = queue.Queue()
        if cho_nut_bam:
            self.create_subscription(Joy, "/rviz_visual_tools_gui",
                                     self._nhan_nut, 10, callback_group=self._nhom_cb)

        self.bo_thuc_thi = BoThucThi(
            self.ky_nang,
            ghi_log=lambda dong: print(dong, flush=True),
            cho_nut_bam=self._cho_bam_next if cho_nut_bam else None)

        self._bo_lap_ke_hoach: Optional[BoLapKeHoachLLM] = None

    # --------------------------------------------------------------- tien ich
    def _tu_the_tool0(self):
        """Vi tri tool0 trong he base_link, dung cho dong bo Gazebo."""
        try:
            bien_doi = self._tf_buffer.lookup_transform(
                self.workcell.khung, self.workcell.link_cong_tac,
                rclpy.time.Time())
        except Exception:
            return None
        t = bien_doi.transform.translation
        return (t.x, t.y, t.z)

    def _bat_bam_theo(self, ten_vat: Optional[str]) -> None:
        # Tam khoi nam o diem kep, thap hon tool0 dung dai_tcp
        cd = self.workcell.chuyen_dong
        lech = -(cd.dai_tcp + cd.khe_ho_gap)
        self.gazebo.bam_theo(ten_vat, lech_z=lech if ten_vat else 0.0)
        self.hien_thi.ve()

    def _nhan_nut(self, tin: Joy) -> None:
        # Panel RvizVisualToolsGui: buttons[1] la nut "Next"
        if len(tin.buttons) > 1 and tin.buttons[1] == 1:
            self._hang_doi_nut.put(1)

    def _cho_bam_next(self, nhan: str) -> None:
        print(f"  [cho bam Next trong RViz de chay {nhan}]", flush=True)
        while rclpy.ok():
            try:
                self._hang_doi_nut.get(timeout=0.1)
                return
            except queue.Empty:
                rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------ khoi tao
    def nap_workcell_vao_moveit(self) -> None:
        """Them ban va ba khoi vao planning scene de MoveIt biet ma tranh."""
        wc = self.workcell
        # Don sach scene cu truoc: move_group giu trang thai giua cac lan chay
        self.moveit.dat_lai_scene(["work_table", *wc.danh_sach_vat()])
        self.moveit.them_hop("work_table", wc.tam_ban, wc.kich_thuoc_ban)
        canh = (wc.canh_vat,) * 3
        for ten, vi_tri in wc.vi_tri_vat.items():
            self.moveit.them_hop(ten, vi_tri, canh)
        self.get_logger().info("da nap ban va 3 vat the vao planning scene")

    def chuan_bi_llm(self) -> None:
        cau_hinh = doc_cau_hinh(self._duong_dan_llm)
        self._bo_lap_ke_hoach = BoLapKeHoachLLM(cau_hinh, self._system_prompt)
        print(f"LLM: {cau_hinh.che_khoa()}", flush=True)

    def in_dau_trang(self) -> None:
        nv = self.nhiem_vu
        print(NGAN, flush=True)
        print(f"  STUDENT     : {nv.ten_sinh_vien}", flush=True)
        print(f"  STUDENT ID  : {nv.ma_sinh_vien}", flush=True)
        print(f"  P = {nv.hai_chu_so_cuoi} mod 6 = {nv.p}  ->  PERSONAL ASSIGNMENT:", flush=True)
        for vung, vat in nv.ban_giao_theo_thu_tu():
            print(f"      {vung}  <-  {vat}", flush=True)
        print(NGAN, flush=True)

    # -------------------------------------------------------- xu ly cau lenh
    def xu_ly(self, cau_lenh: str) -> bool:
        if self._bo_lap_ke_hoach is None:
            raise RuntimeError("chua chuan bi LLM")

        print("", flush=True)
        print("USER COMMAND:", flush=True)
        print(f"  {cau_lenh}", flush=True)

        try:
            ke_hoach_tho = self._bo_lap_ke_hoach.lap_ke_hoach(
                cau_lenh, bang_phan_cong=self.nhiem_vu.mo_ta_bang())
        except LoiGoiLLM as loi:
            print("", flush=True)
            print(f"LLM ERROR: {loi}", flush=True)
            return False

        ket_qua = kiem_tra(ke_hoach_tho, vat_the_tren_ban=self.workcell.danh_sach_vat())
        if not ket_qua.hop_le:
            print("", flush=True)
            print("LLM PLAN (raw):", flush=True)
            print(f"  {ke_hoach_tho}", flush=True)
            print("", flush=True)
            print("PLAN REJECTED:", flush=True)
            for ly_do in ket_qua.ly_do:
                print(f"  - {ly_do}", flush=True)
            print("", flush=True)
            print("TASK REJECTED - the robot did not move", flush=True)
            return False

        cac_dong = [mo_ta_buoc(b) for b in ket_qua.cac_buoc]
        print("", flush=True)
        print("LLM PLAN:", flush=True)
        for dong in cac_dong:
            print(f"  {dong}", flush=True)
        self.hien_thi.dat_ke_hoach(cac_dong)

        goc_cho_nut = self.bo_thuc_thi._cho_nut  # giu lai de boc them viec ve marker

        def cho_va_ve(nhan: str) -> None:
            self.hien_thi.dat_buoc(cac_dong.index(nhan) if nhan in cac_dong else None)
            if goc_cho_nut is not None:
                goc_cho_nut(nhan)

        self.bo_thuc_thi._cho_nut = cho_va_ve
        try:
            kq = self.bo_thuc_thi.chay(ket_qua.cac_buoc)
        finally:
            self.bo_thuc_thi._cho_nut = goc_cho_nut
            self.hien_thi.dat_buoc(None)

        print("", flush=True)
        print("WORKCELL STATE:", flush=True)
        print(self.workcell.tom_tat(), flush=True)
        return kq.thanh_cong


def _thu_muc_mac_dinh(ten: str) -> str:
    """Tim thu muc config/prompt, uu tien thu muc cai dat cua goi.

    Nho vay chay bang `ros2 run ur3_llm_control nut_dieu_khien` o bat ky dau
    cung duoc, khong can dung o goc ma nguon hay go duong dan day du.
    """
    if os.path.isdir(ten):
        return ten
    try:
        from ament_index_python.packages import get_package_share_directory
        duong_dan = os.path.join(get_package_share_directory("ur3_llm_control"), ten)
        if os.path.isdir(duong_dan):
            return duong_dan
    except Exception:
        pass
    return ten


def _doc_lenh_tu_ban_phim(hang_doi: "queue.Queue[Optional[str]]") -> None:
    for dong in sys.stdin:
        dong = dong.strip()
        if dong.lower() in {"quit", "exit", "thoat", "q"}:
            hang_doi.put(None)
            return
        if dong:
            hang_doi.put(dong)
    hang_doi.put(None)


def main(argv: Optional[List[str]] = None) -> int:
    bo_doc = argparse.ArgumentParser(description="Dieu khien UR3e bang cau lenh ngon ngu tu nhien")
    bo_doc.add_argument("--cau-hinh", default="", help="thu muc chua scene/student/llm yaml")
    bo_doc.add_argument("--prompt", default="", help="thu muc chua system prompt")
    bo_doc.add_argument("--lenh", default="", help="chay mot cau lenh roi thoat")
    bo_doc.add_argument("--world", default="ur3_workcell", help="ten world trong Gazebo")
    # Nhan chuoi thay vi co, vi launch file truyen xuong duoi dang "true"/"false"
    bo_doc.add_argument("--cho-nut-bam-neu", default="false",
                        help="true de dung truoc moi skill cho toi khi bam Next trong RViz")
    tham_so, con_lai = bo_doc.parse_known_args(argv if argv is not None else sys.argv[1:])

    thu_muc_cau_hinh = tham_so.cau_hinh or _thu_muc_mac_dinh("config")
    thu_muc_prompt = tham_so.prompt or _thu_muc_mac_dinh("prompt")

    rclpy.init(args=con_lai)
    cho_nut = str(tham_so.cho_nut_bam_neu).strip().lower() in {"true", "1", "yes", "co"}
    node = NutDieuKhien(thu_muc_cau_hinh, thu_muc_prompt, tham_so.world, cho_nut)
    ma_thoat = 0
    try:
        node.in_dau_trang()
        print("Waiting for MoveIt...", flush=True)
        node.moveit.cho_san_sang()
        node.tay_kep.cho_san_sang()
        node.nap_workcell_vao_moveit()
        node.hien_thi.ve()
        node.chuan_bi_llm()

        if tham_so.lenh:
            ma_thoat = 0 if node.xu_ly(tham_so.lenh) else 1
        else:
            print("", flush=True)
            print("Type a command and press Enter ('quit' to exit). Examples:", flush=True)
            print("  Put the red cube in zone A", flush=True)
            print("  Move the blue cube to zone B", flush=True)
            print("  Dua khoi mau vang vao vung C", flush=True)
            print("  Arrange all objects according to my student ID", flush=True)
            hang_doi: "queue.Queue[Optional[str]]" = queue.Queue()
            threading.Thread(target=_doc_lenh_tu_ban_phim, args=(hang_doi,), daemon=True).start()
            while rclpy.ok():
                try:
                    cau_lenh = hang_doi.get(timeout=0.1)
                except queue.Empty:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    continue
                if cau_lenh is None:
                    break
                node.xu_ly(cau_lenh)
                print("", flush=True)
                print("Next command:", flush=True)
    except (LoiCauHinhLLM, LoiMoveIt, LoiTayKep) as loi:
        print(f"ERROR: {loi}", flush=True)
        ma_thoat = 2
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return ma_thoat


if __name__ == "__main__":
    raise SystemExit(main())
