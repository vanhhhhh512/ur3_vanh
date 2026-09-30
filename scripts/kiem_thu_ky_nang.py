#!/usr/bin/env python3
"""Kiem thu tang robot skill ma KHONG goi LLM.

Script nay nap thang mot ke hoach co san vao bo thuc thi de xac minh rang
pick/place/home chay duoc that trong Gazebo. No khong nam trong duong chay
chinh cua bai tap: duong chinh luon di qua LLM. Muc dich la tach loi cua
phan dieu khien robot ra khoi loi cua phan LLM khi go roi.

Cach chay (trong container, sau khi workcell.launch.py da len):
    python3 scripts/kiem_thu_ky_nang.py --kich-ban co_ban
    python3 scripts/kiem_thu_ky_nang.py --kich-ban nang_cao
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ur3_llm_control.bo_thuc_thi import BoThucThi  # noqa: E402
from ur3_llm_control.dong_bo_gazebo import DongBoGazebo  # noqa: E402
from ur3_llm_control.giao_tiep_moveit import GiaoTiepMoveIt  # noqa: E402
from ur3_llm_control.hien_thi_rviz import HienThiRviz  # noqa: E402
from ur3_llm_control.kiem_tra_ke_hoach import kiem_tra  # noqa: E402
from ur3_llm_control.ky_nang_robot import KyNangRobot  # noqa: E402
from ur3_llm_control.mo_hinh_workcell import doc_workcell  # noqa: E402
from ur3_llm_control.nhiem_vu_sinh_vien import doc_nhiem_vu  # noqa: E402
from ur3_llm_control.tay_kep import TayKep  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402


def ke_hoach_co_ban():
    return {"plan": [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]}


def ke_hoach_nang_cao(nhiem_vu):
    cac_buoc = []
    for vung, vat in nhiem_vu.ban_giao_theo_thu_tu():
        cac_buoc.append({"skill": "pick", "object": vat})
        cac_buoc.append({"skill": "place", "object": vat, "zone": vung})
    cac_buoc.append({"skill": "home"})
    return {"plan": cac_buoc}


def main() -> int:
    bo_doc = argparse.ArgumentParser()
    bo_doc.add_argument("--kich-ban", choices=["co_ban", "nang_cao"], default="co_ban")
    bo_doc.add_argument("--cau-hinh", default="config")
    bo_doc.add_argument("--world", default="ur3_workcell")
    tham_so, con_lai = bo_doc.parse_known_args()

    rclpy.init(args=con_lai)
    node = Node("kiem_thu_ky_nang")
    workcell = doc_workcell(os.path.join(tham_so.cau_hinh, "scene.yaml"))
    nhiem_vu = doc_nhiem_vu(os.path.join(tham_so.cau_hinh, "student_config.yaml"))

    moveit = GiaoTiepMoveIt(node, khung=workcell.khung, link_cong_tac=workcell.link_cong_tac)
    hien_thi = HienThiRviz(node, workcell)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node, spin_thread=True)

    def tu_the_tool0():
        try:
            bd = tf_buffer.lookup_transform(workcell.khung, workcell.link_cong_tac, rclpy.time.Time())
        except Exception:
            return None
        t = bd.transform.translation
        return (t.x, t.y, t.z)

    gazebo = DongBoGazebo(node, ten_world=tham_so.world)
    gazebo.gan_nguon_tu_the(tu_the_tool0)

    def bam_theo(ten):
        lech = -(workcell.chuyen_dong.dai_tcp + workcell.chuyen_dong.khe_ho_gap)
        gazebo.bam_theo(ten, lech_z=lech if ten else 0.0)
        hien_thi.ve()

    tay_kep = TayKep(node)
    ky_nang = KyNangRobot(moveit, workcell, tay_kep=tay_kep,
                          ghi_log=lambda d: print(d, flush=True),
                          dong_bo_gazebo=lambda t, x: gazebo.dat_vi_tri(t, x),
                          bam_theo_tay=bam_theo)

    print("Cho MoveIt san sang...", flush=True)
    moveit.cho_san_sang()
    tay_kep.cho_san_sang()
    moveit.dat_lai_scene(["work_table", *workcell.danh_sach_vat()])
    moveit.them_hop("work_table", workcell.tam_ban, workcell.kich_thuoc_ban)
    for ten, vi_tri in workcell.vi_tri_vat.items():
        moveit.them_hop(ten, vi_tri, (workcell.canh_vat,) * 3)
    hien_thi.ve()

    tho = ke_hoach_co_ban() if tham_so.kich_ban == "co_ban" else ke_hoach_nang_cao(nhiem_vu)
    kq_kiem = kiem_tra(tho, vat_the_tren_ban=workcell.danh_sach_vat())
    print(f"kiem tra ke hoach: {kq_kiem.bao_cao()}", flush=True)
    if not kq_kiem.hop_le:
        return 2

    hien_thi.dat_ke_hoach([f"{b['skill']}" for b in kq_kiem.cac_buoc])
    bat_dau = time.monotonic()
    kq = BoThucThi(ky_nang, ghi_log=lambda d: print(d, flush=True)).chay(kq_kiem.cac_buoc)
    print("", flush=True)
    print(f"tong thoi gian thuc thi: {time.monotonic() - bat_dau:.1f} s", flush=True)
    print(workcell.tom_tat(), flush=True)

    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if kq.thanh_cong else 1


if __name__ == "__main__":
    raise SystemExit(main())
