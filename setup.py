from glob import glob

from setuptools import setup

ten_goi = "ur3_llm_control"

setup(
    name=ten_goi,
    version="1.0.0",
    packages=[ten_goi],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{ten_goi}"]),
        (f"share/{ten_goi}", ["package.xml"]),
        (f"share/{ten_goi}/launch", glob("launch/*.launch.py")),
        (f"share/{ten_goi}/config", glob("config/*.yaml")),
        (f"share/{ten_goi}/prompt", glob("prompt/*.txt")),
        (f"share/{ten_goi}/worlds", glob("worlds/*.sdf")),
        (f"share/{ten_goi}/rviz", glob("rviz/*.rviz")),
        (f"share/{ten_goi}/urdf", glob("urdf/*.xacro")),
        (f"share/{ten_goi}/srdf", glob("srdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Do Viet Anh",
    maintainer_email="dangnh@aibb.vn",
    description="Dieu khien UR3e bang LLM va skill-based planning tren ROS 2 Humble",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"nut_dieu_khien = {ten_goi}.nut_dieu_khien:main",
        ],
    },
)
