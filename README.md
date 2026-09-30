# UR3e điều khiển bằng LLM và skill-based planning — ROS 2 Humble + MoveIt 2 + Ignition Gazebo

## 1. Giới thiệu đề tài

Bài thực hành xây dựng một package ROS 2 cho phép điều khiển cánh tay **Universal Robots UR3e**
bằng câu lệnh ngôn ngữ tự nhiên, tiếng Việt hoặc tiếng Anh. Robot mang một tay kẹp song song hai
ngón và thao tác trên một bàn gồm ba khối lập phương (`red_cube`, `yellow_cube`, `blue_cube`) và
ba vùng đặt (`zone_a`, `zone_b`, `zone_c`).

Trọng tâm của cách làm ở đây là **LLM chỉ được chọn và sắp xếp skill**. Câu lệnh đi qua LLM (kết
nối bằng 9Router) để thành một kế hoạch JSON gồm các bước `pick`, `place`, `home`; kế hoạch đó bị
coi là **chưa đáng tin** cho tới khi qua được bộ kiểm tra. Bộ kiểm tra làm việc theo nguyên tắc
*fail-closed*: sai một điểm là từ chối cả kế hoạch, robot đứng yên. LLM không bao giờ sinh giá trị
khớp, toạ độ hay quỹ đạo — những thứ đó do chương trình tự tính từ file cấu hình workcell rồi giao
cho MoveIt 2 lập kế hoạch có kiểm tra va chạm.

Nhiệm vụ cá nhân được tính từ mã sinh viên: **MSSV 23020719 → P = 19 mod 6 = 1**, tức
Zone A ← red_cube, Zone B ← blue_cube, Zone C ← yellow_cube.

* **Video demo:** https://drive.google.com/drive/folders/1AzJHNEvZAVp52gYf9NGtPI1x9Tnfeu-g
* **Mã nguồn:** https://github.com/vanhhhhh512/ur3_vanh

---

## 2. Cấu trúc thư mục

```
ur3_vanh/
├── .gitignore
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/ur3_llm_control
├── launch/
│   ├── workcell.launch.py           # Gazebo + UR3e + tay kẹp + controller + MoveIt + RViz
│   └── llm_robot.launch.py          # node nhận câu lệnh, chế độ một câu lệnh
├── ur3_llm_control/
│   ├── nut_dieu_khien.py            # node chính: câu lệnh → LLM → kiểm tra → thực thi
│   ├── bo_lap_ke_hoach_llm.py       # LLM planner, gọi 9Router
│   ├── kiem_tra_ke_hoach.py         # plan validator, fail-closed
│   ├── bo_thuc_thi.py               # skill executor, dừng ở bước đầu tiên hỏng
│   ├── ky_nang_robot.py             # ba skill home / pick / place
│   ├── tay_kep.py                   # mở / đóng tay kẹp hai ngón
│   ├── giao_tiep_moveit.py          # gọi service và action của move_group
│   ├── mo_hinh_workcell.py          # đọc scene.yaml, giữ trạng thái workcell
│   ├── nhiem_vu_sinh_vien.py        # P = MSSV mod 6 → bảng phân công
│   ├── dong_bo_gazebo.py            # cho vật đang kẹp bám theo tay kẹp trong Gazebo
│   ├── hien_thi_rviz.py             # marker bàn, vùng, vật và kế hoạch trên RViz
│   └── danh_muc.py                  # danh mục skill / object / zone / trạng thái
├── config/
│   ├── scene.yaml                   # toạ độ bàn, vật, vùng và tham số chuyển động
│   ├── student_config.yaml          # tên, MSSV, bảng quy ước 6 trường hợp của P
│   ├── llm.yaml                     # cấu hình 9Router, khoá đọc từ biến môi trường
│   ├── ur3_controllers.yaml         # controller tay máy và controller tay kẹp
│   ├── kinematics.yaml              # IK solver cho MoveIt
│   └── moveit_joint_limits.yaml     # giới hạn vận tốc, gia tốc cho MoveIt
├── prompt/planner_system_prompt.txt # system prompt gửi cho LLM
├── urdf/
│   ├── ur3e_tay_kep.urdf.xacro      # UR3e gắn tay kẹp vào tool0
│   └── tay_kep_hai_ngon.xacro       # mô hình tay kẹp song song hai ngón
├── srdf/ur3e_tay_kep.srdf.xacro     # nhóm ur_manipulator + cặp link tay kẹp
├── worlds/ur3_workcell.sdf          # bàn, ba khối, ba vùng đặt
├── rviz/view_robot.rviz             # cấu hình RViz kèm panel nút Next
└── scripts/kiem_thu_ky_nang.py      # kiểm thử tầng skill, không gọi LLM
```

Hai launch file nằm trong `launch/`:

| Launch file | Nhiệm vụ |
| :--- | :--- |
| **`workcell.launch.py`** | Dựng môi trường mô phỏng: Gazebo (UR3e + tay kẹp + bàn + ba khối + ba vùng), controller, MoveIt, RViz |
| **`llm_robot.launch.py`** | Chạy node `nut_dieu_khien` với một câu lệnh cho trước rồi thoát |

---

## 3. Yêu cầu hệ thống và cài đặt

| Thành phần | Phiên bản |
| :--- | :--- |
| Hệ điều hành | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill (Desktop) |
| Mô phỏng | Ignition Gazebo Fortress |
| Lập kế hoạch chuyển động | MoveIt 2 |
| LLM gateway | 9Router |

Cài ROS 2 Humble (bỏ qua nếu máy đã có):

```bash
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-humble-desktop
```

Các gói cho bài này:

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ur-simulation-gz \
  ros-humble-ur-moveit-config \
  ros-humble-ur-description \
  ros-humble-ign-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rviz-visual-tools \
  python3-colcon-common-extensions \
  python3-pip

python3 -m pip install "openai>=1.0,<2.0"
```

### Kết nối LLM qua 9Router

9Router mở một endpoint tương thích OpenAI Chat Completions ở cổng 20128, nên mã nguồn chỉ dùng SDK
`openai` và đổi model chỉ cần đổi biến môi trường.

```bash
docker run -d --name 9router -p 20128:20128 \
  -v "$HOME/.9router:/app/data" -e DATA_DIR=/app/data \
  -e INITIAL_PASSWORD='doi-mat-khau-cua-ban' decolua/9router:latest
```

Mở `http://localhost:20128`, vào **Providers** bật một provider (ví dụ `OpenCode Free`, không cần
đăng nhập), vào **API Keys** tạo khoá. Nếu 9Router chạy ở máy khác thì mở đường hầm:

```bash
ssh -L 20128:127.0.0.1:20128 <user>@<may-chay-9router>
```

> **Lưu ý về Python.** Nếu máy có Anaconda/Miniconda, `colcon` sẽ bắt nhầm Python của conda và báo
> `ModuleNotFoundError: No module named 'catkin_pkg'`. Chạy `conda deactivate` trước khi build.

---

## 4. Hướng dẫn build

```bash
mkdir -p ~/ur3_llm_ws/src && cd ~/ur3_llm_ws/src
git clone https://github.com/vanhhhhh512/ur3_vanh.git ur3_llm_control
cd ~/ur3_llm_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control
source install/setup.bash
```

URDF, SRDF và controller của bài được đưa vào launch chính thức của Universal Robots qua các tham số
`description_package`, `moveit_config_package` và `runtime_config_package` — không phải sửa file
nào trong `/opt/ros`. Launch MoveIt của UR tìm `kinematics.yaml` và giới hạn khớp trong
`moveit_config_package`, nên hai file đó nằm trong `config/` của package.

Khi `git pull` bản mới có đổi tên file trong `rviz/`, `urdf/` hay `srdf/`, xoá build cũ trước khi
build lại:

```bash
rm -rf build/ur3_llm_control install/ur3_llm_control
```

---

## 5. Hướng dẫn chạy

Mỗi terminal mới đều cần:

```bash
cd ~/ur3_llm_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```

### Cách 1 — gõ câu lệnh trực tiếp (dùng khi demo)

```bash
# Terminal 1
ros2 launch ur3_llm_control workcell.launch.py

# Terminal 2
export NINEROUTER_BASE_URL=http://127.0.0.1:20128/v1
export NINEROUTER_API_KEY=sk-...
export NINEROUTER_MODEL=oc/muse-spark-1.3-contributor-free
ros2 run ur3_llm_control nut_dieu_khien
```

Đợi terminal 1 in `Configured and activated tay_kep_controller` rồi mới chạy terminal 2. Terminal 2
phải dùng `ros2 run` chứ không dùng `ros2 launch`, vì `ros2 launch` không chuyển bàn phím vào node.
Gõ câu lệnh rồi Enter, gõ `quit` để thoát.

Ví dụ câu lệnh:

```
Arrange all objects according to my student ID
Put the red cube in zone A
Move the blue cube to zone B
Pick up the yellow cube and place it in zone C
Set joint 3 to 1.57 radians                    ← bị từ chối, robot đứng yên
```

### Cách 2 — bấm nút rồi mới chạy từng skill (dùng khi quay video)

```bash
ros2 run ur3_llm_control nut_dieu_khien --cho-nut-bam-neu true
```

Trước mỗi skill robot đứng chờ, bấm **Next** trong panel `RvizVisualToolsGui` ở góc trên bên trái
RViz thì robot mới chạy bước đó.

### Cách 3 — một câu lệnh rồi thoát: **`llm_robot.launch.py`**

```bash
ros2 launch ur3_llm_control llm_robot.launch.py lenh:="Put the red cube in zone A"
```

### Đưa ba khối về chỗ cũ khi sim đang chạy

Gõ `quit` ở terminal 2, rồi:

```bash
for n in "red_cube -0.15" "yellow_cube 0.0" "blue_cube 0.15"; do set -- $n
  ign service -s /world/ur3_workcell/set_pose \
    --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 1000 \
    --req "name: \"$1\", position: {x: 0.40, y: $2, z: 0.1725}, orientation: {x: 0, y: 0, z: 0, w: 1}"
done
ros2 run ur3_llm_control nut_dieu_khien
```

Khi khởi động, node đọc lại `scene.yaml` và nạp lại bàn cùng ba khối vào MoveIt.

### Kiểm tra robot, joint state và controller

```bash
ros2 node list                    # /move_group, /robot_state_publisher, /controller_manager ...
ros2 topic hz /joint_states       # 6 khớp tay máy + 2 khớp ngón kẹp
ros2 control list_controllers     # joint_state_broadcaster, joint_trajectory_controller, tay_kep_controller: active
ros2 action list | grep follow_joint_trajectory
```

### Kiểm thử tầng skill, không gọi LLM

```bash
cd ~/ur3_llm_ws/src/ur3_llm_control
python3 scripts/kiem_thu_ky_nang.py --kich-ban co_ban      # 1 vật
python3 scripts/kiem_thu_ky_nang.py --kich-ban nang_cao    # 3 vật theo MSSV
```

Script nạp thẳng một kế hoạch có sẵn vào bộ thực thi, dùng để tách lỗi của phần robot ra khỏi lỗi
của phần LLM.

---

## 6. Nguyên lý hoạt động

### 6.1 Luồng xử lý một câu lệnh

```
Câu lệnh ngôn ngữ tự nhiên
        ↓  bo_lap_ke_hoach_llm.py  — 9Router, temperature = 0
JSON plan  (chưa đáng tin)
        ↓  kiem_tra_ke_hoach.py    — sai một điểm là từ chối cả kế hoạch
Kế hoạch hợp lệ
        ↓  bo_thuc_thi.py          — dừng ở bước đầu tiên không SUCCESS
Robot skills  home / pick / place
        ↓  giao_tiep_moveit.py     — /move_action, /compute_ik, /compute_cartesian_path
MoveIt 2
        ↓
UR3e + tay kẹp trong Gazebo
```

### 6.2 LLM planner

Hội thoại gửi cho LLM gồm đúng hai tin nhắn: system prompt (`prompt/planner_system_prompt.txt`) và
câu lệnh của người dùng. System prompt liệt kê ba skill cùng tham số, ba object, ba zone, định dạng
output `{"plan": [...]}`, bảng ánh xạ từ vựng Việt–Anh, và quy tắc: câu mơ hồ hay yêu cầu ngoài
phạm vi thì trả `{"plan": []}`. Không có câu lệnh nào bị viết cứng trong mã nguồn — việc hiểu câu
hoàn toàn do LLM.

Bảng phân công theo MSSV được nối vào cuối system prompt dưới nhãn `ASSIGNMENT TABLE`, không tách
thành tin nhắn riêng, vì model nhỏ hay bỏ qua system message thứ hai.

### 6.3 Plan validator

`kiem_tra_ke_hoach.py` chặn 12 nhóm lỗi: kết quả không phải JSON, thiếu `plan`, `plan` không phải
danh sách, kế hoạch rỗng, kế hoạch dài bất thường, skill lạ, **tham số thừa** (ví dụ
`{"skill": "home", "joint": [0, 1.57]}`), thiếu tham số, object hoặc zone không hợp lệ, `pick` khi
đang cầm / `place` khi tay trống, hai vật vào cùng một vùng, và kết thúc khi tay còn cầm vật. Các
lỗi thứ tự được phát hiện bằng cách mô phỏng trạng thái tay kẹp qua từng bước của kế hoạch.

### 6.4 Robot skills và tay kẹp

| Skill | Các chặng | Trạng thái trả về |
| :--- | :--- | :--- |
| `home()` | Về tư thế khớp `tu_the_home` | `SUCCESS`, `FAILED`, `PLANNING_FAILED` |
| `pick(object)` | mở kẹp → tới trên vật → hạ thẳng → đóng kẹp → nhấc lên | thêm `INVALID_OBJECT` |
| `place(object, zone)` | mang tới trên vùng → hạ thẳng → mở kẹp → rút lên | thêm `INVALID_ZONE` |

Tay kẹp gồm thân hộp 100 × 64 × 40 mm và hai ngón là hai khớp trượt đối xứng, do
`tay_kep_controller` điều khiển. Điểm kẹp nằm giữa hai ngón, cách `tool0` 60 mm. Mở kẹp tạo khe
75 mm, đóng kẹp tạo khe 46 mm ôm sát khối 45 mm.

Khi đóng kẹp, khối được gắn vào tay kẹp trong planning scene dưới dạng `AttachedCollisionObject`,
nên MoveIt tính va chạm cho cả vật đang cầm. Trong Gazebo, khối không được giữ bằng ma sát (khối
45 mm trượt hoặc văng rất dễ) mà được cập nhật vị trí theo điểm kẹp 40 lần mỗi giây trên một luồng
riêng. Ba khối tắt va chạm vật lý để ngón kẹp không đẩy lệch khối giữa hai lần cập nhật.

### 6.5 Lập kế hoạch một chuyển động

Với mỗi điểm đích (ví dụ điểm treo 8 cm trên khối):

1. gọi `/compute_ik` một lần, hạt giống là tư thế khớp hiện tại, nên nghiệm giữ nguyên nhánh khuỷu
   và cổ tay đang dùng, quỹ đạo ngắn;
2. xoay `wrist_3` thêm bội số 90° để nằm trong [−45°, 45°];
3. gọi `/move_action` để MoveIt vừa lập kế hoạch (OMPL RRTConnect, có kiểm tra va chạm) vừa thực
   thi ở 50 % vận tốc;
4. nếu thất bại thì thử lần lượt các nhánh nghiệm IK khác, rồi ghé qua một tư thế trung chuyển.

Bước 2 cần thiết vì với UR3e, `wrist_3` khai báo không giới hạn vị trí: MoveIt coi nó là khớp quay
vô hạn và luôn quy góc về [−π, π], còn controller giữ góc thô. Để `wrist_3` trôi qua ±π thì hai bên
lệch nhau đúng 2π và controller huỷ quỹ đạo. Tay kẹp đối xứng, khối vuông, nên xoay 90° vẫn kẹp
được như cũ.

Các đoạn hạ và nhấc thẳng đứng dùng `/compute_cartesian_path` ở 25 % vận tốc, giữ nguyên hướng tay
kẹp và tắt kiểm tra va chạm — thông lệ approach/retreat của MoveIt, vì lúc hạ hai ngón ôm sát khối,
lúc nhấc khối vừa kẹp vẫn chạm mặt bàn. Đoạn này chỉ dài 8 cm, ngay trên vật hoặc vùng đã biết là
trống.

### 6.6 Nhiệm vụ cá nhân theo MSSV

`nhiem_vu_sinh_vien.py` lấy hai chữ số cuối của `student_id`, tính `P = XX mod 6`, rồi tra bảng đủ
6 trường hợp trong `config/student_config.yaml`. Đổi MSSV là đổi nhiệm vụ ngay, không sửa mã.

```
MSSV 23020719  →  19 mod 6 = 1  →  zone_a ← red_cube, zone_b ← blue_cube, zone_c ← yellow_cube
```

Nếu vùng đích đã có vật khác, kế hoạch bị từ chối ngay ở validator (hai vật cùng một vùng trong một
kế hoạch) hoặc skill `place` trả `FAILED` (vùng đã bị chiếm từ câu lệnh trước), robot không đặt
chồng.

---

## 7. Tham số workcell

Toàn bộ hình học nằm trong `config/scene.yaml`, toạ độ tính trong hệ `base_link`, đơn vị mét.

| Đối tượng | Vị trí | | Vùng đặt | Tâm vùng |
| :--- | :--- | :--- | :--- | :--- |
| red_cube | (0.40, −0.15, 0.1725) | | zone_a | (0.30, −0.15, 0.15) |
| yellow_cube | (0.40, 0.00, 0.1725) | | zone_b | (0.30, 0.00, 0.15) |
| blue_cube | (0.40, +0.15, 0.1725) | | zone_c | (0.30, +0.15, 0.15) |

Bàn thao tác tâm (0.44, 0, 0.075), kích thước 0.48 × 0.70 × 0.15 m; khối cạnh 45 mm; vùng đặt
vuông 100 mm. Các số này phải khớp với `worlds/ur3_workcell.sdf`.

| Tham số (`chuyen_dong`) | Mặc định | Ý nghĩa |
| :--- | :---: | :--- |
| `dai_tcp` | `0.060` | Khoảng cách từ `tool0` tới điểm kẹp (m) |
| `cao_tiep_can` | `0.08` | Độ cao điểm treo trên vật / vùng trước khi hạ (m) |
| `khe_ho_gap` | `0.003` | Tâm kẹp cao hơn tâm khối một chút để ngón không chạm bàn (m) |
| `he_so_van_toc` / `he_so_gia_toc` | `0.5` | Hệ số tốc độ khi di chuyển tự do |
| `he_so_van_toc_thang` / `he_so_gia_toc_thang` | `0.25` | Hệ số tốc độ khi hạ / nhấc thẳng |
| `buoc_cartesian` | `0.005` | Bước nội suy đường thẳng (m) |
| `ty_le_cartesian_toi_thieu` | `0.85` | Tỉ lệ đường thẳng tối thiểu phải tính được |
| `tu_the_home` | `[0, −1.5708, 1.2217, −1.2217, −1.5708, 0]` | Tư thế khớp home (rad) |

---

## 8. Kiểm chứng và kết quả đo

**Hiểu câu lệnh** — 9 câu thử, đúng cả 9:

| Câu lệnh | Kế hoạch LLM sinh ra |
| :--- | :--- |
| Dua khoi mau do vao vung A | pick(red_cube) → place(red_cube, zone_a) → home() |
| Hay lay khoi mau vang va dat no vao o C | pick(yellow_cube) → place(yellow_cube, zone_c) → home() |
| Move the blue cube to zone B | pick(blue_cube) → place(blue_cube, zone_b) → home() |
| Please put the red cube in zone A. | pick(red_cube) → place(red_cube, zone_a) → home() |
| Arrange all objects according to my student ID | 7 bước theo bảng P = 1 |
| Sap xep tat ca cac vat theo ma so sinh vien | 7 bước theo bảng P = 1 |
| Han khoi mau tim vao vung Z | **từ chối** |
| Dat khop so 3 bang 1.57 radian | **từ chối** |
| Lay cai do kia | **từ chối** |

**Mức nâng cao** — `Arrange all objects according to my student ID`:

```
LLM PLAN:
  pick(red_cube)
  place(red_cube, zone_a)
  pick(blue_cube)
  place(blue_cube, zone_b)
  pick(yellow_cube)
  place(yellow_cube, zone_c)
  home()

EXECUTION:
  pick(red_cube) ................ SUCCESS
  place(red_cube, zone_a) ....... SUCCESS
  pick(blue_cube) ............... SUCCESS
  place(blue_cube, zone_b) ...... SUCCESS
  pick(yellow_cube) ............. SUCCESS
  place(yellow_cube, zone_c) .... SUCCESS
  home() ........................ SUCCESS

TASK SUCCESS
```

**Kế hoạch bị từ chối** — robot không cử động:

```
USER COMMAND:
  Weld the purple cube into zone Z and set joint 3 to 1.57 radians

LLM PLAN (raw):
  {'plan': []}

PLAN REJECTED:
  - LLM tra ve ke hoach rong, khong du thong tin de thuc thi

TASK REJECTED - the robot did not move
```

Kết quả đo với tham số mặc định:

| Chỉ số | Giá trị |
| :--- | :--- |
| Mức cơ bản (1 vật), chạy lặp | **3/3** lần `TASK SUCCESS`, 11.6–23.9 s |
| Mức nâng cao (3 vật), chạy lặp liên tiếp | **5/5** lần `TASK SUCCESS`, 38.0–48.0 s, trung bình 43.7 s |
| Thời gian một chặng di chuyển | trung bình 0.8–1.7 s, lớn nhất 2.3 s (95 chặng) |
| Số chặng phải đi vòng qua tư thế trung chuyển | 0 / 95 |
| Vị trí khối trong Gazebo sau khi đặt | trùng tâm vùng đặt tới 10⁻⁶ m |

---

## 9. Đối chiếu với yêu cầu bài thực hành

| Yêu cầu của đề | Đáp ứng ở đâu |
| :--- | :--- |
| Ubuntu 22.04 + ROS 2 Humble, MoveIt 2, Gazebo | Mục 3 |
| UR3/UR3e simulation | **`workcell.launch.py`** → `ur_simulation_gz` + Ignition Fortress, UR3e gắn tay kẹp |
| Kết nối LLM qua 9Router | `bo_lap_ke_hoach_llm.py`, `config/llm.yaml` — mục 3, 6.2 |
| Môi trường: 1 UR3e, 1 bàn, 3 vật, 3 vùng | `worlds/ur3_workcell.sdf`, toạ độ khai báo trong `config/scene.yaml` — mục 7 |
| Skill `home`, `pick`, `place` qua MoveIt 2 | `ky_nang_robot.py` — mục 6.4 |
| Skill trả về trạng thái | `SUCCESS`, `FAILED`, `INVALID_OBJECT`, `INVALID_ZONE`, `PLANNING_FAILED` |
| Không vượt joint limit, không self-collision, không va chạm môi trường | `safety_limits` bật, MoveIt kiểm tra va chạm với bàn, ba khối, robot và tay kẹp — mục 6.5 |
| Node nhận câu lệnh ngôn ngữ tự nhiên | Node `/nut_dieu_khien_llm` — mục 5 |
| LLM → JSON plan → validator → executor | `bo_lap_ke_hoach_llm.py`, `kiem_tra_ke_hoach.py`, `bo_thuc_thi.py` — mục 6.1 |
| Từ chối skill / object / zone không hợp lệ | 12 nhóm lỗi, fail-closed — mục 6.3 |
| LLM không sinh joint trajectory | Prompt cấm, validator chặn mọi tham số thừa — mục 6.2, 6.3 |
| Nhiều cách diễn đạt, không hard-code | 9/9 câu tiếng Việt và tiếng Anh — mục 8 |
| Cá nhân hoá theo MSSV | `nhiem_vu_sinh_vien.py`, `student_config.yaml` — mục 6.6 |
| Mức cơ bản | `Put the red cube in zone A`, 3/3 lần thành công — mục 8 |
| Mức nâng cao | `Arrange all objects according to my student ID`, 5/5 lần thành công — mục 8 |
| Terminal hiển thị USER COMMAND / LLM PLAN / EXECUTION / TASK SUCCESS | Mục 8 |
