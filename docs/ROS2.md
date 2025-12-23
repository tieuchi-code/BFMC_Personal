# HƯỚNG DẪN ROS2 HUMBLE + GAZEBO

## 1 Cập nhật hệ thống
```bash
sudo apt update && sudo apt upgrade -y
```

## 2 Cài các gói phụ trợ
```bash
sudo apt install software-properties-common curl gnupg lsb-release -y
```

## 3 Thêm khóa GPG mới cho ROS2
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

## 4 Thêm ROS2 repository
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 5 Update để apt nhận repo ROS2
```bash
sudo apt update
```

Nếu thấy lỗi:

**NO_PUBKEY**
*Thiếu key, quay lại bước 2.*

**repository is not signed**
*Sai key, quay lại bước 2.*

Nếu không lỗi,tiếp tục.


## 6 KHẮC PHỤC LỖI QUAN TRỌNG NHẤT (Sẽ xảy ra trên nhiều máy)

Ubuntu thường nâng các thư viện hệ thống lên bản mới hơn, còn ROS2 chỉ build cho bản cũ hơn, gây lỗi:
- libcom-err2
- libpulse0, libpulse-mainloop-glib0
- libusb-1.0-0

Để đảm bảo cài ROS2 không lỗi trên mọi máy, thêm một file ưu tiên apt:
```bash
sudo nano /etc/apt/preferences.d/ros2-fix.pref
```
Rồi copy đoạn code dưới này vào file
```
Package: libcom-err2
Pin: version 1.46.5-2ubuntu1.1
Pin-Priority: 1001

Package: libpulse0
Pin: version 1:15.99.1+dfsg1-1ubuntu1
Pin-Priority: 1001

Package: libpulse-mainloop-glib0
Pin: version 1:15.99.1+dfsg1-1ubuntu1
Pin-Priority: 1001

Package: libusb-1.0-0
Pin: version 2:1.0.25-1ubuntu1
Pin-Priority: 1001
```
Lưu file:
**CTRL + O → ENTER → CTRL + X**

## 7 Update lại apt
```bash
sudo apt update
```
*Nó sẽ nhận version ưu tiên vừa pin.*

## 8 Downgrade các lib gây xung đột
*Để phù hợp ROS2*
```bash
sudo apt install --allow-downgrades \
    libcom-err2=1.46.5-2ubuntu1.1 \
    libpulse0=1:15.99.1+dfsg1-1ubuntu1 \
    libpulse-mainloop-glib0=1:15.99.1+dfsg1-1ubuntu1 \
    libusb-1.0-0=2:1.0.25-1ubuntu1 -y
```

## 9 Cài ROS2 Humble Desktop-Full
```bash
sudo apt install ros-humble-desktop-full -y
```
Nếu muốn cài bản nhẹ
```bash
sudo apt install ros-humble-desktop -y
```

## 10 Thêm ROS2 vào ~/.bashrc
Không cần source lại mỗi lần chạy ROS2
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 11 KIỂM TRA ROS2
**Mở một Terminal**
```bash
ros2 run demo_nodes_cpp talker
```
**Mở thêm một Terminal thứ 2**
```bash
ros2 run demo_nodes_py listener
```
*Nếu nhận “Hello World” → ROS2 hoạt động chuẩn.*

## 12 Cài Gazebo Fortress
*ROS2 Humble đã bao gồm toàn bộ bộ mô phỏng Fortress*
**Chạy lệnh sau để cài**
```bash
sudo apt install ros-humble-ros-ign ros-humble-ros-ign-gazebo -y
```
Nếu muốn GUI và plugin đầy đủ
```bash
sudo apt install ros-humble-simulation -y
```

## 13 Kiểm tra Gazebo Fortress
```bash
ign gazebo --versions
```
Kết quả đúng phải là:
```
6.x.x
```
## 14 Mở mô phỏng Gazebo Fortress
```bash
ign gazebo shapes.sdf
```
*Nếu cửa số mô phỏng mở ra → thành công 100%.*

## 15 Kiểm tra ROS2 ↔ Gazebo Fortress bridge
```bash
ros2 run ros_gz_bridge parameter_bridge
```



