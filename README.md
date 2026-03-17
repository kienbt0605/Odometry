# 🤖 EMILY AI BOT - CONTROL SYSTEM DOCUMENTATION

Hệ thống điều khiển robot tự hành tích hợp động học **Odometry** và truyền thông không dây **ESP-NOW**. Hệ thống được thiết kế theo kiến trúc phân tầng để tối ưu hóa hiệu năng xử lý giữa Master (Raspberry Pi) và Motor Driver (ESP32).

---

## 🛠 1. KIẾN TRÚC TRUYỀN THÔNG (COMMUNICATION)
* **Tầng Master:** Raspberry Pi tính toán quỹ đạo và vận tốc $(v, w)$.
* **Tầng Trung gian:** ESP32 Gateway nhận dữ liệu từ Master.
* **Tầng Thực thi (DriverCore):** Client ESP32 nhận dữ liệu qua **ESP-NOW** và điều khiển trực tiếp động cơ.

---

## 📂 2. CẤU TRÚC MÃ NGUỒN (CODE TABS)

### ⚙️ [Tab 1: CONFIGURATION]
Thiết lập các hằng số vật lý và định nghĩa phần cứng:
* **Physical Const:** `WHEEL_DIAMETER`, `WHEEL_BASE`, `ENCODER_PPR`.
* **Pinout L298N:** `IN1`, `IN2`, `IN3`, `IN4`, `ENA`, `ENB`.
* **Pinout Encoder:** `ENCODER_1A`, `ENCODER_1B`, `ENCODER_2A`, `ENCODER_2B`.
* **Variables:** Cấu trúc nhận dữ liệu $(v, w)$, cờ ngắt `odmFlag`, và biến lưu trữ giá trị Encoder.

### 🔢 [Tab 2: ENCODER]
* Chứa các hàm ngắt `encoder1_isr()` và `encoder2_isr()`.
* Đọc và đếm số xung từ bánh trái và bánh phải theo thời gian thực để phục vụ tính toán vận tốc thực.

### 🏎️ [Tab 3: MOTOR]
* Hàm điều khiển công suất động cơ.
* Nhận giá trị PWM từ `-255` đến `255` để điều hướng (Tiến/Lùi) và tốc độ.

### 📡 [Tab 4: RX]
* Xử lý nhận dữ liệu không dây thông qua giao thức **ESP-NOW**.
* Giải mã gói tin nhận được từ Gateway để gán vào biến $v$ và $w$.

### 📐 [Tab 5: ODOMETRY & PID]
Bộ não điều khiển chuyển động chính xác:
* **Động học thuận:** Tính toán tốc độ mong muốn cho từng bánh dựa trên $(v, w)$.
* **PID Correction:** Sử dụng bộ tham số $K_p, K_d$ để bù trừ sai lệch giữa hai bánh xe.
* **Quy trình thực thi:**
    1. Tính $\Delta$ Encoder trong chu kỳ $DT = 20ms$.
    2. Tính sai số `enc_error` giữa bánh trái và phải.
    3. Trả về giá trị `correction` để điều chỉnh PWM.
    4. Cấp lệnh xuống hàm **MOTOR**.

---

## 🚀 3. THÔNG SỐ CÀI ĐẶT (SYSTEM SPECS)

| Tham số | Giá trị | Ý nghĩa |
| :--- | :--- | :--- |
| **Sampling Time** | 20ms | Chu kỳ cập nhật PID & Odometry |
| **Algorithm** | PD Control | Hiệu chỉnh cân bằng bánh xe |
| **Wireless** | ESP-NOW | Giao thức truyền tin độ trễ thấp |
| **MCU** | ESP32 | Vi điều khiển thực thi tầng thấp |

---

## 📝 4. HƯỚNG DẪN CÀI ĐẶT
1.  **Cấu hình chân:** Thay đổi các chân GPIO trong tab `CONFIGURATION` cho phù hợp với mạch PCB.
2.  **Căn chỉnh PID:** Điều chỉnh $K_p$ và $K_d$ để xe đi thẳng ổn định nhất.
3.  **Kiểm tra Encoder:** Đảm bảo hướng xoay của bánh xe khớp với chiều tăng/giảm của xung Encoder.

---
**Project:** EMILY AI BOT  
**Author:** [Tên của bạn] - Mechatronics UET  
**Date:** March 2026
