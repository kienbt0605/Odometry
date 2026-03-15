**********************************************************************ODOMETRY MOTION**********************************************************************
|_____________________________________________________________________________________________________________________________________________________________|


*******[CÁC_TAB_TRONG_CODE]*******
<OVERVIEW>

[1] Chuyển động của EMILY AI BOT được tích hợp động học Odometry dựa trên phản hồi từ bộ mã hóa (encoder) 2 bánh
[2] Trên BabyCore, Master RaspberryPi truyền thông với GateWay ESP32 và sử dụng ESPNOW để truyền thông xuống Client ESP32 của DRIVERCORE để thực hiện chuyển động
[3] 2 thông số để chuyển động là v (Velocity) và w (Angular Velocity), 2 thông số sẽ được tính toán trên phần Master và truyền xuống DRIVERCORE để thực thi chuyển động


<CODE FUNCTION EXPLANATION>

***********************************************************************[ODOMETRY_VER_X]***********************************************************************

[1] CONFIGURATION
//GPIO CONFIGURATION
- Khai báo các thông số của xe WHEEL_DIAMETER (Đường kính bánh xe), WHEEL_BASE (Khoảng cách giữa tâm 2 bánh xe), ENCODER_PPR (Số xung encoder đếm được trên 1 vòng xoay bánh)
- Khai báo các chân của driver động cơ Driver L298N <[IN1]>, <[IN2]>, <[IN3]>, <[IN4]>, <[ENA]>, <[ENB]>
- Khai báo các chân tín hiệu encoder <[ENCODER_1A]>, <[ENCODER_1B]>, <[ENCODER_2A]>, <[ENCODER_2B]>
- Khai báo số PI phục vụ cho việc tính toán góc radian cho w

//VARIABLE CONFIGURATION
- struct gồm 2 giá trị v và w nhận giá trị từ GateWay ESP32 thông qua ESPNOW
- Khai báo cờ odmFlag để chạy hàm <Odometry()> trong <loop()>
- Khai báo 2 giá trị encoder đọc được

[2]FUCNTION
- Hàm ngắt để mỗi 20ms gọi cờ odomflag một lần để chạy hàm <Odometry()> trong <loop()>
- ESPNOW để nhận dữ liệu từ ESP32 GateWay
- Hàm Odometry trong <loop()> chạy theo chu kỳ set cờ odomflag của hàm ngắt

***********************************************************************[ENCODER]***********************************************************************
- Đọc giá trị của encoder qua 2 hàm <encoder1_isr()> và <encoder2_isr()> tương ứng với bánh trái và bánh phải

***********************************************************************[MOTOR]***********************************************************************
- Là hàm để truyền trực tiếp PWM từ -255 đến 255 để chạy động cơ, chỉ đơn thuần là chạy động cơ, chưa có điều kiện trả về để thực thi chuyển động đặc biệt

***********************************************************************[RX]***********************************************************************
- Hàm nhận dữ liệu từ ESP32 GateWay thông qua ESPNOW

***********************************************************************[ODOMETRY]***********************************************************************

[1]CONFIGURATION
- <[R_wheel]> là bán kính của bánh xe
- <[DT]> là chu kỳ lấy mẫu 1s (1000ms)
- Khai báo các hệ số khuyếch đại <[Kp]>, <[Kd]>, <[Ki]>. Không sử dụng hệ số Ki vì do 2 bán luôn luôn có sự sai lệch dù ít hay nhiều nhưng không ổn định, không phải steady state error
- <[prev_encoder1]> và <[previos_encoder2]> phục vụ cho hàm <Odometry()>

[2]FUNCTION
- Hàm <velocityToPWM()> để chuyển đổi giá trị vận tốc thành giá trị PWM để sau này qua các bước chuyển đổi khác và cấp cho hàm Motor()
- Hàm <PID_Correction(float error)> trả về một giá trị để bù trừ vào 2 bánh trái phải, để giúp xe đi cân bằng hơn, đối số <[error]> tính sự thay đổi giữa các giá trị mã hóa của hai encoder (bánh xe trái và phải), sau đó tính sai số giữa chúng.
- Hàm <Odometry>
    + Sử dụng 2 giá trị chính để chuyển động cho xe là v và w
    + Sử dụng những công thức của động học để tính toán ra v và w   
    + Khai báo 2 giá trị basePWM_L và basePWM_R được gán bằng lần lượt <velocityToPWM(vLeft)> và <velocityToPWM(vRight)> với <[vLeft]> và <[vRight]> được tính bằng công thức của động học v = +- (w * WHEEL_BASE) / 2.0
    + Gán 2 giá trị encoder hiện tại và encoder đọc được, <enc1_now = encoder1_value> <enc2_now = encoder2_value>
    + Tính sự thay đổi của giá trị encoder bánh trái và encoder bánh phải qua 2 biến <delta_encL> và <delta_encR>
    + <[enc_error]> là sai số giữa <[delta_encL]> và <[delta_encR]>, là giá trị để truyền vào đối số của hàm <PID_Correction>. Phải ép kiểu float vì đối số <[error]> để khai báo float
    + Khai báo biến <[correction]> được gán bằng hàm <PID_Correction> với đối số truyền vào là enc_error để thực hiện thuật toán PID
    + Giá trị <[pwmL]> và <[pwmR]> được tính lần lượt bằng <basePWM_L - correction> và <basePWM_R + correction>, ,[correction]> là giá trị được return về từ hàm PID_Correction đã được tính toán, giúp bù trừ sai lệch cho 2 bánh trái phải, lệch trái bù trái, lệch phải bù phải
    + Truyền 2 đối số <[pwmL]> và <[pwmR]> cho hàm <Motor> để thực thi chuyển động, đáp ứng cơ cấu chấp hành của xe






