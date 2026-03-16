%% sim_amr_2d_vw_target.m
% Mo phong AMR 2 banh voi 3 input:
%   1) v  - van toc tinh tien (mm/s)
%   2) w  - van toc goc (rad/s)
%   3) target_distance - quang duong muc tieu (mm)
%
% Dua tren thong so tu file .ino:
%   WHEEL_DIAMETER = 66 mm
%   WHEEL_BASE     = 205 mm
%   ENCODER_PPR    = 495
%
% Mo phong:
% - Chuyen dong 2D cua robot (ground truth)
% - Encoder trai/phai
% - IMU gyro Z + tich phan yaw
% - Odometry estimate tu encoder + IMU
% - Animation robot di chuyen

clear; clc; close all;
rng(2);

%% ================== INPUT ==================
v_cmd = 200;          % mm/s
w_cmd = 0;          % rad/s
target_distance = 1000; % mm

%% ================== THONG SO ROBOT ==================
WHEEL_DIAMETER = 66.0;      % mm
WHEEL_BASE     = 205.0;     % mm
ENCODER_PPR    = 495.0;     % pulse/rev
R_wheel        = WHEEL_DIAMETER / 2.0;

Ts   = 0.02;   % 20 ms, giong timer trong .ino
Tmax = 15;     % s
N    = floor(Tmax / Ts) + 1;

%% ================== THAM SO MO PHONG THUC TE ==================
% Lech 2 banh nhe de di chuyen "nhu ngoai doi" hon
motorGainL = 0.99;
motorGainR = 1.00;

% Encoder noise (tick)
encoderNoiseStd = 0.05;

% IMU gyro Z noise/bias
gyroBiasZ_dps  = 0.15;   % deg/s
gyroNoiseZ_dps = 0.12;   % deg/s

%% ================== THAM SO PID (giong Odometry.ino) ==================
MAX_SPEED_MMS = 449.0;   % mm/s -> 255 PWM
Kp  = 5;   Kd  = 0;      % PID encoder
Kpw = 7;   Kdw = 0;      % PID IMU
error_prev   = 0;
error_w_prev = 0;

%% ================== KHOI TAO GROUND TRUTH ==================
x_true = 0;        % mm
y_true = 0;        % mm
th_true = 0;       % rad
s_true = 0;        % mm

%% ================== KHOI TAO SENSOR ==================
encL = 0;
encR = 0;

yaw_imu_deg = 0;       % deg
gyroZ_meas_dps = 0;    % deg/s

%% ================== KHOI TAO ODOMETRY ==================
x_est = 0;
y_est = 0;
th_est = 0;            % rad
s_est = 0;             % mm

prev_encL = 0;
prev_encR = 0;

%% ================== LOG ==================
t_log      = zeros(N,1);

x_true_log = zeros(N,1);
y_true_log = zeros(N,1);
th_true_log= zeros(N,1);
s_true_log = zeros(N,1);

x_est_log  = zeros(N,1);
y_est_log  = zeros(N,1);
th_est_log = zeros(N,1);
s_est_log  = zeros(N,1);

encL_log   = zeros(N,1);
encR_log   = zeros(N,1);

yaw_imu_log= zeros(N,1);
gyro_log   = zeros(N,1);

vL_log     = zeros(N,1);
vR_log     = zeros(N,1);

pwmL_log   = zeros(N,1);
pwmR_log   = zeros(N,1);
corr_enc_log = zeros(N,1);
corr_imu_log = zeros(N,1);

stop_idx = N;

%% ================== VONG LAP MO PHONG ==================
for k = 1:N
    t = (k-1)*Ts;
    t_log(k) = t;

    % ----- Dieu kien dung theo quang duong muc tieu -----
    if s_true >= target_distance
        stop_idx = k;
        % log trang thai cuoi roi break
        x_true_log(k)  = x_true;
        y_true_log(k)  = y_true;
        th_true_log(k) = th_true;
        s_true_log(k)  = s_true;

        x_est_log(k)   = x_est;
        y_est_log(k)   = y_est;
        th_est_log(k)  = th_est;
        s_est_log(k)   = s_est;

        encL_log(k)    = encL;
        encR_log(k)    = encR;
        yaw_imu_log(k) = yaw_imu_deg;
        gyro_log(k)    = gyroZ_meas_dps;
        vL_log(k)      = 0;
        vR_log(k)      = 0;
        break;
    end

    % =========================================================
    % 1) TINH VAN TOC BANH TU INPUT (giong Odometry.ino)
    % =========================================================
    vR_cmd = v_cmd + (w_cmd * WHEEL_BASE)/2;
    vL_cmd = v_cmd - (w_cmd * WHEEL_BASE)/2;

    % ----- velocityToPWM (giong .ino) -----
    basePWM_L = round((vL_cmd / MAX_SPEED_MMS) * 255);
    basePWM_R = round((vR_cmd / MAX_SPEED_MMS) * 255);
    basePWM_L = max(min(basePWM_L, 255), -255);
    basePWM_R = max(min(basePWM_R, 255), -255);

    % ----- PID Encoder: enc_error = delta_encL - delta_encR -----
    enc_error = (encL - prev_encL) - (encR - prev_encR);
    derivative_enc = (enc_error - error_prev) / Ts;
    correction_enc = Kp * enc_error + Kd * derivative_enc;
    correction_enc = max(min(correction_enc, 100), -100);
    error_prev = enc_error;

    % ----- PID IMU: so sanh GOC voi GOC (giong .ino) -----
    % Trong .ino: setpoint = w (= 0 khi di thang, la GOC setpoint, don vi: do)
    %             actual_w = roll (goc heading tu IMU, don vi: do)
    setpoint_heading = w_cmd;              % = 0 khi di thang (goc setpoint, do)
    actual_heading   = yaw_imu_deg;        % Goc heading IMU tich luy (do)
    w_error = setpoint_heading - actual_heading;
    derivative_w = (w_error - error_w_prev) / Ts;
    correction_imu = Kpw * w_error + Kdw * derivative_w;
    correction_imu = max(min(correction_imu, 100), -100);
    error_w_prev = w_error;

    % ----- Tong hop correction (giong .ino) -----
    total_correction = correction_enc + correction_imu;

    pwmL = max(min(basePWM_L - round(total_correction), 255), -255);
    pwmR = max(min(basePWM_R + round(total_correction), 255), -255);

    % ----- Chuyen PWM -> van toc thuc (nghich dao velocityToPWM) -----
    vL_actual = (pwmL / 255.0) * MAX_SPEED_MMS;
    vR_actual = (pwmR / 255.0) * MAX_SPEED_MMS;

    % ----- Them sai lech banh cho giong thuc te -----
    vL_true = motorGainL * vL_actual;
    vR_true = motorGainR * vR_actual;

    v_true = (vR_true + vL_true)/2;
    w_true = (vR_true - vL_true)/WHEEL_BASE;

    % Cap nhat pose that bang midpoint integration
    th_mid = th_true + 0.5*w_true*Ts;
    x_true = x_true + v_true*cos(th_mid)*Ts;
    y_true = y_true + v_true*sin(th_mid)*Ts;
    th_true = th_true + w_true*Ts;
    s_true = s_true + abs(v_true)*Ts;

    % =========================================================
    % 2) ENCODER MODEL: sinh xung encoder tu banh that
    % =========================================================
    dL_true = vL_true * Ts;   % mm
    dR_true = vR_true * Ts;   % mm

    deltaTickL = dL_true/(pi*WHEEL_DIAMETER) * ENCODER_PPR;
    deltaTickR = dR_true/(pi*WHEEL_DIAMETER) * ENCODER_PPR;

    deltaTickL = deltaTickL + encoderNoiseStd*randn();
    deltaTickR = deltaTickR + encoderNoiseStd*randn();

    encL = encL + round(deltaTickL);
    encR = encR + round(deltaTickR);

    % =========================================================
    % 3) IMU MODEL: gyro Z + tich phan yaw
    % =========================================================
    gyroZ_true_dps = rad2deg(w_true);
    gyroZ_meas_dps = gyroZ_true_dps + gyroBiasZ_dps + gyroNoiseZ_dps*randn();
    yaw_imu_deg = yaw_imu_deg + gyroZ_meas_dps*Ts;

    % =========================================================
    % 4) ODOMETRY ESTIMATION: encoder + IMU
    % =========================================================
    dEncL = encL - prev_encL;
    dEncR = encR - prev_encR;
    prev_encL = encL;
    prev_encR = encR;

    dsL = (dEncL / ENCODER_PPR) * (pi*WHEEL_DIAMETER);
    dsR = (dEncR / ENCODER_PPR) * (pi*WHEEL_DIAMETER);
    ds  = (dsL + dsR)/2;

    th_new = deg2rad(yaw_imu_deg);
    th_mid_est = 0.5*(th_est + th_new);

    x_est = x_est + ds*cos(th_mid_est);
    y_est = y_est + ds*sin(th_mid_est);
    th_est = th_new;
    s_est = s_est + abs(ds);

    % =========================================================
    % 5) LOG
    % =========================================================
    x_true_log(k)  = x_true;
    y_true_log(k)  = y_true;
    th_true_log(k) = th_true;
    s_true_log(k)  = s_true;

    x_est_log(k)   = x_est;
    y_est_log(k)   = y_est;
    th_est_log(k)  = th_est;
    s_est_log(k)   = s_est;

    encL_log(k)    = encL;
    encR_log(k)    = encR;
    yaw_imu_log(k) = yaw_imu_deg;
    gyro_log(k)    = gyroZ_meas_dps;

    vL_log(k)      = vL_true;
    vR_log(k)      = vR_true;

    pwmL_log(k)    = pwmL;
    pwmR_log(k)    = pwmR;
    corr_enc_log(k)= correction_enc;
    corr_imu_log(k)= correction_imu;
end

% Cat log den diem dung
t_log       = t_log(1:stop_idx);
x_true_log  = x_true_log(1:stop_idx);
y_true_log  = y_true_log(1:stop_idx);
th_true_log = th_true_log(1:stop_idx);
s_true_log  = s_true_log(1:stop_idx);

x_est_log   = x_est_log(1:stop_idx);
y_est_log   = y_est_log(1:stop_idx);
th_est_log  = th_est_log(1:stop_idx);
s_est_log   = s_est_log(1:stop_idx);

encL_log    = encL_log(1:stop_idx);
encR_log    = encR_log(1:stop_idx);
yaw_imu_log = yaw_imu_log(1:stop_idx);
gyro_log    = gyro_log(1:stop_idx);
vL_log      = vL_log(1:stop_idx);
vR_log      = vR_log(1:stop_idx);
pwmL_log    = pwmL_log(1:stop_idx);
pwmR_log    = pwmR_log(1:stop_idx);
corr_enc_log= corr_enc_log(1:stop_idx);
corr_imu_log= corr_imu_log(1:stop_idx);

%% ================== IN KET QUA ==================
fprintf('===== KET QUA MO PHONG =====\n');
fprintf('Input: v = %.2f mm/s, w = %.4f rad/s, target = %.2f mm\n', ...
    v_cmd, w_cmd, target_distance);
fprintf('Thoi gian chay: %.3f s\n', t_log(end));
fprintf('Quang duong that: %.2f mm\n', s_true_log(end));
fprintf('Quang duong estimate: %.2f mm\n', s_est_log(end));
fprintf('Pose that cuoi: x = %.2f mm, y = %.2f mm, theta = %.2f deg\n', ...
    x_true_log(end), y_true_log(end), rad2deg(th_true_log(end)));
fprintf('Pose estimate cuoi: x = %.2f mm, y = %.2f mm, theta = %.2f deg\n', ...
    x_est_log(end), y_est_log(end), rad2deg(th_est_log(end)));
fprintf('Sai so vi tri cuoi = %.2f mm\n', ...
    hypot(x_true_log(end)-x_est_log(end), y_true_log(end)-y_est_log(end)));

%% ================== VE QUY DAO ==================
figure('Color','w','Name','Trajectory');
plot(x_true_log, y_true_log, 'b-', 'LineWidth', 2); hold on;
plot(x_est_log,  y_est_log,  'r--', 'LineWidth', 2);
plot(x_true_log(1), y_true_log(1), 'go', 'MarkerFaceColor','g');
plot(x_true_log(end), y_true_log(end), 'ks', 'MarkerFaceColor','k');
grid on; axis equal;
xlabel('x (mm)');
ylabel('y (mm)');
legend('True trajectory','Estimated trajectory','Start','Stop','Location','best');
title('Quy dao 2D cua robot');

%% ================== VE CAC DO THI KHAC ==================
figure('Color','w','Name','Distance and Heading');
subplot(2,1,1);
plot(t_log, s_true_log, 'b', 'LineWidth', 1.8); hold on;
plot(t_log, s_est_log,  'r--', 'LineWidth', 1.8);
yline(target_distance, 'k:', 'Target');
grid on;
xlabel('Time (s)');
ylabel('Distance (mm)');
legend('s true','s est','target','Location','best');
title('Quang duong');

subplot(2,1,2);
plot(t_log, rad2deg(th_true_log), 'b', 'LineWidth', 1.8); hold on;
plot(t_log, yaw_imu_log, 'r--', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\theta true','yaw IMU','Location','best');
title('Heading');

figure('Color','w','Name','Encoder and Wheel Speed');
subplot(2,1,1);
plot(t_log, encL_log, 'LineWidth', 1.8); hold on;
plot(t_log, encR_log, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Ticks');
legend('Encoder Left','Encoder Right','Location','best');
title('Gia tri encoder');

subplot(2,1,2);
plot(t_log, vL_log, 'LineWidth', 1.8); hold on;
plot(t_log, vR_log, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Wheel speed (mm/s)');
legend('v_L','v_R','Location','best');
title('Van toc 2 banh');

%% ================== VE DO THI PID ==================
figure('Color','w','Name','PID Control');
subplot(3,1,1);
plot(t_log, pwmL_log, 'LineWidth', 1.8); hold on;
plot(t_log, pwmR_log, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('PWM');
legend('PWM Left','PWM Right','Location','best');
title('Gia tri PWM sau PID');

subplot(3,1,2);
plot(t_log, corr_enc_log, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Correction');
title('PID Encoder Correction');

subplot(3,1,3);
plot(t_log, corr_imu_log, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Correction');
title('PID IMU Correction');

%% ================== ANIMATION 2D ==================
figure('Color','w','Name','2D Animation');
hold on; grid on; axis equal;

xmin = min([x_true_log; x_est_log]) - 200;
xmax = max([x_true_log; x_est_log]) + 200;
ymin = min([y_true_log; y_est_log]) - 200;
ymax = max([y_true_log; y_est_log]) + 200;

if abs(ymax - ymin) < 200
    ymin = ymin - 100;
    ymax = ymax + 100;
end

axis([xmin xmax ymin ymax]);
xlabel('x (mm)');
ylabel('y (mm)');
title('Mo phong robot 2 banh vi sai');

hTruePath = plot(NaN, NaN, 'b-', 'LineWidth', 2);
hEstPath  = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);

% Robot components
hBody     = patch(NaN, NaN, [0.75 0.85 0.95], 'EdgeColor', [0.2 0.3 0.5], 'LineWidth', 1.5, 'FaceAlpha', 0.9);
hWheelL   = patch(NaN, NaN, [0.2  0.2  0.2 ], 'EdgeColor', 'k', 'LineWidth', 1.2);
hWheelR   = patch(NaN, NaN, [0.2  0.2  0.2 ], 'EdgeColor', 'k', 'LineWidth', 1.2);
hCaster   = plot(NaN, NaN, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', [0.4 0.4 0.4]);
hArrow    = plot(NaN, NaN, 'r-', 'LineWidth', 2.5);
hArrowHead= patch(NaN, NaN, [0.9 0.1 0.1], 'EdgeColor', 'none');

legend('True path','Estimated path','Location','best');

% Kich thuoc robot (mm) — tuong ung thong so thuc
bodyL = 140;   bodyW = 120;    % Than xe
wheelL = 40;   wheelW = 16;    % Banh xe
wb2 = WHEEL_BASE / 2;          % Nua khoang cach 2 banh
casterR = 8;                   % Ban kinh banh tu do

for k = 1:3:length(t_log)
    set(hTruePath, 'XData', x_true_log(1:k), 'YData', y_true_log(1:k));
    set(hEstPath,  'XData', x_est_log(1:k),  'YData', y_est_log(1:k));

    xc = x_true_log(k);
    yc = y_true_log(k);
    th = th_true_log(k);

    R = [cos(th) -sin(th); sin(th) cos(th)];

    % --- Than xe (hinh chu nhat bo goc) ---
    bx = [-bodyL/2  bodyL/2  bodyL/2 -bodyL/2];
    by = [-bodyW/2 -bodyW/2  bodyW/2  bodyW/2];
    body = R * [bx; by] + [xc; yc];
    set(hBody, 'XData', body(1,:), 'YData', body(2,:));

    % --- Banh trai (hinh chu nhat, dat o y = +wb2) ---
    wlx = [-wheelL/2  wheelL/2  wheelL/2 -wheelL/2];
    wly = [wb2-wheelW/2  wb2-wheelW/2  wb2+wheelW/2  wb2+wheelW/2];
    wL = R * [wlx; wly] + [xc; yc];
    set(hWheelL, 'XData', wL(1,:), 'YData', wL(2,:));

    % --- Banh phai (hinh chu nhat, dat o y = -wb2) ---
    wrx = wlx;
    wry = [-wb2-wheelW/2  -wb2-wheelW/2  -wb2+wheelW/2  -wb2+wheelW/2];
    wR = R * [wrx; wry] + [xc; yc];
    set(hWheelR, 'XData', wR(1,:), 'YData', wR(2,:));

    % --- Banh tu do phia truoc ---
    casterPos = R * [bodyL/2 - 15; 0] + [xc; yc];
    set(hCaster, 'XData', casterPos(1), 'YData', casterPos(2));

    % --- Mui ten chi huong ---
    arrowStart = R * [bodyL/4; 0] + [xc; yc];
    arrowEnd   = R * [bodyL/2 + 30; 0] + [xc; yc];
    set(hArrow, 'XData', [arrowStart(1) arrowEnd(1)], ...
                'YData', [arrowStart(2) arrowEnd(2)]);

    % Dau mui ten (tam giac nho)
    ahSize = 15;
    ah1 = R * [bodyL/2 + 30; 0] + [xc; yc];
    ah2 = R * [bodyL/2 + 15; ahSize/2] + [xc; yc];
    ah3 = R * [bodyL/2 + 15; -ahSize/2] + [xc; yc];
    set(hArrowHead, 'XData', [ah1(1) ah2(1) ah3(1)], ...
                    'YData', [ah1(2) ah2(2) ah3(2)]);

    drawnow;
    pause(0.01);
end