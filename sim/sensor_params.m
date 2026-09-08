function p = sensor_params()
%SENSOR_PARAMS Simple MEMS-grade IMU error-model parameters for v0.
%   Representative orders of magnitude for a low-cost automotive/consumer
%   MEMS IMU - NOT taken from a specific datasheet. Tune these to see how
%   sensor quality affects dead-reckoning drift and how much map matching
%   is able to compensate.
%
%   Accelerometer:
%     accel_bias0_std   - turn-on (constant) bias                 [m/s^2]
%     accel_bias_rw_std - bias random-walk rate (per sqrt(s))     [m/s^2 / sqrt(s)]
%     accel_noise_std   - white measurement noise, 1-sigma        [m/s^2]
%   Gyroscope:
%     gyro_bias0_std    - turn-on (constant) bias                 [rad/s]
%     gyro_bias_rw_std  - bias random-walk rate (per sqrt(s))     [rad/s / sqrt(s)]
%     gyro_noise_std    - white measurement noise, 1-sigma        [rad/s]
%
% Русское резюме: параметры модели ошибок акселерометра и гироскопа.
% Порядок величины типичен для дешёвого MEMS-датчика, не привязан к
% конкретной модели/даташиту - можно менять и смотреть, как сильно
% ошибка датчика влияет на уход dead reckoning и насколько map matching
% способен это компенсировать.

p.accel_bias0_std   = 0.05;         % ~5 cm/s^2 constant offset
% ^ м/с^2 - постоянное смещение акселерометра "при включении"
p.accel_bias_rw_std = 0.001;        % slow drift of the bias over time
% ^ м/с^2/√с - скорость медленного "уплывания" смещения со временем
p.accel_noise_std   = 0.02;         % per-sample measurement noise
% ^ м/с^2 - случайный шум измерения на каждом отсчёте

p.gyro_bias0_std   = deg2rad(0.5);  % ~0.5 deg/s constant offset
% ^ рад/с (~0.5 град/с) - постоянное смещение гироскопа "при включении"
p.gyro_bias_rw_std = deg2rad(0.02); % slow drift of the bias over time
% ^ рад/с/√с - скорость медленного "уплывания" смещения гироскопа со временем
p.gyro_noise_std   = deg2rad(0.1);  % per-sample measurement noise
% ^ рад/с - случайный шум измерения гироскопа на каждом отсчёте
end
