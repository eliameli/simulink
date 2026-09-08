function s_next = unicycle_rk4_step(s, a, w, dt)
%UNICYCLE_RK4_STEP One RK4 step of the 2D kinematic ("unicycle") car model.
%   S_NEXT = UNICYCLE_RK4_STEP(S, A, W, DT) advances state
%   S = [x; y; psi; v] by one step of length DT, given forward
%   (longitudinal) acceleration A and yaw rate W, both held constant
%   over the step (zero-order hold - the standard assumption for
%   discrete-time IMU mechanization).
%
%   Uses classical 4th-order Runge-Kutta instead of a single
%   forward-Euler step, per the assignment requirement to integrate with
%   something other than the Euler method.
%
%   This exact function is used both to generate the ground-truth
%   trajectory (see truth/generate_trajectory.m, with zero sensor error)
%   and, in equivalent form, inside the "INS Mechanization (RK4)"
%   MATLAB Function block of the Simulink model (see
%   sim/blocks/ins_mechanization_block.m) to turn noisy sensor
%   measurements into a dead-reckoned position estimate.
%
% Русское резюме: один шаг метода Рунге-Кутты 4-го порядка для
% кинематической модели "велосипеда" (x,y,курс,скорость). Используется
% и для эталонной траектории (без ошибок), и внутри Simulink-блока
% dead reckoning (с зашумлёнными данными датчика).

k1 = deriv(s,          a, w);    % наклон (производная состояния) в начале шага
k2 = deriv(s + dt/2*k1, a, w);   % наклон в середине шага, оценённый по k1
k3 = deriv(s + dt/2*k2, a, w);   % наклон в середине шага, уточнённый по k2
k4 = deriv(s + dt*k3,   a, w);   % наклон в конце шага, оценённый по k3
s_next = s + dt/6*(k1 + 2*k2 + 2*k3 + k4); % итоговый шаг - взвешенное среднее четырёх наклонов (формула RK4)
end

function ds = deriv(s, a, w)
% Правая часть уравнений движения: производные состояния по времени.
psi = s(3);    % курс (текущий угол направления движения)
v   = s(4);    % скорость
ds = [v*cos(psi); v*sin(psi); w; a]; % [dx/dt; dy/dt; dpsi/dt; dv/dt]
end
