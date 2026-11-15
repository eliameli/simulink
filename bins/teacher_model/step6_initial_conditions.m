%% step6_initial_conditions.m — начальные условия для блока БИНС (реальные данные, телефон B)
%
% По заданию:
%   - крен и тангаж - из среднего вектора гравитации за первые секунды
%     стоянки;
%   - курс - из GPS (Direction) на первом участке уверенного движения;
%   - начальные скорость и положение - 0 на стоянке.
%
% Проблема: "стоянка" (где мерим крен/тангаж) и "первое уверенное
% движение по GPS" (где берём курс) - это РАЗНЫЕ моменты времени. Чтобы
% свести всё к ОДНОМУ начальному моменту (началу стоянки, где v=0),
% "довозим" курс назад во времени интегралом гироскопа - это чисто
% техническая привязка, не отдельный алгоритм ориентации.

clear; clc; close all;

load('step6_real_data.mat');   % t_fixed, a_fixed, w_fixed, C, up_unit, fwd_unit, third_unit, t_good, v_good, dir_good

%% 1) Находим окно "машина стоит" - по гироскопу (на стоянке вращения почти нет)
w_norm = vecnorm(w_fixed, 2, 2);
is_still = w_norm < 0.03;   % рад/с (~1.7 град/с) - порог "почти не крутится"

% ищем самый длинный "спокойный" участок В НАЧАЛЕ записи (не вообще
% самый длинный по всей записи - машина могла постоять и позже, на
% светофоре, это не то же самое "начало стоянки")
idx = find(~is_still, 1, 'first') - 1;   % до первого заметного вращения
if isempty(idx) || idx < 50
    idx = 200;   % запасной вариант - первые 2 с (200 отсчётов при 0.01с), если авто-поиск не сработал
end
idx_start = 1;
idx_end_still = idx;

fprintf('Окно "стоянка": %.2f - %.2f с (%d отсчётов)\n', ...
    t_fixed(idx_start), t_fixed(idx_end_still), idx_end_still - idx_start + 1);

%% 2) Крен и тангаж - из среднего ускорения на этом окне
% В осях машины (X-вперёд, Y-вверх, Z=вперёд×вверх), на стоянке истинное
% ускорение = 0, значит измеренное a_fixed = только гравитация, и по
% формуле (см. обсуждение в чате, T_ba = R_y(пси)*R_z(тета)*R_x(гамма)):
%   a_измеренное / |g| = [sin(тета), cos(тета)cos(гамма), -cos(тета)sin(гамма)]
a_rest = mean(a_fixed(idx_start:idx_end_still, :), 1);
a_rest_dir = a_rest / norm(a_rest);

theta0 = asin(a_rest_dir(1));                          % тангаж
gamma0 = atan2(-a_rest_dir(3), a_rest_dir(2));         % крен

fprintf('Средний |a| на стоянке: %.3f м/с^2 (ожидаем ~10, т.к. в модели g=10)\n', norm(a_rest));
fprintf('Тангаж тета0 = %.2f°, крен гамма0 = %.2f°\n', rad2deg(theta0), rad2deg(gamma0));

%% 3) Курс - из GPS на первом участке уверенного движения (v > 2 м/с)
idx_move = find(v_good > 2, 1, 'first');
if isempty(idx_move)
    error('Не нашёл ни одной GPS-точки с v>2 м/с - проверьте данные/порог.');
end
t_course_anchor = t_good(idx_move);
dir_compass_deg = dir_good(idx_move);   % 0=север, растёт по часовой (компасная конвенция)

% psi в модели - это поворот вокруг Y (вверх) в R_y(псi); как мы разобрали
% в чате: psi_модель = -курс_компас (математическое CCW-направление
% против часовой стрелки, компас - по часовой).
psi_at_anchor = -deg2rad(dir_compass_deg);

fprintf('Точка привязки курса: t=%.2f с, GPS Direction=%.1f°\n', t_course_anchor, dir_compass_deg);

%% 4) "Довозим" курс назад к началу стоянки интегралом гироскопа
% wz - угловая скорость вокруг "вверх" (это и есть компонента w_fixed(:,2),
% т.к. столбец 2 - это как раз ось "вверх" в нашем базисе C).
wz_fixed = w_fixed(:, 2);
raw_integral = cumtrapz(t_fixed, wz_fixed);               % интеграл от t_fixed(1) (всегда 0 в начале)
integral_at_anchor = interp1(t_fixed, raw_integral, t_course_anchor);

% psi(t) = psi_at_anchor - (raw_integral(t) - integral_at_anchor)
% в момент начала стоянки: psi_start = psi_at_anchor - (0 - integral_at_anchor)
psi0 = psi_at_anchor + integral_at_anchor;

fprintf('Курс псi0 в начале стоянки (t=%.2f с) = %.2f°\n', t_fixed(idx_start), rad2deg(mod(psi0+pi,2*pi)-pi));

%% 5) Начальные скорость и положение - ноль (машина стоит, точка отсчёта)
v0 = [0 0 0];
r0 = [0 0 0];

%% 6) Обрезаем данные - подаём в БИНС начиная с начала стоянки (а не со
%     старта всей записи, где могли быть лишние манипуляции с телефоном)
t_run = t_fixed(idx_start:end) - t_fixed(idx_start);   % время с нуля
a_run = a_fixed(idx_start:end, :);
w_run = w_fixed(idx_start:end, :);

fprintf('\nИтоговые начальные условия для блока БИНС:\n');
fprintf('  start angles = [тета, псi, гамма] = [%.4f, %.4f, %.4f] рад\n', theta0, psi0, gamma0);
fprintf('  start v = [0 0 0],  start r = [0 0 0]\n');
fprintf('  длительность прогона: %.1f с\n', t_run(end));

save('step6_initial_conditions.mat', 't_run', 'a_run', 'w_run', ...
    'theta0', 'psi0', 'gamma0', 'v0', 'r0', 't_course_anchor', 'idx_move');
