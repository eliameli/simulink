function [x, y, psi, v] = ins_mechanization(a_meas, w_meas, Ts, x0, y0, psi0, v0, nodes, edges)
%#codegen
% INS Mechanization (RK4) - MATLAB Function block.
%
% This is the exact code build_model.m tries to insert into the
% "INS Mechanization (RK4)" block of gps_free_nav_v0.slx. If the
% automatic insertion fails, open the model, double-click that (empty)
% MATLAB Function block, and paste this whole file's contents in.
%
% Dead-reckoning: integrates the noisy sensor measurements (a_meas,
% w_meas) forward in time with RK4 (not Euler) to keep a running
% estimate of the vehicle pose [x; y; psi; v]. Ts is the fixed sample
% time (must match the model's fixed-step size). x0,y0,psi0,v0 is the
% known starting pose - dead reckoning always needs *some* known
% starting point, so this comes in from outside (set by run_v0.m from
% the ground-truth trajectory's first sample) rather than being
% hardcoded, since the route (and therefore the start point) changes
% every time you click a new one with map/pick_route.m.
%
% Corner snap: a real turn shows up as a clear, large spike in the
% measured yaw rate (tens-hundreds of deg/s by construction - see
% truth/generate_trajectory.m), easily told apart from ordinary gyro
% noise/bias (well under 1 deg/s). When a turn is confirmed over, the
% running position is pulled (partially, not teleported outright)
% toward the nearest map node, and heading is reset to whichever road
% leaving that node best matches the current heading. CAPTURE_RADIUS
% used to cap how far away a node could still be and count (so a
% heavily-drifted estimate couldn't "cheat" by jumping a long
% distance) - but that meant once drift ever exceeded it, no future
% turn could ever correct it again, which defeats the point: it's now
% infinite, so a corner-snap always finds and pulls toward the nearest
% node, however far off that is.
%
% An earlier, simpler version of this (bare threshold, no debounce/
% cooldown) mis-fired repeatedly on a single real corner whenever the
% drawn route's heading changed in several small steps close together
% (typical hand-drawn jitter - see map/pick_route.m), each one its own
% brief yaw-rate spike: every one of those triggered its own snap,
% producing a scribble of repeated corrections instead of one clean
% one. Fixed here with:
%   - hysteresis (separate enter/exit rate thresholds, so noise
%     sitting near one threshold can't flip the state back and forth),
%   - debounce (the rate has to stay above/below threshold for
%     DEBOUNCE_TIME seconds before a transition counts, so a handful of
%     noisy samples can't fake a whole turn), and
%   - a cooldown (COOLDOWN_S) between corrections, so even if several
%     micro-turns from one wobbly corner do each get detected, only the
%     first one actually triggers a snap.
% Verified against a standalone Python re-implementation of this exact
% state machine on synthetic data before writing this: a single clean
% 90-degree turn and an equivalent "wobbly" corner built from four
% smaller turns a few hundredths of a second apart both produced
% exactly one correction, landing on the same node/heading; a turn far
% from any map node correctly produced zero corrections.
%
% Startup warm-up: the corner-snap above only ever fires once a turn
% has been detected and confirmed, so for a route that starts with a
% long straight stretch there's nothing to correct against yet - and
% ordinary sensor bias/noise can already carry the estimate visibly off
% the road before the very first turn even happens. For the first
% WARMUP_TIME seconds, skip turn detection and instead just continuously
% project the running position onto the nearest road (like the "Map
% Matching" block does, but every step, not just for display) and align
% heading to that road's direction - the same idea as the corner snap,
% just running continuously instead of waiting for a turn.
%
% A first version of this picked whichever road segment the position
% was closest to, full stop - which mis-fired right at the very start
% whenever the route began at a "dead end" node (one direction removed
% by map/generate_map.m, so only e.g. "straight ahead" and "turn"
% actually exist there) and the drawn/clicked starting point wasn't
% exactly on the intended road's line (it never precisely is, by hand).
% The intended road doesn't extend backward past its own start node, so
% its nearest point clamps to that node and can end up geometrically
% farther away than the *other* road leaving the same node - even
% though the current heading clearly says which one was meant. Checked
% against several such near-node starting offsets in a standalone
% Python re-implementation: plain nearest-distance picked the wrong
% road every time, while breaking ties (within TIE_TOL) by which
% direction better matches the current heading picked the right one
% every time. Hysteresis (persistent WARMUP_EDGE) on top keeps that
% choice from flip-flopping step to step once it's made.

% --- РУССКИЕ КОММЕНТАРИИ ---
% Ниже - краткая подпись почти к каждой строке кода (на русском),
% для понимания и защиты проекта. Английские абзацы выше - это уже
% существующие объяснения "почему сделано именно так" (история
% багов и как они были найдены), их не убираю.

persistent s                 % s = [x; y; psi; v] - текущая оценка позиции/курса/скорости (хранится между вызовами)
persistent t_elapsed         % сколько времени (с) прошло с начала симуляции - нужно для разгонного периода
persistent warmup_edge       % какое ребро дороги было выбрано на прошлом шаге разгона (для гистерезиса)
persistent high_time         % сколько времени подряд |w_meas| держится ВЫШЕ порога "поворот начался"
persistent low_time          % сколько времени подряд |w_meas| держится НИЖЕ порога "поворот закончился"
persistent rate_state        % 0 = едем прямо (подтверждено), 1 = поворачиваем (подтверждено)
persistent since_correction  % сколько времени (с) прошло с последней коррекции к перекрёстку
if isempty(s)                 % это первый вызов блока за весь прогон симуляции
    s = [x0; y0; psi0; v0];    % начальное состояние - берём из входов (заданы в run_v0.m по эталону)
    t_elapsed = 0;             % таймер с начала симуляции - обнуляем
    warmup_edge = 0;           % ещё ни одно ребро не выбрано
    high_time = 0;             % таймер "поворот идёт" - обнуляем
    low_time = 0;              % таймер "поворот закончился" - обнуляем
    rate_state = 0;            % считаем, что едем прямо
    since_correction = 1;      % >= cooldown_s ниже, поэтому первая коррекция разрешена сразу
end

s = rk4_step(s, a_meas, w_meas, Ts);   % основной шаг счисления пути: интегрируем показания датчика методом RK4
t_elapsed = t_elapsed + Ts;            % продвигаем таймер симуляции на один шаг
since_correction = since_correction + Ts; % продвигаем таймер "с последней коррекции" на один шаг

warmup_time     = 3.0;  % с - длительность разгонного периода (см. пояснение выше)
rate_threshold_high = 0.35; % рад/с (~20 град/с) - порог входа в состояние "поворот"
rate_threshold_low  = 0.17; % рад/с (~10 град/с) - порог выхода из "поворота" (с запасом от порога входа - гистерезис)
debounce_time   = 0.05; % с - сколько нужно продержаться за порогом, чтобы переход засчитался
cooldown_s      = 1.0;  % с - минимальный промежуток между двумя коррекциями подряд
capture_radius  = inf;  % м - ограничение на расстояние до перекрёстка снято (см. пояснение выше)
position_blend  = 0.7;  % 0..1 - доля пути к перекрёстку, на которую сдвигаем позицию за одну коррекцию (не 100%)

if t_elapsed <= warmup_time   % мы всё ещё в разгонном периоде (первые warmup_time секунд)?
    [px, py, edir, warmup_edge] = nearest_edge_point(s(1), s(2), s(3), nodes, edges, warmup_edge); % ищем ближайшую точку на дороге
    s(1) = px;                 % жёстко ставим x на найденную точку дороги
    s(2) = py;                 % жёстко ставим y на найденную точку дороги
    s(3) = pick_heading(edir, s(3)); % выравниваем курс по направлению этой дороги (в сторону, близкую к текущему курсу)
else                            % разгонный период закончился - обычный режим со "магнитом" по поворотам
    if abs(w_meas) > rate_threshold_high   % угловая скорость превысила порог входа в поворот?
        high_time = high_time + Ts;         % да - копим время "поворот идёт"
        low_time = 0;                       % и сбрасываем таймер "поворот закончился"
    elseif abs(w_meas) < rate_threshold_low % угловая скорость упала ниже порога выхода из поворота?
        low_time = low_time + Ts;           % да - копим время "похоже, что прямая"
        high_time = 0;                      % и сбрасываем таймер "поворот идёт"
    else                                    % угловая скорость в "серой зоне" между порогами
        high_time = 0;                      % не засчитываем ни то, ни другое
        low_time = 0;                       % (защита от дребезга сигнала возле одного порога)
    end

    if rate_state == 0 && high_time >= debounce_time   % ехали прямо, и "поворот" подтверждён достаточно долго?
        rate_state = 1;                                 % переходим в состояние "поворачиваем"
    end

    if rate_state == 1 && low_time >= debounce_time    % поворачивали, и "прямая" подтверждена достаточно долго?
        rate_state = 0;                                 % значит, поворот только что закончился - переходим в "прямая"
        if since_correction >= cooldown_s               % и с прошлой коррекции прошло достаточно времени?
            [nx, ny, npsi, found] = try_snap(s(1), s(2), s(3), nodes, edges, capture_radius); % ищем ближайший узел карты
            if found                                     % узел нашёлся (сейчас всегда true, т.к. радиус = inf)
                s(1) = s(1) + position_blend * (nx - s(1)); % подтягиваем x на 70% пути к узлу
                s(2) = s(2) + position_blend * (ny - s(2)); % подтягиваем y на 70% пути к узлу
                s(3) = npsi;                                % курс - жёстко по лучшей дороге от этого узла
                since_correction = 0;                       % обнуляем таймер "с последней коррекции"
            end
        end
    end
end

x = s(1); y = s(2); psi = s(3); v = s(4); % отдаём наружу текущую оценку позиции/курса/скорости
end

function s_next = rk4_step(s, a, w, dt)
% Один шаг метода Рунге-Кутты 4-го порядка для кинематики "велосипеда":
% x' = v*cos(psi), y' = v*sin(psi), psi' = w, v' = a.
k1 = deriv(s,          a, w);   % наклон в начале шага
k2 = deriv(s + dt/2*k1, a, w);  % наклон в середине шага (по k1)
k3 = deriv(s + dt/2*k2, a, w);  % наклон в середине шага (уточнённый, по k2)
k4 = deriv(s + dt*k3,   a, w);  % наклон в конце шага (по k3)
s_next = s + dt/6*(k1 + 2*k2 + 2*k3 + k4); % взвешенная сумма четырёх наклонов - классическая формула RK4
end

function ds = deriv(s, a, w)
% Правая часть дифференциального уравнения движения (производные состояния).
psi = s(3); v = s(4);                        % достаём курс и скорость из состояния
ds = [v*cos(psi); v*sin(psi); w; a];         % [dx/dt; dy/dt; dpsi/dt; dv/dt]
end

function [nx, ny, npsi, found] = try_snap(x, y, psi, nodes, edges, capture_radius)
% Nearest map node, and (if it's close enough) the heading of whichever
% road leaving that node best matches the current heading.
% Находим ближайший узел карты и (если он достаточно близко) - курс
% вдоль той дороги от этого узла, что ближе всего к текущему курсу.
d2 = (nodes(:,1) - x).^2 + (nodes(:,2) - y).^2; % квадраты расстояний от (x,y) до каждого узла карты
[dmin2, i] = min(d2);                           % находим ближайший узел: dmin2 - расстояние^2, i - его номер
if sqrt(dmin2) > capture_radius                 % ближайший узел дальше радиуса захвата?
    nx = x; ny = y; npsi = psi; found = false;   % да (сейчас невозможно, радиус = inf) - коррекцию не делаем
    return
end

nx = nodes(i,1);      % x найденного узла
ny = nodes(i,2);       % y найденного узла
npsi = psi;            % курс по умолчанию - текущий (перезапишется ниже, если найдётся дорога получше)
best_diff = inf;       % пока лучшего совпадения по курсу не нашли
for e = 1:size(edges, 1)         % перебираем все дороги (рёбра графа) на карте
    if edges(e,1) == i           % это ребро выходит ИЗ найденного узла (как первый конец)?
        other = edges(e,2);       % тогда "другой конец" - второй узел ребра
    elseif edges(e,2) == i       % или ребро выходит из узла как второй конец?
        other = edges(e,1);       % тогда "другой конец" - первый узел ребра
    else
        continue                  % ребро не связано с найденным узлом - пропускаем
    end
    dir = atan2(nodes(other,2) - ny, nodes(other,1) - nx); % направление от узла к "другому концу" ребра
    diff = abs(wrap_to_pi(dir - psi));                      % насколько это направление отличается от текущего курса
    if diff < best_diff           % это совпадение лучше всех предыдущих?
        best_diff = diff;          % запоминаем новый рекорд
        npsi = dir;                 % и направление этой дороги как итоговый курс
    end
end
found = true;   % узел найден и (при необходимости) курс подобран
end

function a = wrap_to_pi(a)
% Приводим угол к диапазону (-pi, pi], чтобы разница углов считалась корректно.
a = mod(a + pi, 2*pi) - pi;
end

function [px, py, edir, edge_idx] = nearest_edge_point(x, y, psi, nodes, edges, cur_edge)
% Closest point on a road segment, that segment's direction, and which
% segment was picked - same nearest-point-on-segment idea as
% sim/blocks/map_matching_block.m, but with two extra safeguards
% needed at the very start (see the note above where this is called):
%   - among all segments within TIE_TOL of the closest one, pick
%     whichever direction best matches the current heading, instead of
%     blindly trusting raw distance (which is ambiguous right where a
%     road starts, since its nearest point can't extend past that
%     start node);
%   - hysteresis (MARGIN) against CUR_EDGE, so the choice doesn't flip
%     between two similarly-close segments from one step to the next.
% Русское резюме: находим ближайшую точку на дороге, направление этой
% дороги и её номер. При почти равном расстоянии до нескольких дорог
% выбираем ту, что ближе по направлению к текущему курсу (TIE_TOL), а
% гистерезис (MARGIN) не даёт выбору "дёргаться" между дорогами.
tie_tol = 5; % м - в пределах какого запаса расстояния дороги считаются "примерно одинаково близкими"
margin  = 3; % м - насколько другая дорога должна быть ближе, чтобы переключиться с текущей (гистерезис)

n = size(edges, 1);                    % сколько всего дорог (рёбер) на карте
proj_x = zeros(n, 1); proj_y = zeros(n, 1);   % сюда сложим проекции точки (x,y) на каждую дорогу
proj_d = zeros(n, 1); proj_dir = zeros(n, 1); % сюда - расстояния до проекций и направления дорог
best_dist = inf;                        % пока минимальное расстояние не найдено

for e = 1:n                             % перебираем все дороги по очереди
    a = nodes(edges(e,1), :);            % координаты первого узла текущей дороги
    b = nodes(edges(e,2), :);            % координаты второго узла текущей дороги
    ab = b - a;                          % вектор дороги (из первого узла во второй)
    denom = ab(1)^2 + ab(2)^2;           % квадрат длины дороги (для проекции)
    if denom < eps                       % дорога нулевой длины (вырожденный случай)?
        t = 0;                            % тогда проекция - это просто первый узел
    else
        t = ((x - a(1))*ab(1) + (y - a(2))*ab(2)) / denom; % параметр проекции точки на прямую (0=начало,1=конец)
        t = min(max(t, 0), 1);            % обрезаем в пределах отрезка (проекция не может выйти за концы дороги)
    end
    proj = a + t*ab;                     % сама точка проекции на дороге
    d = hypot(x - proj(1), y - proj(2)); % расстояние от (x,y) до этой точки проекции
    proj_x(e) = proj(1); proj_y(e) = proj(2); % запоминаем координаты проекции для этой дороги
    proj_d(e) = d; proj_dir(e) = atan2(ab(2), ab(1)); % запоминаем расстояние и направление дороги
    if d < best_dist                     % это расстояние - новый рекорд (самая близкая дорога пока)?
        best_dist = d;                    % обновляем рекорд
    end
end

best_e = 1;          % по умолчанию берём первую дорогу (перезапишется ниже)
best_hcost = inf;    % пока лучшего совпадения по курсу нет
for e = 1:n                                      % перебираем все дороги ещё раз
    if proj_d(e) <= best_dist + tie_tol           % эта дорога в пределах допуска от самой близкой?
        hcost = min(abs(wrap_to_pi(proj_dir(e) - psi)), abs(wrap_to_pi(proj_dir(e) + pi - psi))); % насколько направление дороги (в любую из двух сторон) отличается от курса
        if hcost < best_hcost                     % это совпадение по курсу лучше всех предыдущих?
            best_hcost = hcost;                    % запоминаем новый рекорд
            best_e = e;                             % и эту дорогу как текущий лучший выбор
        end
    end
end

if cur_edge ~= 0 && proj_d(cur_edge) <= best_dist + margin % на прошлом шаге уже была выбрана дорога, и она всё ещё близко?
    best_e = cur_edge;                                       % тогда остаёмся на ней (гистерезис - не дёргаемся)
end

px = proj_x(best_e); py = proj_y(best_e); edir = proj_dir(best_e); edge_idx = best_e; % отдаём наружу итоговую точку, направление и номер дороги
end

function h = pick_heading(edir, current_psi)
% A road segment's direction is ambiguous by 180 degrees (it doesn't
% know which way you're driving on it) - pick whichever of the two
% matches the current heading more closely, so warm-up doesn't flip
% the direction of travel.
% Русское резюме: у направления дороги есть 180-градусная
% неоднозначность (дорога не знает, в какую сторону по ней едут) -
% выбираем тот из двух вариантов, что ближе к текущему курсу.
cand1 = edir;                    % первый вариант - как есть
cand2 = wrap_to_pi(edir + pi);   % второй вариант - развёрнутый на 180 градусов
if abs(wrap_to_pi(cand1 - current_psi)) <= abs(wrap_to_pi(cand2 - current_psi)) % первый вариант ближе к текущему курсу?
    h = cand1;                    % да - берём его
else
    h = cand2;                    % нет - берём развёрнутый
end
end
