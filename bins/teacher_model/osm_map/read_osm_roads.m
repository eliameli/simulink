function R = read_osm_roads(fname, lat0, lon0)
%READ_OSM_ROADS  Читает файл OpenStreetMap (.osm, XML) и достаёт из него дороги.
%   R = read_osm_roads('map.osm', lat0, lon0)
%
%   Как устроен файл .osm (это обычный XML):
%     <node id="123" lat="55.76" lon="37.69" .../>      — точка с координатами
%     <way id="456"> <nd ref="123"/> <nd ref="124"/> ...  — линия из точек (улица, здание, забор...)
%                    <tag k="highway" v="residential"/>   — тег: что это за линия
%     </way>
%     <relation> ...                                    — группы объектов (нам не нужны)
%
%   Дорога = way с тегом highway=<тип>. Всё остальное (здания, деревья, остановки) отбрасываем.
%
%   Выход R (структура):
%     R.node_id   — id узлов OSM, которые лежат на дорогах (N x 1)
%     R.lat, R.lon — их координаты, градусы
%     R.x, R.y     — те же точки в локальных метрах (x — восток, y — север) от точки (lat0, lon0)
%     R.s, R.t     — номера узлов (в массивах выше) начала и конца каждого отрезка дороги
%     R.len        — длина отрезка, м
%     R.type       — тип дороги отрезка (cell): 'primary', 'residential', ...
%     R.oneway     — true, если улица односторонняя (движение только s -> t)
%
%   Работает и в MATLAB, и в Octave: только regexp, без тулбоксов.

    % --- какие типы highway считаем проезжими для машины ---
    car_types = {'motorway','trunk','primary','secondary','tertiary', ...
                 'unclassified','residential','living_street','service', ...
                 'motorway_link','trunk_link','primary_link','secondary_link','tertiary_link'};

    txt = fileread(fname);

    % ---------- 1. Узлы: id, lat, lon ----------
    % Берём каждый открывающий тег <node ...> целиком, потом из него вытаскиваем атрибуты
    % (порядок атрибутов в файле может быть любым, поэтому ищем каждый отдельно).
    node_tags = regexp(txt, '<node\s[^>]*>', 'match');
    nN = numel(node_tags);
    id  = zeros(nN,1); lat = zeros(nN,1); lon = zeros(nN,1);
    for k = 1:nN
        s = node_tags{k};
        id(k)  = str2double(regexp(s, '\sid="([^"]+)"',  'tokens', 'once'));
        lat(k) = str2double(regexp(s, '\slat="([^"]+)"', 'tokens', 'once'));
        lon(k) = str2double(regexp(s, '\slon="([^"]+)"', 'tokens', 'once'));
    end
    % id OSM до ~1e10 — в double хранятся точно (точность double до 2^53 ≈ 9e15)

    % ---------- 2. Линии (way) с тегом highway ----------
    ways = regexp(txt, '<way\s.*?</way>', 'match');   % каждый way целиком, от <way до </way>
    S = []; T = []; TYPE = {}; ONEWAY = [];
    for k = 1:numel(ways)
        w = ways{k};
        hw = regexp(w, '<tag\s+k="highway"\s+v="([^"]+)"', 'tokens', 'once');
        if isempty(hw), continue; end                 % не дорога
        hw = hw{1};
        if ~any(strcmp(hw, car_types)), continue; end % пешеходная, велодорожка и т.п.

        refs_c = regexp(w, '<nd\s+ref="([^"]+)"', 'tokens');   % список точек линии
        refs = cellfun(@(c) str2double(c{1}), refs_c);
        if numel(refs) < 2, continue; end

        ow = regexp(w, '<tag\s+k="oneway"\s+v="([^"]+)"', 'tokens', 'once');
        is_oneway = ~isempty(ow) && any(strcmp(ow{1}, {'yes','1','true'}));
        is_rev    = ~isempty(ow) && strcmp(ow{1}, '-1');   % односторонняя «против» порядка точек
        if is_rev, refs = fliplr(refs); is_oneway = true; end

        % линия из n точек -> n-1 отрезков (соседние точки)
        S = [S; refs(1:end-1)'];          %#ok<AGROW>
        T = [T; refs(2:end)'];            %#ok<AGROW>
        TYPE   = [TYPE;   repmat({hw}, numel(refs)-1, 1)];        %#ok<AGROW>
        ONEWAY = [ONEWAY; repmat(is_oneway, numel(refs)-1, 1)];   %#ok<AGROW>
    end
    if isempty(S)
        error('В файле не найдено ни одной автомобильной дороги (way с тегом highway).');
    end

    % ---------- 3. Оставляем только узлы, которые лежат на дорогах ----------
    used = unique([S; T]);
    [ok, pos] = ismember(used, id);
    if any(~ok)
        warning('%d узлов дорог нет в файле (дорога выходит за рамку выгрузки) — эти отрезки выброшены.', sum(~ok));
    end
    used = used(ok); pos = pos(ok);
    R.node_id = used;
    R.lat = lat(pos);
    R.lon = lon(pos);

    % номера концов отрезков в новых массивах
    [okS, R.s] = ismember(S, used);
    [okT, R.t] = ismember(T, used);
    keep = okS & okT;
    R.s = R.s(keep); R.t = R.t(keep);
    R.type = TYPE(keep); R.oneway = ONEWAY(keep);

    % ---------- 4. Локальные метры (плоская касательная плоскость) ----------
    % x = R·cos(lat0)·Δlon, y = R·Δlat, углы в радианах. Для района в 2–3 км ошибка < 0,5 м.
    Rz = 6378137;
    R.x = deg2rad(R.lon - lon0) * Rz * cosd(lat0);
    R.y = deg2rad(R.lat - lat0) * Rz;
    R.len = hypot(R.x(R.s) - R.x(R.t), R.y(R.s) - R.y(R.t));

    fprintf('OSM: %d узлов всего, %d на дорогах; %d отрезков дорог, суммарно %.1f км\n', ...
            nN, numel(R.node_id), numel(R.s), sum(R.len)/1000);
end
