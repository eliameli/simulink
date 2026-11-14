function dv_dt = f_turning_by_Euler2(v, w)
%
%     dv_dt = zeros(3,1);
    dv_dt = [1 sin(v(1))*tan(v(2)) cos(v(1))*tan(v(2));...
             0 cos(v(1)) -sin(v(1));...
             0 sin(v(1))/cos(v(2)) cos(v(1))/cos(v(2))]*w;

end