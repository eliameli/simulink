function dv_dt = f_turning_by_Euler(v, w)
%
    dv_dt = zeros(3,1);
    dv_dt(1) = w(2)*sin(v(3)) + w(3)*cos(v(3));
    dv_dt(2) = (w(2)*cos(v(3)) - w(3)*sin(v(3)))/cos(v(1));
    dv_dt(3) = w(1) - tan(v(1))*(w(2)*cos(v(3)) - w(3)*sin(v(3)));    

end