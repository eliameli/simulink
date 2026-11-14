function w = f_calc_ang_vel(u, du_dt)
%
    w = [0, sin(u(1)), 1;...
         sin(u(3)), cos(u(1))*cos(u(3)), 0;...
         cos(u(3)), -cos(u(1))*sin(u(3)), 0]*du_dt;

end