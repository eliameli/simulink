function [r_e, v_e, w, dw_dt, T_ba, ang] = f_motion_model(t, pA, pT, pF)
%
    dt = 0.001;
    [r_e, v_e, w1, T_ba] = u_motion_model(t+dt, pA, pT, pF);
    [r_e, v_e, w2, T_ba] = u_motion_model(t-dt, pA, pT, pF);
    [r_e, v_e, w, T_ba]  = u_motion_model(t, pA, pT, pF);
    dw_dt = 0.5*(w1 - w2)/dt;
    theta = asin(T_ba(2,1));
    c = 1/sqrt(1-T_ba(2,1)^2);
%     psi = acos(T_ba(1,1)/cos(theta));
%     gamma = acos(T_ba(2,2)/cos(theta));
%     psi = atan(-T_ba(3,1)/T_ba(1,1));
%     gamma = atan(-T_ba(2,3)/T_ba(2,2));
    psi = atan2(-T_ba(3,1)*c,T_ba(1,1)*c);
    gamma = atan2(-T_ba(2,3)*c,T_ba(2,2)*c);
%     theta = atan2(T_ba(2,1),T_ba(2,2)*cos(gamma)-T_ba(2,3)*sin(gamma));
    ang = [theta; psi; gamma];
    
end

function [r, v, w, T_ba] = u_motion_model(t, pA, pT, pF)
%
    pW = 2*pi./pT;
    si = sin(pW*t + pF);
    co = cos(pW*t + pF);
    r = pA.*si;
    v = pW.*pA.*co;
    a = -pW.^2.*pA.*si;
    da = -pW.^3.*pA.*co;
    et = v/norm(v);
    at = (a'*et)*et;
    an = a - at;
%     if norm(an)/norm(a) < 1E-10
%         w = [0;0;0];
%     else
        en = an/norm(an);
        eb = cross(et, en);
        k1 = norm(an)/norm(v);
%         k1 = norm(cross(v,a))/norm(v)^3;
%         ek = cross(a, da);
%         k2 = v'*ek/norm(ek);
%         k2 = v'*cross(a, da)/norm(cross(v,a))^2;
        k2 = det([v,a,da])/norm(cross(v,a))^2;
        w = k2*et + k1*eb;
%     end
    T_ba = [et en eb];
end
