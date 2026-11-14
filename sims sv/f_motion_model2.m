function [r_e, v_e, a_e, T_ba, ang] = f_motion_model2(t, pA, pT, pF)
%
    pW = 2*pi./pT;
    si = sin(pW*t + pF);
    co = cos(pW*t + pF);
    r_e = pA.*si;
    v_e = pW.*pA.*co;
    a_e = -pW.^2.*pA.*si;
    if norm(v_e) < 1E-30
        et = [1;0;0];
    else
        et = v_e/norm(v_e);
    end
    at = (a_e'*et)*et;
    an = a_e - at;
    if norm(an) < 1E-30
        en = [-et(2), et(1), 0];
    else
        en = an/norm(an);
    end
    eb = cross(et, en);
    T_ba = [et en eb];
    theta = asin(T_ba(2,1));
    c = 1/sqrt(1-T_ba(2,1)^2);
    psi = atan2(-T_ba(3,1)*c,T_ba(1,1)*c);
    gamma = atan2(-T_ba(2,3)*c,T_ba(2,2)*c);
%     theta = atan2(T_ba(2,1),T_ba(2,2)*cos(gamma)-T_ba(2,3)*sin(gamma));
    ang = [theta; psi; gamma];

end
