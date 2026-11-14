function [ ang_out ] = f_normalize_angle( ang_in )
%f_normalize_angle
%
    if ang_in > 2*pi
        ang_in = ang_in - 2*pi*floor(ang_in/2/pi);
    elseif ang_in < -pi
        ang_in = ang_in + 2*pi*ceil(-ang_in/2/pi);
    end
    if ang_in > pi
        ang_out = ang_in - 2*pi;
    else
        ang_out = ang_in;
    end
end

