function delta = f_angles_diff(u1, u2)
%
    delta = u1 - u2;
    for i=1:length(delta)
        if delta(i) > pi
            delta(i) = 2*pi-delta(i);
        end
    end

end