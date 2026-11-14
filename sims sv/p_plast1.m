function p_plast1

DT = 2*pi;
t = 0:DT/1000:DT;
p = [1 1 1 DT DT/3 DT/5 0 90 30];
[r v a] = f_motion_params(t, p);

n = length(a);
at = zeros(3,n);
an = zeros(3,n);
et = zeros(3,n);
en = zeros(3,n);
eb = zeros(3,n);
for i=1:n
    et(:,i) = v(:,i)/norm(v(:,i));
    at(:,i) = a(:,i)'*et(:,i)*et(:,i);
    an(:,i) = a(:,i) - at(:,i);
    en(:,i) = an(:,i)/norm(an(:,i));
    eb(:,i) = cross(et(:,i), en(:,i));
%     norm(an(:,i))
end

subplot(4,1,1:2);
plot3(r(1,:), r(2,:), r(3,:));
grid on;
subplot(4,1,3);
% plot3(v(1,:), v(2,:), v(3,:));
%plot(t, hypot(v(1,:),hypot(v(2,:),v(3,:))));
plot(t, hypot(an(1,:),hypot(an(2,:),an(3,:))));
grid on;
subplot(4,1,4);
% plot3(a(1,:), a(2,:), a(3,:));
%plot(t, hypot(a(1,:),hypot(a(2,:),a(3,:))));
plot(t, hypot(at(1,:),hypot(at(2,:),at(3,:))));
grid on;



end

function [r v a] = f_motion_params(t, p)
%
    
    A  = [p(1); p(2); p(3)];
    om = 2*pi./[p(4); p(5); p(6)];
    fi = pi/180*[p(7); p(8); p(9)];

    si = sin(om*t + repmat(fi,1,length(t)));
    co = cos(om*t + repmat(fi,1,length(t)));
    o1 = repmat(om, 1, length(t));
    o2 = repmat(om.^2, 1, length(t));
    r = repmat(A,1,length(t)).*si;
    v = repmat(A,1,length(t)).*o1.*co;
    a = -repmat(A,1,length(t)).*o2.*si;

end