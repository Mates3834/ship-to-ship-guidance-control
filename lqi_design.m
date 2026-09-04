function Kaug = lqi_design(A,B,Ctrack,Qaug,R)
%LQI_DESIGN Generic continuous-time LQI gain via state augmentation.
%
% Plant:
%   xdot = A x + B u
% Tracked output:
%   y = Ctrack x
%
% Integral state:
%   xidot = r - y

n = size(A,1);
p = size(Ctrack,1);

Aaug = [A, zeros(n,p);
       -Ctrack, zeros(p,p)];
Baug = [B;
        zeros(p,size(B,2))];

Kaug = lqr(Aaug,Baug,Qaug,R);
end
