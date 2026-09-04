function out = mrac_first_order(ref,t,params)
%MRAC_FIRST_ORDER Educational first-order MRAC example.
%
% Generic plant:
%   ydot = a*y + b*u
%
% Reference model:
%   ymdot = am*ym + bm*r
%
% Adaptive control:
%   u = theta_r*r - theta_y*y
%
% Sigma-modification is included as a robustness term.

a = params.a;
b = params.b;
am = params.am;
bm = params.bm;
gamma = params.gamma;
sigma = params.sigma;

dt = mean(diff(t));

y = 0;
ym = 0;
theta_r = 0;
theta_y = 0;

Y = zeros(size(t));
YM = zeros(size(t));
U = zeros(size(t));
TR = zeros(size(t));
TY = zeros(size(t));

for k = 1:numel(t)
    r = ref(k);
    e = y - ym;

    u = theta_r*r - theta_y*y;

    dtheta_r = -gamma*e*r - sigma*theta_r;
    dtheta_y =  gamma*e*y - sigma*theta_y;

    dy  = a*y + b*u;
    dym = am*ym + bm*r;

    y = y + dt*dy;
    ym = ym + dt*dym;
    theta_r = theta_r + dt*dtheta_r;
    theta_y = theta_y + dt*dtheta_y;

    Y(k) = y;
    YM(k) = ym;
    U(k) = u;
    TR(k) = theta_r;
    TY(k) = theta_y;
end

out.y = Y;
out.ym = YM;
out.u = U;
out.theta_r = TR;
out.theta_y = TY;
end
