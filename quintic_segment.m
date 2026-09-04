function [p,v,a] = quintic_segment(p0,pf,T,t)
%QUINTIC_SEGMENT Smooth quintic interpolation with zero end velocity/acceleration.
%
% p0,pf : scalar or same-size vectors
% T     : segment duration
% t     : evaluation time(s), 0 <= t <= T
%
% This is a simple generic smooth-trajectory primitive.

tau = t./T;
s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
sd  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)./T;
sdd = (60*tau - 180*tau.^2 + 120*tau.^3)./(T^2);

dp = pf - p0;
p = p0 + dp.*s;
v = dp.*sd;
a = dp.*sdd;
end
