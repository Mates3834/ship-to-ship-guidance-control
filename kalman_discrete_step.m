function [xhat,P] = kalman_discrete_step(xhat,P,u,y,A,B,C,Q,R)
%KALMAN_DISCRETE_STEP One predict/update cycle of a linear discrete Kalman filter.
%
% Model:
%   x(k+1) = A x(k) + B u(k) + w(k)
%   y(k)   = C x(k) + v(k)

% Prediction
xPred = A*xhat + B*u;
PPred = A*P*A' + Q;

% Innovation
innovation = y - C*xPred;
S = C*PPred*C' + R;

% Kalman gain
K = PPred*C'/S;

% Update
xhat = xPred + K*innovation;
P = (eye(size(P)) - K*C)*PPred;
end
