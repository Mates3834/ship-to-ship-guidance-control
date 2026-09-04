function K = lqr_design(A,B,Q,R)
%LQR_DESIGN Continuous-time LQR gain.
%
% Requires Control System Toolbox.

validateattributes(Q,{'numeric'},{'2d','square'});
validateattributes(R,{'numeric'},{'2d','square','positive'});

K = lqr(A,B,Q,R);
end
