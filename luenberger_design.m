function L = luenberger_design(A,C,observerPoles)
%LUENBERGER_DESIGN Observer gain by pole placement.
%
% Observer:
%   xhat_dot = A*xhat + B*u + L*(y - C*xhat)

L = place(A',C',observerPoles)';
end
