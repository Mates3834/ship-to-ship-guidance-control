function [psiRef, reached] = los_waypoint_guidance(pos, waypoint, acceptanceRadius)
%LOS_WAYPOINT_GUIDANCE Generic planar line-of-sight waypoint guidance.
%
% pos      : [x y]
% waypoint : [x y]
%
% psiRef is the desired planar heading angle [rad].

delta = waypoint - pos;
distance = norm(delta);

psiRef = atan2(delta(2),delta(1));
reached = distance <= acceptanceRadius;
end
