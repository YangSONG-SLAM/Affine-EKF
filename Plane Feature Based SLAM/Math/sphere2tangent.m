function a=sphere2tangent(x,y)
% the chart centered at x
% x,y \in S^2, a\in R^2
a=[eye(2) zeros(2,1)]*(R_e3(x))'*y;
