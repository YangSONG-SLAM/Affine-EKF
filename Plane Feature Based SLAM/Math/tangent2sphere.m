function y=tangent2sphere(x,a)
% the chart centered at x
% x,y \in S^2, a\in R^2
if norm(a)>1
    warning('The delta is out of range');
    a=a/norm(a);
end
y=(R_e3(x))*[a;sqrt(1-norm(a)^2)];

% y=so3_exp((R_e3(x))*[eye(2);zeros(1,2)]*a)*x;