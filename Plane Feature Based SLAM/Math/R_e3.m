function R=R_e3(n)
J=[0 -1;1 0];
norm_n1n2=norm(n(1:2));
if norm_n1n2>10^(-4)
    R=so3_exp([J*n(1:2)/norm_n1n2;0]*acos(n(3)));
else
    R=eye(3);
end