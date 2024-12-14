function X=posestate_Std_exp_conpoint(xi)
%xi is an element in lie algebra, whose form is a 6(N+1)*1 column (xi_R,xi_Rp,xi_x,xi_p)
%coded by Yang SONG

N=round((size(xi,1)-6)/2);
X=zeros(3,N+4);
X(:,1:3)=so3_exp( xi(1:3,1) );
X(:,4)=( xi(4:6,1) );

for i =1:N
%     X(:,3*i+1:3*i+3)=so3_exp( xi(3*i+1:3*i+3,1) );
    X(:,4+i)=[(xi(5+2*i:6+2*i,1));0];
end







