function xi=pointState_DRI_minus(X1,X2)
%X1 and X2 are the state with point feature, the form of them are
%(R,x,p1,...,pN), R is the rotation of the robot, x is the position of the robot, and pi is
%the position of the i-th pose. 
%the dimension of X1 and X2 should be the same.
%xi is in the lie algebra. Its form is (xi_R;xi_x;xi_p1;...;xi_pN)
%coded by Yang SONG

N1=size(X1,2);
N2=size(X2,2);

if N1==N2
    N=round(N1-3); %N is the num of features +1
else
    warning('The dimensions of the inputs do not match in poseState_minus!');
end

R1=X1(:,1:3);
R2=X2(:,1:3);
xi=zeros(3*N+3,1);
xi(1:3,1)= so3_log(R1*R2');
normV=norm(xi(1:3,1));
if normV<1.0e-30
    xi(1:3,1)=zeros(3,1);
else
    normvec=xi(1:3,1)/normV;
    if normV>=2*pi
        normV=mod(normV,2*pi);
        if normV>pi
            normV=normV-2*pi;
        end
    end
    xi(1:3,1)=normV*normvec;
end

xi(4:6)=-R1*R2'*X2(:,4)+X1(:,4);
for i = 2:N
%     xi(1+3*i:3+3*i)=-R1*R2'*X2(:,3+i)+X1(:,3+i);
    xi(1+3*i:3+3*i)=-X2(:,3+i)+X1(:,3+i);
end


