function xi=planeState_Std_minus(X1,X2)
%X1 and X2 are the state with plane feature, the form of them are
%([R,x;zeros(1,4)],[n1;d1],...,[nN;dN]), R is the rotation of the robot, x is the position of the robot, 
%coded by Yang SONG

N1=size(X1,2);
N2=size(X2,2);

if N1==N2
    N=round(N1)-4; %N is the num of features 
else
    warning('The dimensions of the inputs do not match in poseState_minus!');
end

R1=X1(1:3,1:3);
R2=X2(1:3,1:3);
xi=zeros(3*N+3,1);
xi(1:3,1)= so3_log(R1*R2');
normV=norm(xi(1:3,1));
if normV<1.0e-20
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

xi(4:6)=-X2(1:3,4)+X1(1:3,4);



for i = 1:N
%     X(:,3*i-2:3*i)=X1(:,3*i-2:3*i)*X2(:,3*i-2:3*i)';
    xi(4+3*i:5+3*i)=sphere2tangent(X2(1:3,4+i),X1(1:3,4+i));
    xi(6+3*i)=X1(4,4+i)-X2(4,4+i);
end






