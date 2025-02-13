function X=poseState_plus_point(X1,X2)
%X1 and X2 are the state with point feature, the form of them are
%(R,x,p1,...,pN), R is the rotation of the robot, x is the position of the robot, and pi is
%the position of the i-th pose. 
%the dimension of X1 and X2 should be the same.
%coded by Yang SONG
N1=size(X1,2);
N2=size(X2,2);

if N1==N2
    N=round(N1)-3;
else
    warning('The dimensions of the inputs do not match!');
end

R1=X1(:,1:3);
R2=X2(:,1:3);
X=zeros(3,N+3);
X(:,1:3)=R1*R2;

% for i = 1:N-1
%     X(:,3*i+1:3*i+3)=X1(:,3*i+1:3*i+3)*X2(:,3*i+1:3*i+3);
% end

for i= 1:N
    X(:,3+i)=R1*X2(:,3+i)+X1(:,3+i); 
end
