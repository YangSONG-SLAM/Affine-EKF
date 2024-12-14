function [X_new,P_new]=Augmentation_Affv3_point(X,P,z,omega)
% X=(R,Rp,x,p) is the states with the previous features
% P is the estimated covariance of X
% z=[z1_new_p1,..,z1_new_pk,z2_new_p1,...,z2_new_pk] is the observation of
% the k new features.
%z1_pi is a rotation matrix,
%z2_pi is a position vector.
%omega is the observaation noise for each (z1_pi,z2_pi). It should be a 6*6
%matrix.
% coded by Yang SONG
N1=size(X,2);
N2=size(P,2);
N1=round(N1)-4;
N2=round(N2/3-2);
if N1==N2
    N=N1; %N is the num of previous features
else
    warning('The dimensions of the inputs do not match in function "Augmentation"!');
end

%N_new=round(size(z,2)/4);%the num of new features
N_new=round(size(z.position,2));
R=X(1:3,1:3);
%new features
% f1=zeros(3,3*N_new); %f1 is the rotation part for the new features
f2=zeros(3,N_new); %f2 is the rotation part for the new features
for i=1:N_new
%     f1(1:3,3*i-2:3*i)=R*z.rotation(1:3,3*i-2:3*i);
    f2(1:3,i)=X(1:3,4)+R*z.position(1:3,i);
%     f1(1:3,3*i-2:3*i)=z(1:3,3*i-2:3*i)*R;
%     f2(1:3,i)=X(1:3,3*N+4)+R*z(1:3,3*N_new+i);
end

%new state
X_new=[X f2];

%new covariance
P=(P+P')/2;
P11=P(1:3,1:3);
P12=P(1:3,4:3*N+6);
P21=P12';
P22=P(4:3*N+6,4:3*N+6);

% M1=zeros(3*N_new,3*N+3);
M2=zeros(3*N_new,3*N+3);
% Omega11=zeros(3*N_new,3*N_new);
% Omega12=zeros(3*N_new,3*N_new);
Omega22=zeros(3*N_new,3*N_new);
R_blk=zeros(3*N_new,3*N_new);
for i=1:N_new
%     M1(3*i-2:3*i,1:3)=eye(3);
    M2(3*i-2:3*i,1:3)=R';
%     Omega11(3*i-2:3*i,3*i-2:3*i)=omega(1:3,1:3);
%     Omega12(3*i-2:3*i,3*i-2:3*i)=omega(1:3,4:6);
    Omega22(3*i-2:3*i,3*i-2:3*i)=omega;
    R_blk(3*i-2:3*i,3*i-2:3*i)=R;
end
% Omega21=Omega12';

P_new=[P11  P12 P12*M2';...
    P21  P22 P22*M2';...
    M2*P21 M2*P22 M2*P22*M2'+R_blk*Omega22*R_blk'];




