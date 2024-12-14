function [X_new,P_new]=Augmentation_Std_conpoint2(X,P,z,omega)
% X=(R,Rp,x,p) is the states with the previous features
% P is the estimated covariance of X
% z=[z1_new_p1,..,z1_new_pk,z2_new_p1,...,z2_new_pk] is the observation of
% the k new features.
%z1_pi is a rotation matrix,
%z2_pi is a position vector.
%omega is the observaation noise for each (z1_pi,z2_pi). It should be a 6*6
%matrix.
% coded by Yang SONG
% N1=size(X,2);
% N2=size(P,2);
% N1=round(N1)-4;
% N2=round((N2-6)/2);
% if N1==N2
%     N=N1; %N is the num of previous features
% else
%     warning('The dimensions of the inputs do not match in function "Augmentation"!');
% end
N=size(X,2)-4;



%N_new=round(size(z,2)/4);%the num of new features
N_new=round(size(z.position,2));
R=X(1:3,1:3);
%new features
% f1=zeros(3,3*N_new); %f1 is the rotation part for the new features
f2=zeros(3,N_new); %f2 is the rotation part for the new features

if N==0
    c=sum(z.position(3,:))/N_new;
    P=[P zeros(6,1);zeros(1,6) omega(3,3)/N_new];
else
    c=X(3,5);
end
    
for i=1:N_new
%     f1(1:3,3*i-2:3*i)=R*z.rotation(1:3,3*i-2:3*i);
    f2(1:3,i)=X(1:3,4)+R*z.position(1:3,i);
    f2(3,i)=c;
%     f1(1:3,3*i-2:3*i)=z(1:3,3*i-2:3*i)*R;
%     f2(1:3,i)=X(1:3,3*N+4)+R*z(1:3,3*N_new+i);
end

%new state
X_new=[X f2];

%new covariance
P=(P+P')/2;
P11=P(1:3,1:3);
P12=P(1:3,4:2*N+7);
P21=P12';
P22=P(4:2*N+7,4:2*N+7);


M2=zeros(2*N_new,3);
M3=zeros(2*N_new,2*N+4);
Omega22=zeros(3*N_new,3*N_new);
R_blk=zeros(2*N_new,3*N_new);
for i=1:N_new

    M2(2*i-1:2*i,1:3)=-[eye(2) zeros(2,1)]*skew(R*z.position(1:3,i));
    M3(2*i-1:2*i,1:3)=[eye(2) zeros(2,1)];

    Omega22(3*i-2:3*i,3*i-2:3*i)=omega;
    R_blk(2*i-1:2*i,3*i-2:3*i)=[eye(2) zeros(2,1)]*R;
end
% Omega21=Omega12';

% P_new=[P11 P12 P11*M2'+P12*M3';...
%     P21 P22 P21*M2'+P22*M3';...
%     M2*P11+M3*P21 M2*P12+M3*P22 M2*P11*M2'+M2*P12*M3'+M3*P21*M2'+M3*P22*M3'+R_blk*Omega22*R_blk'];
P_new=[P11 P12 P11*M2'+P12*M3';...
    P21 P22 P21*M2'+P22*M3';...
    M2*P11+M3*P21 M2*P12+M3*P22 M2*P11*M2'+M2*P12*M3'+M3*P21*M2'+M3*P22*M3'+R_blk*Omega22*R_blk'];
P_new=(P_new+P_new')/2;



