function X_estimation=pointfeature_FEJEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise)

% coded by Yang SONG
tic

T_steps=size(Index,2);
[Xn,Pn]=Augmentation_Std_point(X0,P0,z0,OBSV_noise);

X_estimation=cell(1,T_steps);
Xstate4Jacobian=Xn;
for i =1:T_steps
    %[Xn,Pn]=removeUnobser(Xn,Pn,Index{i}.RemainIndex);
    N=round(size(Xn,2)-4); %the num of features  in the (i-1)-th state
    
    %Prediction
    %The form of error xi is a 6(N+1)*1 column (xi_R,xi_Rp,xi_x,xi_p)
    X_Prediction=Xn;
    X_Prediction(1:3,1:3)=Xn(1:3,1:3)*U_noise{i}.rotation;
    X_Prediction(1:3,4)=Xn(1:3,1:3)*U_noise{i}.position+Xn(1:3,4);
    
    %Fn
    F=eye(3*N+6);
    F(4:6,1:3)=-skew(X_Prediction(1:3,4)-Xstate4Jacobian(1:3,4));
    
    
    %Odometry noise is the first-order integration
    A=zeros(6,6);
    if i==1
        A(1:3,1:3)=eye(3);
        A(4:6,4:6)=eye(3);
    else
        A(1:3,1:3)=Xstate4Jacobian(1:3,1:3);
        A(4:6,4:6)=Xstate4Jacobian(1:3,1:3);
    end
    
    P_adX_W=blkdiag(A*ODOM_noise*A',zeros(3*N,3*N));
    %Prediction of cov of noise, P_{n+1|n}
    Pn_Prediction=F*Pn*F'+P_adX_W;
    Pn_Prediction=(Pn_Prediction+Pn_Prediction')/2;
    
    
    %H_{n+1}
    N_ob=size(Index{i}.RemainIndex,2);
    H=zeros(3*N_ob,3*N+6);
    %The form of error xi is a 6(N+1)*1 column (xi_R,xi_Rp,xi_x,xi_p)
    for j=1:N_ob
        

        H(3*j-2:3*j,1:3)=X_Prediction(1:3,1:3)'*skew(Xstate4Jacobian(1:3,4+Index{i}.RemainIndex(j))-X_Prediction(1:3,4));
        H(3*j-2:3*j,4:6)=-X_Prediction(1:3,1:3)';
        H(3*j-2:3*j,4+3*Index{i}.RemainIndex(j):6+3*Index{i}.RemainIndex(j))=X_Prediction(1:3,1:3)';

     end
    
    %Kalman Gain K_{n+1}
    Omega=zeros(3*N_ob,3*N_ob);
    for j=1:N_ob
        Omega(3*j-2:3*j,3*j-2:3*j)=OBSV_noise(1:3,1:3);
    end
    S=H*Pn_Prediction*H'+ Omega;
    K=Pn_Prediction*H'/S;
    
    %Y_{n+1}=[y1_p1;y1_p2;...;y2_pN]
    Y=zeros(3*N_ob,1);
    for j=1:N_ob
            Y(3*j-2:3*j,1)=z_noise{i}.position(1:3,j)-X_Prediction(1:3,1:3)'*(X_Prediction(1:3,4+Index{i}.RemainIndex(j))-X_Prediction(1:3,4));

    end
    
    Xn=poseState_Std_plus_point(posestate_Std_exp_point(K*Y),X_Prediction);
    Pn=(eye(3*N+6)-K*H)*Pn_Prediction;
    
    Xstate4Jacobian(1:3,1:4)=X_Prediction(1:3,1:4);

    if size(z_noise{i}.position(:,N_ob+1:end),2)>0.5
        z_new.position=z_noise{i}.position(1:3,N_ob+1:end);
        [Xn,Pn]=Augmentation_Std_point(Xn,Pn,z_new,OBSV_noise);
        Pn=(Pn+Pn')/2;
        
        n_old=size(Xstate4Jacobian,2);
        Xstate4Jacobian=[Xstate4Jacobian Xn(1:3,n_old+1:end)];
    end
    

    X_estimation{i}.state=Xn;
    X_estimation{i}.cov=Pn;
    
    
end
X_estimation{1}.time=toc;    













%%
% T_steps=size(Index,2);
% [Xn,Pn]=Augmentation_Std_point(X0,P0,z0,OBSV_noise);
% % Pn=0*Pn;
% % [Xn,Pn]=Augmentation(X0,P0,z0,OBSV_noise);
% % X_estimation.X0=Xn;
% % X_estimation.P0=Pn;
% 
% 
% % Xn=X0;
% % Pn=P0;
% X_estimation=cell(1,T_steps);
% 
% for i =1:T_steps
%     %[Xn,Pn]=removeUnobser(Xn,Pn,Index{i}.RemainIndex);
%     N=round(size(Xn,2)-4); %the num of features  in the (i-1)-th state
%     
%     %Prediction
%     %The form of error xi is a 6(N+1)*1 column (xi_R,xi_Rp,xi_x,xi_p)
%     X_Prediction=Xn;
%     X_Prediction(1:3,1:3)=Xn(1:3,1:3)*U_noise{i}.rotation;
%     X_Prediction(1:3,4)=Xn(1:3,1:3)*U_noise{i}.position+Xn(1:3,4);
% 
%     
%     %Fn
%     F=eye(3*N+6);
%     if i==1
%         F(4:6,1:3)=-skew(Xstate_gt{i}(1:3,4));
%     else
%         F(4:6,1:3)=-skew(Xstate_gt{i}(1:3,4)-Xstate_gt{i-1}(1:3,4));
%     end
% 
%     
%     %Gn*Qn*Gn'
% %     Gn=zeros(6*N+6,6*N+6);
% %     Gn(1:3,1:3)=Xn(1:3,1:3);
% %     Gn(3*N+4:3*N+6,1:3)=skew( Xn(1:3,3*N+4) )*Xn(1:3,1:3);
% %     Gn(3*N+4:3*N+6,3*N+4:3*N+6)=Xn(1:3,1:3);
% %     for j=1:N
% %         Gn(3*j+1:3*j+3,3*j+1:3*j+3)=Xn(1:3,3*j+1:3*j+3);
% %         Gn(3*N+4+3*j:3*N+6+3*j,1:3)=skew( Xn(1:3,3*N+4+j) )*Xn(1:3,1:3);
% %         Gn(3*N+4+3*j:3*N+6+3*j,3*N+4+3*j:3*N+6+3*j)=Xn(1:3,1:3);
% %     end
% %     bigODOM_noise=zeros(6*N+6,6*N+6);
% %     bigODOM_noise(1:3,1:3)=ODOM_noise(1:3,1:3);
% %     bigODOM_noise(1:3,3*N+4:3*N+6)=ODOM_noise(1:3,4:6);
% %     bigODOM_noise(3*N+4:3*N+6,1:3)=ODOM_noise(4:6,1:3);
% %     bigODOM_noise(3*N+4:3*N+6,3*N+4:3*N+6)=ODOM_noise(4:6,4:6);
% %     
% %     P_adX_W=Gn*bigODOM_noise*Gn';
%     
%     %Odometry noise is the first-order integration
%     A=zeros(6,6);
%     if i==1
%         A(1:3,1:3)=eye(3);
%         A(4:6,4:6)=eye(3);
%     else
%         A(1:3,1:3)=Xstate_gt{i-1}(1:3,1:3);
%         A(4:6,4:6)=Xstate_gt{i-1}(1:3,1:3);
%     end
%     C=zeros(3*N,6);
%     
%     P_adX_W=[A*ODOM_noise*A' A*ODOM_noise*C';C*ODOM_noise'*A' C*ODOM_noise*C'];
%     
%     %Prediction of cov of noise, P_{n+1|n}
%     Pn_Prediction=F*Pn*F'+P_adX_W;
%     Pn_Prediction=(Pn_Prediction+Pn_Prediction')/2;
%     
%     
%     %H_{n+1}
%     N_ob=size(Index{i}.RemainIndex,2);
%     %N_ob_pre=size(Index{i}.RemainIndex,2);
%     H=zeros(3*N_ob,3*N+6);
%     %The form of error xi is a 6(N+1)*1 column (xi_R,xi_Rp,xi_x,xi_p)
%     for j=1:N_ob
% %         H(3*j-2:3*j,1:3)=-X_Prediction(1:3,1:3)';
% %         H(3*j-2:3*j,3*Index{i}.RemainIndex(j)+1:3*Index{i}.RemainIndex(j)+3)=X_Prediction(1:3,1:3)';
%         
% 
%         H(3*j-2:3*j,1:3)=Xstate_gt{i}(1:3,1:3)'*skew(Xstate_gt{i}(1:3,4+Index{i}.RemainIndex(j))-Xstate_gt{i}(1:3,4));
%         H(3*j-2:3*j,4:6)=-Xstate_gt{i}(1:3,1:3)';
%         H(3*j-2:3*j,4+3*Index{i}.RemainIndex(j):6+3*Index{i}.RemainIndex(j))=Xstate_gt{i}(1:3,1:3)';
% %         for k=Index{i}.RemainIndex
% % %             H(3*j-2:3*j,1:3)=-X_Prediction(1:3,3*k+1:3*k+3)*X_Prediction(1:3,1:3)';%Right
% % %             %H(3*j-2:3*j,1:3)=-X_Prediction(1:3,3*k+1:3*k+3);%Left
% % %             
% % %             H(3*j-2:3*j,3*k+1:3*k+3)=eye(3);%Right
% % %             %H(3*j-2:3*j,3*k+1:3*k+3)=X_Prediction(1:3,3*k+1:3*k+3);%Left
% %             
% %             H(3*j-2:3*j,1:3)=-X_Prediction(1:3,1:3)';
% %             H(3*j-2:3*j,3*k+1:3*k+3)=X_Prediction(1:3,1:3)';
% % 
% %             H(3*N_ob+3*j-2:3*N_ob+3*j,3*N+4:3*N+6)=-X_Prediction(1:3,1:3)';
% %             H(3*N_ob+3*j-2:3*N_ob+3*j,3*N+4+3*k:3*N+6+3*k)=X_Prediction(1:3,1:3)';
% % %             H(6*j-5:6*j-3,1:3)=-X_Prediction(1:3,3*k+1:3*k+3)*X_Prediction(1:3,1:3)';
% % %             H(6*j-5:6*j-3,3*j+1:3*j+3)=eye(3);
% % %             H(6*j-2:6*j,3*N+4:3*N+6)=-X_Prediction(1:3,1:3)';
% % %             H(6*j-2:6*j,3*N+4+3*j:3*N+6+3*j)=X_Prediction(1:3,1:3)';
% %         end
%      end
%     
%     %Kalman Gain K_{n+1}
%     Omega=zeros(3*N_ob,3*N_ob);
%     for j=1:N_ob
%         Omega(3*j-2:3*j,3*j-2:3*j)=OBSV_noise(1:3,1:3);
% %         Omega(3*N_ob+3*j-2:3*N_ob+3*j,3*N_ob+3*j-2:3*N_ob+3*j)=OBSV_noise(4:6,4:6);
%         %Omega(6*j-5:6*j,6*j-5:6*j)=OBSV_noise;%this is wrong
%     end
%     S=H*Pn_Prediction*H'+ Omega;
%     K=Pn_Prediction*H'/S;
%     
%     %Y_{n+1}=[y1_p1;y1_p2;...;y2_pN]
%     Y=zeros(3*N_ob,1);
%     for j=1:N_ob
%         
%         Y(3*j-2:3*j,1)=z_noise{i}.position(1:3,j)-X_Prediction(1:3,1:3)'*(X_Prediction(1:3,4+Index{i}.RemainIndex(j))-X_Prediction(1:3,4));
% %         Y(6*j-5:6*j-3,1)=so3_log(z_noise{i}.rotation(1:3,3*j-2:3*j)*X_Prediction(1:3,1:3)*X_Prediction(1:3,3*j+1:3*j+3)');
% %         Y(6*j-2:6*j,1)=z_noise{i}.position(1:3,j)-X_Prediction(1:3,1:3)'*(X_Prediction(1:3,3*N+4+j)-X_Prediction(1:3,3*N+4));
%     end
%     
%     Xn=poseState_Std_plus_point(posestate_Std_exp_point(K*Y),X_Prediction);
%     Pn=(eye(3*N+6)-K*H)*Pn_Prediction;
% 
%     if size(z_noise{i}.position(:,N_ob+1:end),2)>0.5
% %         z_new.rotation=z_noise{i}.rotation(1:3,3*N_ob+1:end);
%         z_new.position=z_noise{i}.position(1:3,N_ob+1:end);
% %         z_new=[z_noise{i}.rotation(1:3,3*N+1:end) z_noise{i}.position(1:3,N+1:end)];
%         [Xn,Pn]=Augmentation_Std_point(Xn,Pn,z_new,OBSV_noise);
%         Pn=(Pn+Pn')/2;
%     end
%     
% %     X_estimation{i}.H=H;
% %     X_estimation{i}.S=S;
% %     X_estimation{i}.K=K;
% %     X_estimation{i}.Y=Y;
%     X_estimation{i}.state=Xn;
%     X_estimation{i}.cov=Pn;
%     
%     
%     
% end
    