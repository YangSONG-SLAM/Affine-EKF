%increment of NEES for pose feature based Std-SLAM
%the result is (some difference of X_Estimation and X_state_gt)'/Pn*(some difference of X_Estimation and X_state_gt)/(m*d), 
%m is the number of tests
%coded by Yang SONG

function NEES_add=conpointfeature2_NEES_Std_add(X_Estimation,Xstate_gt,m)
T_steps=size(X_Estimation,2);


NEES_add.RobotRotation=zeros(1,T_steps);
NEES_add.RobotPosition=zeros(1,T_steps);
NEES_add.RobotPose=zeros(1,T_steps);
% NEES_add.FeatureRotation=zeros(1,T_steps);
NEES_add.FeaturePosition=zeros(1,T_steps);
% NEES_add.FeaturePose=zeros(1,T_steps);
% NEES_add.Total=zeros(1,T_steps);

for i=1:T_steps
    error=pointState_Std_minus(X_Estimation{i}.state,Xstate_gt{i});
    N=round((size(error,1))/3)-2;
    
    NEES_add.RobotRotation(i)=error(1:3)'/X_Estimation{i}.cov(1:3,1:3)...
        *error(1:3)/(m*3);
    NEES_add.RobotPosition(i)=error(4:6)'/X_Estimation{i}.cov(4:6,4:6)...
        *error(4:6)/(m*3);
    NEES_add.RobotPose(i)=[error(1:3);error(4:6)]'/...
        X_Estimation{i}.cov(1:6,1:6)*[error(1:3);error(4:6)]/(m*6);
    for j=1:N
        NEES_add.FeaturePosition(i)=NEES_add.FeaturePosition(i)+error(3*j+4:3*j+6)'...
            /[X_Estimation{i}.cov(2*j+6:2*j+7,2*j+6:2*j+7) X_Estimation{i}.cov(2*j+6:2*j+7,7);...
            X_Estimation{i}.cov(7,2*j+6:2*j+7) X_Estimation{i}.cov(7,7)]...
            *error(3*j+4:3*j+6)/(m*3*N);
    end
    NEES_add.outsigB(:,i)=abs(error(1:6))>3*sqrt(diag(X_Estimation{i}.cov(1:6,1:6)));
%     NEES_add.FeatureRotation(i)=error(4:3*N+3)'/X_Estimation{i}.cov(4:3*N+3,4:3*N+3)...
%         *error(4:3*N+3)/(m*3*N);
%     NEES_add.FeaturePosition(i)=error(3*N+7:6*N+6)'/X_Estimation{i}.cov(3*N+7:6*N+6,3*N+7:6*N+6)...
%         *error(3*N+7:6*N+6)/(m*3*N);
%     NEES_add.FeaturePose(i)=[error(4:3*N+3);error(3*N+7:6*N+6)]'/...
%         [X_Estimation{i}.cov(4:3*N+3,4:3*N+3) X_Estimation{i}.cov(4:3*N+3,3*N+7:6*N+6);...
%         X_Estimation{i}.cov(3*N+7:6*N+6,4:3*N+3) X_Estimation{i}.cov(3*N+7:6*N+6,3*N+7:6*N+6)]...
%         *[error(4:3*N+3);error(3*N+7:6*N+6)]/(m*(6*N));
%     NEES_add.Total(i)=error'/X_Estimation{i}.cov*error/(m*(6*N+6));
end







