%coded by Yang SONG, UTS
clear all
clc

addpath('./Math/')
addpath('./Statistic/')

Results_record=cell(1,2);
results_save=cell(1,2);

noiseSettings; % we test 3 different noise settings
Samp4_3sigBID=1;

for dataID=1:2
    
    %% load Datasets
    if dataID==1
        load('data3D_TypicalPoint.mat');
        SigmaTable=SigmaTable1;
    elseif dataID==2
        load('data3D_TypicalPoint.mat');
        SigmaTable=SigmaTable2;
    end
    
    %% Measurement Noises
    
    Results_record{dataID}=zeros(3*size(SigmaTable,1),6); 
    results_save{dataID}=cell(1,size(SigmaTable,1));
    % Std: RMSE RobOri. RMSE RobTrans. RMSE Fea NEES RobPose NEES Fea Time
     % RI: ..
     % Aff: ..
     
     
    for noiseID=1:size(SigmaTable,1)
        sigVec=SigmaTable(noiseID,:);
        ODOM_noise = blkdiag(sigVec(1)^2*eye(3),sigVec(2)^2*eye(3)); % Cov of odometry noise
        OBSV_noise=sigVec(3)^2*eye(3); % Cov of observation noise
        
        %% Monte Carlo
        m=50; % number of Monte Carlo runs

        %MSE_Std
        RMSE_Std.RobotRotation=zeros(1,T_steps);
        RMSE_Std.RobotPosition=zeros(1,T_steps);
        RMSE_Std.FeaturePosition=zeros(1,T_steps);
        MSE_Std.RobotRotation=zeros(1,T_steps);
        MSE_Std.RobotPosition=zeros(1,T_steps);
        MSE_Std.FeaturePosition=zeros(1,T_steps);

        %NEES_Std
        NEES_Std.RobotRotation=zeros(1,T_steps);
        NEES_Std.RobotPosition=zeros(1,T_steps);
        NEES_Std.RobotPose=zeros(1,T_steps);
        NEES_Std.FeaturePosition=zeros(1,T_steps);

        %Time_Std
        Time_Std=0;
        


        %MSE_RI
        RMSE_RI.RobotRotation=zeros(1,T_steps);
        RMSE_RI.RobotPosition=zeros(1,T_steps);
        RMSE_RI.FeaturePosition=zeros(1,T_steps);
        MSE_RI.RobotRotation=zeros(1,T_steps);
        MSE_RI.RobotPosition=zeros(1,T_steps);
        MSE_RI.FeaturePosition=zeros(1,T_steps);

        %NEES_RI
        NEES_RI.RobotRotation=zeros(1,T_steps);
        NEES_RI.RobotPosition=zeros(1,T_steps);
        NEES_RI.RobotPose=zeros(1,T_steps);
        NEES_RI.FeaturePosition=zeros(1,T_steps);

        %Time_ri
        Time_RI=0;

        %MSE_Aff
        RMSE_Aff.RobotRotation=zeros(1,T_steps);
        RMSE_Aff.RobotPosition=zeros(1,T_steps);
        RMSE_Aff.FeaturePosition=zeros(1,T_steps);
        MSE_Aff.RobotRotation=zeros(1,T_steps);
        MSE_Aff.RobotPosition=zeros(1,T_steps);
        MSE_Aff.FeaturePosition=zeros(1,T_steps);

        %NEES_Aff
        NEES_Aff.RobotRotation=zeros(1,T_steps);
        NEES_Aff.RobotPosition=zeros(1,T_steps);
        NEES_Aff.RobotPose=zeros(1,T_steps);
        NEES_Aff.FeaturePosition=zeros(1,T_steps);

        %Time_Aff
        Time_Aff=0;
        
        %MSE_Aff2
        RMSE_Aff2.RobotRotation=zeros(1,T_steps);
        RMSE_Aff2.RobotPosition=zeros(1,T_steps);
        RMSE_Aff2.FeaturePosition=zeros(1,T_steps);
        MSE_Aff2.RobotRotation=zeros(1,T_steps);
        MSE_Aff2.RobotPosition=zeros(1,T_steps);
        MSE_Aff2.FeaturePosition=zeros(1,T_steps);

        %NEES_Aff2
        NEES_Aff2.RobotRotation=zeros(1,T_steps);
        NEES_Aff2.RobotPosition=zeros(1,T_steps);
        NEES_Aff2.RobotPose=zeros(1,T_steps);
        NEES_Aff2.FeaturePosition=zeros(1,T_steps);

        %Time_Aff2
        Time_Aff2=0;
        
        %MSE_FEJ
        RMSE_FEJ.RobotRotation=zeros(1,T_steps);
        RMSE_FEJ.RobotPosition=zeros(1,T_steps);
        RMSE_FEJ.FeaturePosition=zeros(1,T_steps);
        MSE_FEJ.RobotRotation=zeros(1,T_steps);
        MSE_FEJ.RobotPosition=zeros(1,T_steps);
        MSE_FEJ.FeaturePosition=zeros(1,T_steps);

        %NEES_FEJ
        NEES_FEJ.RobotRotation=zeros(1,T_steps);
        NEES_FEJ.RobotPosition=zeros(1,T_steps);
        NEES_FEJ.RobotPose=zeros(1,T_steps);
        NEES_FEJ.FeaturePosition=zeros(1,T_steps);

        %Time_FEJ
        Time_FEJ=0;
        
        %MSE_FEJ2
        RMSE_FEJ2.RobotRotation=zeros(1,T_steps);
        RMSE_FEJ2.RobotPosition=zeros(1,T_steps);
        RMSE_FEJ2.FeaturePosition=zeros(1,T_steps);
        MSE_FEJ2.RobotRotation=zeros(1,T_steps);
        MSE_FEJ2.RobotPosition=zeros(1,T_steps);
        MSE_FEJ2.FeaturePosition=zeros(1,T_steps);

        %NEES_FEJ2
        NEES_FEJ2.RobotRotation=zeros(1,T_steps);
        NEES_FEJ2.RobotPosition=zeros(1,T_steps);
        NEES_FEJ2.RobotPose=zeros(1,T_steps);
        NEES_FEJ2.FeaturePosition=zeros(1,T_steps);

        %Time_FEJ2
        Time_FEJ2=0;
        
        %MSE_OC
        RMSE_OC.RobotRotation=zeros(1,T_steps);
        RMSE_OC.RobotPosition=zeros(1,T_steps);
        RMSE_OC.FeaturePosition=zeros(1,T_steps);
        MSE_OC.RobotRotation=zeros(1,T_steps);
        MSE_OC.RobotPosition=zeros(1,T_steps);
        MSE_OC.FeaturePosition=zeros(1,T_steps);

        %NEES_OC
        NEES_OC.RobotRotation=zeros(1,T_steps);
        NEES_OC.RobotPosition=zeros(1,T_steps);
        NEES_OC.RobotPose=zeros(1,T_steps);
        NEES_OC.FeaturePosition=zeros(1,T_steps);

        %Time_OC
        Time_OC=0;
        
        %MSE_DRI
        RMSE_DRI.RobotRotation=zeros(1,T_steps);
        RMSE_DRI.RobotPosition=zeros(1,T_steps);
        RMSE_DRI.FeaturePosition=zeros(1,T_steps);
        MSE_DRI.RobotRotation=zeros(1,T_steps);
        MSE_DRI.RobotPosition=zeros(1,T_steps);
        MSE_DRI.FeaturePosition=zeros(1,T_steps);

        %NEES_DRI
        NEES_DRI.RobotRotation=zeros(1,T_steps);
        NEES_DRI.RobotPosition=zeros(1,T_steps);
        NEES_DRI.RobotPose=zeros(1,T_steps);
        NEES_DRI.FeaturePosition=zeros(1,T_steps);

        %Time_DRI
        Time_DRI=0;
        
        
        
        


        for i=1:m

            U_noise=UaddNoise_FirstOrderInte(U,ODOM_noise); % add noises to the odometry
            [z_noise,z0]=zaddNoise_point(z_expectation,z_expectation0,OBSV_noise); % add noises to the observations
            
            
            X_Estimation_ri=pointfeature_RIEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_std=pointfeature_StdEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_FEJ=pointfeature_FEJEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_OC=pointfeature_OCEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_FEJ2=pointfeature_FEJ2EKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_DRI=pointfeature_DRIEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_aff_v1=pointfeature_AffEKF_v1(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_aff_v2=pointfeature_AffEKF_v2(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);

            %MSE_Std
            MSE_Std_add=pointfeature_MSEstd_add(X_Estimation_std,Xstate_gt,m);
            MSE_Std.RobotRotation=MSE_Std.RobotRotation+(MSE_Std_add.RobotRotation);
            MSE_Std.RobotPosition=MSE_Std.RobotPosition+(MSE_Std_add.RobotPosition);
            MSE_Std.FeaturePosition=MSE_Std.FeaturePosition+(MSE_Std_add.FeaturePosition);

            %NEES_Std
            NEES_Std_add=pointfeature_NEES_Std_add(X_Estimation_std,Xstate_gt,m);
            NEES_Std.RobotRotation=NEES_Std.RobotRotation+NEES_Std_add.RobotRotation;
            NEES_Std.RobotPosition=NEES_Std.RobotPosition+NEES_Std_add.RobotPosition;
            NEES_Std.RobotPose=NEES_Std.RobotPose+NEES_Std_add.RobotPose;
            NEES_Std.FeaturePosition=NEES_Std.FeaturePosition+NEES_Std_add.FeaturePosition;

            %Time_Std
            Time_Std=Time_Std+X_Estimation_std{1}.time/m;
            
            

            %MSE_RI
            MSE_RI_add=pointfeature_MSEstd_add(X_Estimation_ri,Xstate_gt,m);
            MSE_RI.RobotRotation=MSE_RI.RobotRotation+(MSE_RI_add.RobotRotation);
            MSE_RI.RobotPosition=MSE_RI.RobotPosition+(MSE_RI_add.RobotPosition);
            MSE_RI.FeaturePosition=MSE_RI.FeaturePosition+(MSE_RI_add.FeaturePosition);

            %NEES_RI
            NEES_RI_add=pointfeature_NEES_add(X_Estimation_ri,Xstate_gt,m);
            NEES_RI.RobotRotation=NEES_RI.RobotRotation+NEES_RI_add.RobotRotation;
            NEES_RI.RobotPosition=NEES_RI.RobotPosition+NEES_RI_add.RobotPosition;
            NEES_RI.RobotPose=NEES_RI.RobotPose+NEES_RI_add.RobotPose;
            NEES_RI.FeaturePosition=NEES_RI.FeaturePosition+NEES_RI_add.FeaturePosition;

            %Time_RI
            Time_RI=Time_RI+X_Estimation_ri{1}.time/m;

            %MSE_Aff
            MSE_Aff_add=pointfeature_MSEstd_add(X_Estimation_aff_v1,Xstate_gt,m);
            MSE_Aff.RobotRotation=MSE_Aff.RobotRotation+(MSE_Aff_add.RobotRotation);
            MSE_Aff.RobotPosition=MSE_Aff.RobotPosition+(MSE_Aff_add.RobotPosition);
            MSE_Aff.FeaturePosition=MSE_Aff.FeaturePosition+(MSE_Aff_add.FeaturePosition);

            %NEES_Aff
            NEES_Aff_add=pointfeature_NEES_Std_add(X_Estimation_aff_v1,Xstate_gt,m);
            NEES_Aff.RobotRotation=NEES_Aff.RobotRotation+NEES_Aff_add.RobotRotation;
            NEES_Aff.RobotPosition=NEES_Aff.RobotPosition+NEES_Aff_add.RobotPosition;
            NEES_Aff.RobotPose=NEES_Aff.RobotPose+NEES_Aff_add.RobotPose;
            NEES_Aff.FeaturePosition=NEES_Aff.FeaturePosition+NEES_Aff_add.FeaturePosition;

            %Time_Aff
            Time_Aff=Time_Aff+X_Estimation_aff_v1{1}.time/m;
            
            %MSE_Aff2
            MSE_Aff2_add=pointfeature_MSEstd_add(X_Estimation_aff_v2,Xstate_gt,m);
            MSE_Aff2.RobotRotation=MSE_Aff2.RobotRotation+(MSE_Aff2_add.RobotRotation);
            MSE_Aff2.RobotPosition=MSE_Aff2.RobotPosition+(MSE_Aff2_add.RobotPosition);
            MSE_Aff2.FeaturePosition=MSE_Aff2.FeaturePosition+(MSE_Aff2_add.FeaturePosition);

            %NEES_Aff2
            NEES_Aff2_add=pointfeature_NEES_Std_add(X_Estimation_aff_v2,Xstate_gt,m);
            NEES_Aff2.RobotRotation=NEES_Aff2.RobotRotation+NEES_Aff2_add.RobotRotation;
            NEES_Aff2.RobotPosition=NEES_Aff2.RobotPosition+NEES_Aff2_add.RobotPosition;
            NEES_Aff2.RobotPose=NEES_Aff2.RobotPose+NEES_Aff2_add.RobotPose;
            NEES_Aff2.FeaturePosition=NEES_Aff2.FeaturePosition+NEES_Aff2_add.FeaturePosition;

            %Time_Aff2
            Time_Aff2=Time_Aff2+X_Estimation_aff_v2{1}.time/m;
            
            %MSE_FEJ
            MSE_FEJ_add=pointfeature_MSEstd_add(X_Estimation_FEJ,Xstate_gt,m);
            MSE_FEJ.RobotRotation=MSE_FEJ.RobotRotation+MSE_FEJ_add.RobotRotation;
            MSE_FEJ.RobotPosition=MSE_FEJ.RobotPosition+MSE_FEJ_add.RobotPosition;
            MSE_FEJ.FeaturePosition=MSE_FEJ.FeaturePosition+MSE_FEJ_add.FeaturePosition;

            %NEES_FEJ
            NEES_FEJ_add=pointfeature_NEES_Std_add(X_Estimation_FEJ,Xstate_gt,m);
            NEES_FEJ.RobotRotation=NEES_FEJ.RobotRotation+NEES_FEJ_add.RobotRotation;
            NEES_FEJ.RobotPosition=NEES_FEJ.RobotPosition+NEES_FEJ_add.RobotPosition;
            NEES_FEJ.RobotPose=NEES_FEJ.RobotPose+NEES_FEJ_add.RobotPose;
            NEES_FEJ.FeaturePosition=NEES_FEJ.FeaturePosition+NEES_FEJ_add.FeaturePosition;
            
            %Time_FEJ
            Time_FEJ=Time_FEJ+X_Estimation_FEJ{1}.time/m;

        %MSE_OC
            MSE_OC_add=pointfeature_MSEstd_add(X_Estimation_OC,Xstate_gt,m);
            MSE_OC.RobotRotation=MSE_OC.RobotRotation+MSE_OC_add.RobotRotation;
            MSE_OC.RobotPosition=MSE_OC.RobotPosition+MSE_OC_add.RobotPosition;
            MSE_OC.FeaturePosition=MSE_OC.FeaturePosition+MSE_OC_add.FeaturePosition;

            %NEES_OC
            NEES_OC_add=pointfeature_NEES_Std_add(X_Estimation_OC,Xstate_gt,m);
            NEES_OC.RobotRotation=NEES_OC.RobotRotation+NEES_OC_add.RobotRotation;
            NEES_OC.RobotPosition=NEES_OC.RobotPosition+NEES_OC_add.RobotPosition;
            NEES_OC.RobotPose=NEES_OC.RobotPose+NEES_OC_add.RobotPose;
            NEES_OC.FeaturePosition=NEES_OC.FeaturePosition+NEES_OC_add.FeaturePosition;
            
            %Time_OC
            Time_OC=Time_OC+X_Estimation_OC{1}.time/m;

            %MSE_FEJ2
            MSE_FEJ2_add=pointfeature_MSEstd_add(X_Estimation_FEJ2,Xstate_gt,m);
            MSE_FEJ2.RobotRotation=MSE_FEJ2.RobotRotation+MSE_FEJ2_add.RobotRotation;
            MSE_FEJ2.RobotPosition=MSE_FEJ2.RobotPosition+MSE_FEJ2_add.RobotPosition;
            MSE_FEJ2.FeaturePosition=MSE_FEJ2.FeaturePosition+MSE_FEJ2_add.FeaturePosition;

            %NEES_FEJ2
            NEES_FEJ2_add=pointfeature_NEES_Std_add(X_Estimation_FEJ2,Xstate_gt,m);
            NEES_FEJ2.RobotRotation=NEES_FEJ2.RobotRotation+NEES_FEJ2_add.RobotRotation;
            NEES_FEJ2.RobotPosition=NEES_FEJ2.RobotPosition+NEES_FEJ2_add.RobotPosition;
            NEES_FEJ2.RobotPose=NEES_FEJ2.RobotPose+NEES_FEJ2_add.RobotPose;
            NEES_FEJ2.FeaturePosition=NEES_FEJ2.FeaturePosition+NEES_FEJ2_add.FeaturePosition;
            
            %Time_FEJ2
            Time_FEJ2=Time_FEJ2+X_Estimation_FEJ2{1}.time/m;

            %MSE_DRI
            MSE_DRI_add=pointfeature_MSEstd_add(X_Estimation_DRI,Xstate_gt,m);
            MSE_DRI.RobotRotation=MSE_DRI.RobotRotation+MSE_DRI_add.RobotRotation;
            MSE_DRI.RobotPosition=MSE_DRI.RobotPosition+MSE_DRI_add.RobotPosition;
            MSE_DRI.FeaturePosition=MSE_DRI.FeaturePosition+MSE_DRI_add.FeaturePosition;

            %NEES_DRI
            NEES_DRI_add=pointfeature_NEES_DRI_add(X_Estimation_DRI,Xstate_gt,m);
            NEES_DRI.RobotRotation=NEES_DRI.RobotRotation+NEES_DRI_add.RobotRotation;
            NEES_DRI.RobotPosition=NEES_DRI.RobotPosition+NEES_DRI_add.RobotPosition;
            NEES_DRI.RobotPose=NEES_DRI.RobotPose+NEES_DRI_add.RobotPose;
            NEES_DRI.FeaturePosition=NEES_DRI.FeaturePosition+NEES_DRI_add.FeaturePosition;

            %Time_DRI
            Time_DRI=Time_DRI+X_Estimation_DRI{1}.time/m;
            
            
            

        end
        

        
        results_save{dataID}{noiseID}=cell(2,8);
        
        
        RMSE_RI.RobotRotation=sqrt(MSE_RI.RobotRotation);
        RMSE_RI.RobotPosition=sqrt(MSE_RI.RobotPosition);
        RMSE_RI.FeaturePosition=sqrt(MSE_RI.FeaturePosition);
        
        RMSE_Std.RobotRotation=sqrt(MSE_Std.RobotRotation);
        RMSE_Std.RobotPosition=sqrt(MSE_Std.RobotPosition);
        RMSE_Std.FeaturePosition=sqrt(MSE_Std.FeaturePosition);
        
        
        RMSE_Aff.RobotRotation=sqrt(MSE_Aff.RobotRotation);
        RMSE_Aff.RobotPosition=sqrt(MSE_Aff.RobotPosition);
        RMSE_Aff.FeaturePosition=sqrt(MSE_Aff.FeaturePosition);
        
        RMSE_Aff2.RobotRotation=sqrt(MSE_Aff2.RobotRotation);
        RMSE_Aff2.RobotPosition=sqrt(MSE_Aff2.RobotPosition);
        RMSE_Aff2.FeaturePosition=sqrt(MSE_Aff2.FeaturePosition);
        
        RMSE_FEJ.RobotRotation=sqrt(MSE_FEJ.RobotRotation);
        RMSE_FEJ.RobotPosition=sqrt(MSE_FEJ.RobotPosition);
        RMSE_FEJ.FeaturePosition=sqrt(MSE_FEJ.FeaturePosition);

        RMSE_OC.RobotRotation=sqrt(MSE_OC.RobotRotation);
        RMSE_OC.RobotPosition=sqrt(MSE_OC.RobotPosition);
        RMSE_OC.FeaturePosition=sqrt(MSE_OC.FeaturePosition);

        RMSE_FEJ2.RobotRotation=sqrt(MSE_FEJ2.RobotRotation);
        RMSE_FEJ2.RobotPosition=sqrt(MSE_FEJ2.RobotPosition);
        RMSE_FEJ2.FeaturePosition=sqrt(MSE_FEJ2.FeaturePosition);

        RMSE_DRI.RobotRotation=sqrt(MSE_DRI.RobotRotation);
        RMSE_DRI.RobotPosition=sqrt(MSE_DRI.RobotPosition);
        RMSE_DRI.FeaturePosition=sqrt(MSE_DRI.FeaturePosition);
        
        num_Method=8;
        
        
        methodID=1; %Std
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_Std.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_Std.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_Std.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_Std.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_Std.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_Std;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_Std;
        results_save{dataID}{noiseID}{2,methodID}=NEES_Std;
        
        methodID=2; %FEJ
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_FEJ.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_FEJ.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_FEJ.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_FEJ.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_FEJ.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_FEJ;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_FEJ;
        results_save{dataID}{noiseID}{2,methodID}=NEES_FEJ;
        
        methodID=3; %OC
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_OC.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_OC.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_OC.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_OC.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_OC.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_OC;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_OC;
        results_save{dataID}{noiseID}{2,methodID}=NEES_OC;
        
        methodID=4; %FEJ2
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_FEJ2.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_FEJ2.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_FEJ2.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_FEJ2.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_FEJ2.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_FEJ2;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_FEJ2;
        results_save{dataID}{noiseID}{2,methodID}=NEES_FEJ2;
        
        
        methodID=5; %DRI
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_DRI.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_DRI.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_DRI.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_DRI.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_DRI.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_DRI;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_DRI;
        results_save{dataID}{noiseID}{2,methodID}=NEES_DRI;
        
        methodID=6; %RI
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_RI.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_RI.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_RI.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_RI.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_RI.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_RI;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_RI;
        results_save{dataID}{noiseID}{2,methodID}=NEES_RI;
        
        methodID=7; %Aff_v1
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_Aff.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_Aff.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_Aff.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_Aff.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_Aff.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_Aff;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_Aff;
        results_save{dataID}{noiseID}{2,methodID}=NEES_Aff;
        
        methodID=8; %Aff_v2
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,1)=sum(RMSE_Aff2.RobotRotation)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,2)=sum(RMSE_Aff2.RobotPosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,3)=sum(RMSE_Aff2.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,4)=sum(NEES_Aff2.RobotPose)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,5)=sum(NEES_Aff2.FeaturePosition)/T_steps;
        Results_record{dataID}(num_Method*noiseID-num_Method+methodID,6)=Time_Aff2;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_Aff2;
        results_save{dataID}{noiseID}{2,methodID}=NEES_Aff2;
        

        

        
        
        
    end
    
end

%% plot NEES and RMSE results of Std-EKF, RI-EKF, Aff-EKF v1 and Aff-EKF v2
colorTable=cell(1,3);
colorTable{1}='#2F7FC1';
colorTable{2}='#2E8B57';
colorTable{3}='#c195e8';
colorTable{4}='#D8383A';

colorTable{5}='#82B0D2';
colorTable{6}='#3CB371';
colorTable{7}='#af72e5';
colorTable{8}='#FA7F6F';



LineWidthSettings=1;
for dataID=1 %:2

    noiseID=1; % 2; 3;
    
    figure
    seq=1:20:T_steps;
    %% NEES
    FontSize_settings=10;
    name1=results_save{dataID}{noiseID}{2,1};
    name2=results_save{dataID}{noiseID}{2,6};
    name3=results_save{dataID}{noiseID}{2,7};
    name4=results_save{dataID}{noiseID}{2,8};
    
    figID=1;
    subplot(3,2,figID)
    plot(seq,ones(1,length(seq)),'k-.');hold on
    plot(seq,name1.RobotRotation(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.RobotRotation(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.RobotRotation(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold on
    plot(seq,name4.RobotRotation(seq),'color',colorTable{4},'LineWidth',LineWidthSettings);hold off
    title('NEES for Robot Orientation')
    set(gca, 'FontSize',FontSize_settings)
%             xlabel('Time Steps')
%             ylabel('(rad)')
%             legend('Std-EKF','Ideal-EKF','Aff-EKF')

    figID=figID+2;
    subplot(3,2,figID)
    plot(seq,ones(1,length(seq)),'k-.');hold on
    plot(seq,name1.RobotPosition(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.RobotPosition(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.RobotPosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold on
    plot(seq,name4.RobotPosition(seq),'color',colorTable{4},'LineWidth',LineWidthSettings);hold off
    title('NEES for Robot Position')
    set(gca, 'FontSize',FontSize_settings)

    figID=figID+2;
    subplot(3,2,figID)
    plot(seq,ones(1,length(seq)),'k-.');hold on
    plot(seq,name1.FeaturePosition(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.FeaturePosition(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.FeaturePosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold on
    plot(seq,name4.FeaturePosition(seq),'color',colorTable{4},'LineWidth',LineWidthSettings);hold off
    title('NEES for Feature')
    xlabel('Time Steps')
    set(gca, 'FontSize',FontSize_settings)
    
    %% RMSE
    name1=results_save{dataID}{noiseID}{1,1};
    name2=results_save{dataID}{noiseID}{1,6};
    name3=results_save{dataID}{noiseID}{1,7};
    name4=results_save{dataID}{noiseID}{1,8};
    figID=2;
    subplot(3,2,figID)
    plot(seq,name1.RobotRotation(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.RobotRotation(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.RobotRotation(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold on
    plot(seq,name4.RobotRotation(seq),'color',colorTable{4},'LineWidth',LineWidthSettings);hold off
    title('RMSE for Robot Orientation')
    ylabel('rad')
    set(gca, 'FontSize',FontSize_settings)
%             legend('Std-EKF','Ideal-EKF','Aff-EKF')

    figID=figID+2;
    subplot(3,2,figID)
    plot(seq,name1.RobotPosition(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.RobotPosition(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.RobotPosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold on
    plot(seq,name4.RobotPosition(seq),'color',colorTable{4},'LineWidth',LineWidthSettings);hold off
    title('RMSE for Robot Position')
    ylabel('m')
    set(gca, 'FontSize',FontSize_settings)

    figID=figID+2;
    subplot(3,2,figID)
    plot(seq,name1.FeaturePosition(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.FeaturePosition(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.FeaturePosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold on
    plot(seq,name4.FeaturePosition(seq),'color',colorTable{4},'LineWidth',LineWidthSettings);hold off
    title('RMSE for Feature')
    ylabel('m')
    xlabel('Time Steps')
    set(gca, 'FontSize',FontSize_settings)
    legend('Std-EKF','RI-EKF','Aff-EKF v1','Aff-EKF v2','NumColumns',num_Method)



end 
% save results_EKF_PointFeature