%coded by Yang SONG
clear all
clc

addpath('./Math/')
addpath('./Statistic/')

Results_record=cell(1,2);
results_save=cell(1,2);

noiseSettings
for dataID=1:2
    
    %% load Datasets
    if dataID==1
        load('data_CPplane1.mat');
        SigmaTable=SigmaTable1;
    elseif dataID==2
        load('data_CPplane2.mat');
        SigmaTable=SigmaTable2;
    end
    
    %% Measurement Noises
    
    Results_record{dataID}=zeros(3*size(SigmaTable,1),6); 
    results_save{dataID}=cell(1,size(SigmaTable,1));
    % Std: RMSE RobOri. RMSE RobTrans. RMSE Fea NEES RobPose NEES Fea Time
     % Ideal: ..
     % Aff: ..
     
     
    for noiseID=1:size(SigmaTable,1)
        sigVec=SigmaTable(noiseID,:);
        ODOM_noise = blkdiag(sigVec(1)^2*eye(3),sigVec(2)^2*eye(3)); % odometry noise cov matrix should be 6*6
        OBSV_noise=sigVec(3)^2*eye(3);
        
        %% Monte Carlo
        m=50;

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

        %MSE_ideal
        RMSE_ideal.RobotRotation=zeros(1,T_steps);
        RMSE_ideal.RobotPosition=zeros(1,T_steps);
        RMSE_ideal.FeaturePosition=zeros(1,T_steps);
        MSE_ideal.RobotRotation=zeros(1,T_steps);
        MSE_ideal.RobotPosition=zeros(1,T_steps);
        MSE_ideal.FeaturePosition=zeros(1,T_steps);

        %NEES_ideal
        NEES_ideal.RobotRotation=zeros(1,T_steps);
        NEES_ideal.RobotPosition=zeros(1,T_steps);
        NEES_ideal.RobotPose=zeros(1,T_steps);
        NEES_ideal.FeaturePosition=zeros(1,T_steps);

        %Time_ideal
        Time_ideal=0;

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


        for i=1:m

            U_noise=UaddNoise_FirstOrderInte(U,ODOM_noise);
            [z_noise,z0]=zaddNoise(z_expectation,z_expectation0,OBSV_noise);

            X_Estimation_std=CPplanefeature_StdEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            X_Estimation_ideal=CPplanefeature_idealEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise,Xstate_gt);

            X_Estimation_aff=CPplanefeature_AffEKF(X0,P0,z0,U_noise,z_noise,Index,ODOM_noise,OBSV_noise);
            


                

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

            %MSE_ideal
            MSE_ideal_add=pointfeature_MSEstd_add(X_Estimation_ideal,Xstate_gt,m);
            MSE_ideal.RobotRotation=MSE_ideal.RobotRotation+(MSE_ideal_add.RobotRotation);
            MSE_ideal.RobotPosition=MSE_ideal.RobotPosition+(MSE_ideal_add.RobotPosition);
            MSE_ideal.FeaturePosition=MSE_ideal.FeaturePosition+(MSE_ideal_add.FeaturePosition);

            %NEES_ideal
            NEES_ideal_add=pointfeature_NEES_Std_add(X_Estimation_ideal,Xstate_gt,m);
            NEES_ideal.RobotRotation=NEES_ideal.RobotRotation+NEES_ideal_add.RobotRotation;
            NEES_ideal.RobotPosition=NEES_ideal.RobotPosition+NEES_ideal_add.RobotPosition;
            NEES_ideal.RobotPose=NEES_ideal.RobotPose+NEES_ideal_add.RobotPose;
            NEES_ideal.FeaturePosition=NEES_ideal.FeaturePosition+NEES_ideal_add.FeaturePosition;

            %Time_ideal
            Time_ideal=Time_ideal+X_Estimation_ideal{1}.time/m;

            %MSE_Aff
            MSE_Aff_add=pointfeature_MSEstd_add(X_Estimation_aff,Xstate_gt,m);
            MSE_Aff.RobotRotation=MSE_Aff.RobotRotation+(MSE_Aff_add.RobotRotation);
            MSE_Aff.RobotPosition=MSE_Aff.RobotPosition+(MSE_Aff_add.RobotPosition);
            MSE_Aff.FeaturePosition=MSE_Aff.FeaturePosition+(MSE_Aff_add.FeaturePosition);

            %NEES_Aff
            NEES_Aff_add=pointfeature_NEES_Std_add(X_Estimation_aff,Xstate_gt,m);
            NEES_Aff.RobotRotation=NEES_Aff.RobotRotation+NEES_Aff_add.RobotRotation;
            NEES_Aff.RobotPosition=NEES_Aff.RobotPosition+NEES_Aff_add.RobotPosition;
            NEES_Aff.RobotPose=NEES_Aff.RobotPose+NEES_Aff_add.RobotPose;
            NEES_Aff.FeaturePosition=NEES_Aff.FeaturePosition+NEES_Aff_add.FeaturePosition;

            %Time_Aff
            Time_Aff=Time_Aff+X_Estimation_aff{1}.time/m;
            


        end
        
        results_save{dataID}{noiseID}=cell(2,3);
        
        RMSE_Std.RobotRotation=sqrt(MSE_Std.RobotRotation);
        RMSE_Std.RobotPosition=sqrt(MSE_Std.RobotPosition);
        RMSE_Std.FeaturePosition=sqrt(MSE_Std.FeaturePosition);
        
        RMSE_ideal.RobotRotation=sqrt(MSE_ideal.RobotRotation);
        RMSE_ideal.RobotPosition=sqrt(MSE_ideal.RobotPosition);
        RMSE_ideal.FeaturePosition=sqrt(MSE_ideal.FeaturePosition);
        
        RMSE_Aff.RobotRotation=sqrt(MSE_Aff.RobotRotation);
        RMSE_Aff.RobotPosition=sqrt(MSE_Aff.RobotPosition);
        RMSE_Aff.FeaturePosition=sqrt(MSE_Aff.FeaturePosition);
        
        methodID=1; %Std
        Results_record{dataID}(3*noiseID-3+methodID,1)=sum(RMSE_Std.RobotRotation)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,2)=sum(RMSE_Std.RobotPosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,3)=sum(RMSE_Std.FeaturePosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,4)=sum(NEES_Std.RobotPose)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,5)=sum(NEES_Std.FeaturePosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,6)=Time_Std;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_Std;
        results_save{dataID}{noiseID}{2,methodID}=NEES_Std;

        methodID=2; %Ideal
        Results_record{dataID}(3*noiseID-3+methodID,1)=sum(RMSE_ideal.RobotRotation)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,2)=sum(RMSE_ideal.RobotPosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,3)=sum(RMSE_ideal.FeaturePosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,4)=sum(NEES_ideal.RobotPose)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,5)=sum(NEES_ideal.FeaturePosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,6)=Time_ideal;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_ideal;
        results_save{dataID}{noiseID}{2,methodID}=NEES_ideal;

        methodID=3; %Aff
        Results_record{dataID}(3*noiseID-3+methodID,1)=sum(RMSE_Aff.RobotRotation)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,2)=sum(RMSE_Aff.RobotPosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,3)=sum(RMSE_Aff.FeaturePosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,4)=sum(NEES_Aff.RobotPose)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,5)=sum(NEES_Aff.FeaturePosition)/T_steps;
        Results_record{dataID}(3*noiseID-3+methodID,6)=Time_Aff;
        results_save{dataID}{noiseID}{1,methodID}=RMSE_Aff;
        results_save{dataID}{noiseID}{2,methodID}=NEES_Aff;
        
        
        

        
        
    end
    
end

%% plot figures

colorTable=cell(1,3);
colorTable{1}='#2F7FC1';
colorTable{2}='#EE82EE';
colorTable{3}='#D8383A';
colorTable{4}='#82B0D2';
colorTable{5}='#DDA0DD';
colorTable{6}='#FA7F6F';



LineWidthSettings=1;
for dataID=1:2
    
    if dataID==1
        noiseID=3;
    elseif dataID==2
        noiseID=1;
    end
    
    figure
    seq=1:20:T_steps;
    %% NEES
    FontSize_settings=10;
    name1=results_save{dataID}{noiseID}{2,1};
    name2=results_save{dataID}{noiseID}{2,2};
    name3=results_save{dataID}{noiseID}{2,3};
    
    figID=1;
    subplot(3,2,figID)
    plot(seq,ones(1,length(seq)),'k-.');hold on
    plot(seq,name1.RobotRotation(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.RobotRotation(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.RobotRotation(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold off
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
    plot(seq,name3.RobotPosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold off
    title('NEES for Robot Position')
    set(gca, 'FontSize',FontSize_settings)

    figID=figID+2;
    subplot(3,2,figID)
    plot(seq,ones(1,length(seq)),'k-.');hold on
    plot(seq,name1.FeaturePosition(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.FeaturePosition(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.FeaturePosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold off
    title('NEES for Feature')
    xlabel('Time Steps')
    set(gca, 'FontSize',FontSize_settings)
    
    %% RMSE
    name1=results_save{dataID}{noiseID}{1,1};
    name2=results_save{dataID}{noiseID}{1,2};
    name3=results_save{dataID}{noiseID}{1,3};
    figID=2;
    subplot(3,2,figID)
    plot(seq,name1.RobotRotation(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.RobotRotation(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.RobotRotation(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold off
    title('RMSE for Robot Orientation')
%             xlabel('Time Steps')
    ylabel('rad')
    set(gca, 'FontSize',FontSize_settings)
%             legend('Std-EKF','Ideal-EKF','Aff-EKF')

    figID=figID+2;
    subplot(3,2,figID)
    plot(seq,name1.RobotPosition(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.RobotPosition(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.RobotPosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold off
    title('RMSE for Robot Position')
    ylabel('m')
    set(gca, 'FontSize',FontSize_settings)

    figID=figID+2;
    subplot(3,2,figID)
    plot(seq,name1.FeaturePosition(seq),'color',colorTable{1},'LineWidth',LineWidthSettings);hold on
    plot(seq,name2.FeaturePosition(seq),'color',colorTable{2},'LineWidth',LineWidthSettings);hold on
    plot(seq,name3.FeaturePosition(seq),'color',colorTable{3},'LineWidth',LineWidthSettings);hold off
    title('RMSE for Feature')
    ylabel('m')
    xlabel('Time Steps')
    set(gca, 'FontSize',FontSize_settings)
    legend('Std-EKF','Ideal-EKF','Aff-EKF','NumColumns',3)
    
    

end    

    
    
        