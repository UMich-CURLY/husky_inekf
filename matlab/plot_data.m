% scp -r %mars%:/home/justin/code/husky_inekf_ws/catkin_ws/src/husky_inekf/data/3floor_data/ C:\Users\tylin\Documents\MATLAB\

%%
clear
vel_in = load("/home/justin/code/husky_inekf_ws/catkin_ws/src/husky_inekf/data/21-15_data_yaw_update/vel_input.txt");
bias_est = load("/home/justin/code/husky_inekf_ws/catkin_ws/src/husky_inekf/data/21-15_data_yaw_update/bias_est.txt");
vel_est = load("/home/justin/code/husky_inekf_ws/catkin_ws/src/husky_inekf/data/21-15_data_yaw_update/vel_est.txt");
% gt_pose = load("/home/justin/code/husky_inekf_ws/catkin_ws/src/husky_inekf/data/3floor_data/gt.txt");
est_pose = load("/home/justin/code/husky_inekf_ws/catkin_ws/src/husky_inekf/data/21-15_data_yaw_update/husky_inekf_pose_tum_yaw_update.txt");
t = est_pose(:,1);
imu = load("/home/justin/code/husky_inekf_ws/catkin_ws/src/husky_inekf/data/21-15_data_yaw_update/imu.txt");



%%  plot estimated pose with gt pose from zed slam
figure(1)

plot(t,est_pose(:,2),'LineWidth',2);
hold on
plot(t,est_pose(:,3),'LineWidth',2);
hold on
plot(t,est_pose(:,4),'LineWidth',2);
hold on

plot(gt_pose(:,1),gt_pose(:,2),'LineWidth',2);
hold on
plot(gt_pose(:,1),gt_pose(:,3),'LineWidth',2);
hold on
plot(gt_pose(:,1),gt_pose(:,4),'LineWidth',2);

legend("est_x", "est_y","est_z", "gt_x", "gt_y", "gt_z",'FontSize',20);

%%  plot estimated velocity with input velocity
figure(2)
plot(t,vel_est(:,2),'LineWidth',2);
hold on
plot(t,vel_est(:,3),'LineWidth',2);
hold on
plot(t,vel_est(:,4),'LineWidth',2);
hold on

plot(vel_in(:,1),vel_in(:,2),'LineWidth',2);
hold on
plot(vel_in(:,1),vel_in(:,3),'LineWidth',2);
hold on
plot(vel_in(:,1),vel_in(:,4),'LineWidth',2);

legend("vel\_est_x", "vel\_est_y","vel\_est_z", "vel\_in_x", "vel\_in_y", "vel\_in_z",'FontSize',20);

%%  plot bias
plot(t,bias_est(:,2),'LineWidth',2);
hold on
plot(t,bias_est(:,3),'LineWidth',2);
hold on
plot(t,bias_est(:,4),'LineWidth',2);
hold on
plot(t,bias_est(:,5),'LineWidth',2);
hold on
plot(t,bias_est(:,6),'LineWidth',2);
hold on
plot(t,bias_est(:,7),'LineWidth',2);


legend("bias_{raw}", "bias_{pitch}", "bias_{yaw}", "bias_x", "bias_y", "bias_z",'FontSize',20);

%% plot bias with estimated pose
figure(4)

plot(t,est_pose(:,2),'LineWidth',2);
hold on
plot(t,est_pose(:,3),'LineWidth',2);
hold on
plot(t,est_pose(:,4),'LineWidth',2);
hold on

plot(t,bias_est(:,2),'LineWidth',2);
hold on
plot(t,bias_est(:,3),'LineWidth',2);
hold on
plot(t,bias_est(:,4),'LineWidth',2);
hold on
plot(t,bias_est(:,5),'LineWidth',2);
hold on
plot(t,bias_est(:,6),'LineWidth',2);
hold on
plot(t,bias_est(:,7),'LineWidth',2);


legend("est_x", "est_y","est_z", "bias_{raw}", "bias_{pitch}", "bias_{yaw}", "bias_x", "bias_y", "bias_z",'FontSize',20);



%% acc
figure(5);

plot(imu(:,1),imu(1:end,5),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,6),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,7),'LineWidth',2);

legend("acc_x", "acc_y","acc_z",'FontSize',20);

%% omega
figure(6);

plot(imu(:,1),imu(1:end,2),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,3),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,4),'LineWidth',2);

legend("\omega_x", "\omega_y","\omega_z",'FontSize',20);
%%
figure(7);
th = 0.03;
fs = 80;
imu_filtered = imu;
imu_filtered(:,2) = lowpass(imu(:,2),th,fs);
imu_filtered(:,3) = lowpass(imu(:,3),th,fs);
imu_filtered(:,4) = lowpass(imu(:,4),th,fs);
imu_filtered(:,5) = lowpass(imu(:,5),th,fs);
imu_filtered(:,6) = lowpass(imu(:,6),th,fs);
imu_filtered(:,7) = lowpass(imu(:,7),th,fs);

plot(imu_filtered(:,1),imu_filtered(1:end,5),'LineWidth',2);
hold on
plot(imu_filtered(:,1),imu_filtered(1:end,6),'LineWidth',2);
hold on
plot(imu_filtered(:,1),imu_filtered(1:end,7),'LineWidth',2);

legend("acc_x", "acc_y","acc_z",'FontSize',20);

%%
figure(8);

plot(imu(:,1),imu(1:end,2),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,3),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,4),'LineWidth',2);
hold on 
plot(imu1(:,1),imu1(1:end,2),'LineWidth',2);
hold on
plot(imu1(:,1),imu1(1:end,3),'LineWidth',2);
hold on
plot(imu1(:,1),imu1(1:end,4),'LineWidth',2);


legend("\omega_x", "\omega_y","\omega_z","\omega_{x\_origin}", "\omega_{y\_origin}","\omega_{z\_origin}",'FontSize',20);


%%  plot estimated velocity with input velocity
figure(9)
plot(t,vel_est(:,2),'LineWidth',2);
% hold on
% plot(t,vel_est(:,3),'LineWidth',2);
% hold on
% plot(t,vel_est(:,4),'LineWidth',2);
hold on

plot(vel_in(:,1),vel_in(:,2),'LineWidth',2);
% hold on
% plot(vel_in(:,1),vel_in(:,3),'LineWidth',2);
hold on
% plot(vel_in(:,1),vel_in(:,4),'LineWidth',2);

plot(t,bias_est(:,5).*0.000000001,'LineWidth',2);
% hold on
% plot(t,bias_est(:,6),'LineWidth',2);
hold on
% plot(t,bias_est(:,7),'LineWidth',2);

legend("vel\_est_x",  "vel\_in_x", "bias_x", 'FontSize',20);

%% omega
figure(10);

plot(imu(:,1),imu(1:end,2),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,3),'LineWidth',2);
hold on
plot(imu(:,1),imu(1:end,4),'LineWidth',2);
hold on
plot(t,bias_est(:,2),'LineWidth',2);
hold on
plot(t,bias_est(:,3),'LineWidth',2);
hold on
plot(t,bias_est(:,4),'LineWidth',2);

legend("\omega_x", "\omega_y","\omega_z","est\_bias_{roll}", "est\_bias_{pitch}", "est\_bias_{yaw}",'FontSize',20);