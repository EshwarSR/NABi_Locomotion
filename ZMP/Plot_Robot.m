foldername = 'C:\Walking\';
RobotVideo = VideoWriter([foldername,'Walking','.avi']);
RobotVideo.FrameRate = 10; %can adjust this 5 - 10 works well
open(RobotVideo);
  
for i = 1:1:901
    r = [0; X_com(i) ; 0]; R = [ 1 0 0; 0 1 0; 0 0 1]; 
    NABIRoS_model(8,r,R,deg2rad(theta3(i)),deg2rad(theta4(i)),deg2rad(theta1(i)),deg2rad(theta2(i)));
    hold on
    plot3([0 0],[-0.4 0.9],[-0.735 -0.735])
    text(0,0.5,['Iteration :',num2str(i)])
    frame = getframe(gcf);
    writeVideo(RobotVideo, frame);
    delete(gcf);
end

close(RobotVideo)