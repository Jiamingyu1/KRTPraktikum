function [a_d,b_d,mag] = trajectorygeneration()
%% Trajectory generation

          % Arrival, Waypoints, Orientation
constraints = [0,      0,-27,1,    0,0,0;
               0.5,    0,-25,1,    0,0,0;
               1.5,    0,-20,1,    0,0,0;
               2,      0,-15,1,    0,0,0;
               3,      0,-7.5,1,   0,0,0;
               3.75,   3,-5,1,     18,0,0;
               5,      15,-3,1,    90,0,0;
               5.5,    25,-3,1,    90,0,0;
               6,      50,-3,1,    90,0,0;
               6.5,    65,-3,1,    90,0,0;
               7.5,    78,-3,1,    90,0,0;          
               8.5,    88,-6,1,    162,0,0;
               8.75,   89,-7.5,1,  180,0,0;
               11,     90,-20,1    180,0,0;
               13,     90,-20,1    0,0,0;
               13.5,   90,-15,1    0,0,0;
               14.5,   90,-7.5,1   0,0,0;
               15.25,  93,-5,1     18,0,0;
               16,     105,-3,1    90,0,0;
               16.5,   120,-3,1    90,0,0;
               17.25,  150,-3,1    90,0,0;
               18,     200,-3,1    90,0,0;
               18.75,  250,-3,1    90,0,0;
               19.5,   300,-3,1    90,0,0;
               20.25,  350,-3,1    90,0,0;
               21,     400,-3,1    90,0,0;
               21.5,   422,-3,1    90,0,0;
               22.25,  438,-3,1    90,0,0;
               23,     448,-6,1    162,0,0;
               23.25,  449,-7.5,1  180,0,0;
               24.25,  450,-15,1   180,0,0;
               25,     450,-20,1   180,0,0;
               26.05,  450,-20,0   180,0,0;
               26.5,   450,-15,0   180,0,0;
               27.75,  449,-7.5,0  0,0,0;
               28,     448,-6,0     290,0,0;
               28.5,   438,-3.5,0   270,0,0;
               29,     422,-3.5,0     270,0,0;
               29.5,   400,-3.5,0     270,0,0;
               30.5,   350,-3.5,0     270,0,0;
               31.5,   300,-3.5,0     270,0,0;
               32.5,   250,-3.5,0     270,0,0;
               33.5,   200,-3.5,0     270,0,0;
               34.5,   150,-3.5,0     270,0,0;
               35.5,   100,-3.5,0     270,0,0;
               36.5,   50,-3.5,0      270,0,0;
               37.5,   25,-3.5,0      270,0,0;
               38,     15,-3.5,0      270,0,0;
               38.5,   2,-6,0      252,0,0;
               39,     0,-7.5,0    180,0,0;
               39.5,   0,-10,0     180,0,0;
               40,     0,-20,0     180,0,0;
               40.5,   0,-25,0     180,0,0;
               41,     0,-27,0     180,0,0;
               ];

trajectory = waypointTrajectory(constraints(:,2:4), ...
    'TimeOfArrival',constraints(:,1), ...
    'Orientation',quaternion(constraints(:,5:7),'eulerd','ZYX','frame'));

tInfo = waypointInfo(trajectory);
trajectory.SampleRate = 10;
figure(1)
plot(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),'b*')
title('Position')
% axis([-20,480,-30,50])
xlabel('Travel')
ylabel('Elevation')
grid on
% daspect([1 1 1])
hold on

trajectory.SampleRate = 10;
orientationLog = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,'quaternion');
count = 1;
 currentPositionData = zeros(5000,2);
while ~isDone(trajectory)
   [currentPosition,orientationLog(count)] = trajectory();
   i = 1;
   for i = 1:3
%    currentPositionData(count,i) = currentPosition(i);
   if i ==1 
       a_d(count,1) = count*0.3;
       a_d(count,2) = currentPosition(i);
   end
   if i ==2 
       b_d(count,1) = count*0.3;
       b_d(count,2) = currentPosition(i);
   end
   if i ==3 
       mag(count,1) = count*0.3;
       mag(count,2) = currentPosition(i);
   end
   end
   plot(currentPosition(1),currentPosition(2), 'r.')
%    pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
   count = count+1;
end
% plot(a_d(:,2),b_d(:,2),'r-','LineWidth',1)
% hold off
end
