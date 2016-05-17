function ContFig = visInverseKin(params,ikinParam)
%VISINVERSEKIN visualizes the results on the inverse kinematics of the
%              robot iCub.
%   VISINVERSEKIN verifies if the joint reference trajectory calculated in
%   the inverse kinematics solver fits with the contact constraints and the
%   desired CoM and momentum trajectories.
%
%   ContFig = VISINVERSEKIN(params,ikinParam) takes as input the
%   structure params containing all the utility parameters, and the structure
%   ikinParam which contains the visualization parameters. The output ContFig
%   is a counter which automatically modifies the figure number according 
%   to the visualization setup.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
set(0,'DefaultFigureWindowStyle','Docked');
ContFig                 = params.ContFig;
t                       = ikinParam.t;
CoMTrajectoryError      = ikinParam.CoMTrajectoryError;
feetError               = ikinParam.feetError;
momentumError           = ikinParam.momentumError;

%% CoM trajectory
titCoM  = {'X axis' 'Y axis' 'Z axis'};

for k = 1:3

% POSITION
figure(ContFig)
subplot(3,1,k)
plot(t,CoMTrajectoryError(k,:),'b')
hold on
grid on
plot(t,CoMTrajectoryError(k+9,:),'r')
xlabel('Time [s]')
ylabel('Position [m]')
title(titCoM(k))
legend('desired CoM pos','ikin CoM pos')

% VELOCITY
figure(ContFig+1)
subplot(3,1,k)
plot(t,CoMTrajectoryError(k+3,:),'b')
hold on
grid on
plot(t,CoMTrajectoryError(k+9+3,:),'r')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
title(titCoM(k))
legend('desired CoM vel','ikin CoM vel')

% ACCELERATION
figure(ContFig+2)
subplot(3,1,k)
plot(t,CoMTrajectoryError(k+6,:),'b')
hold on
grid on
plot(t,CoMTrajectoryError(k+9+6,:),'r')
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
title(titCoM(k))
legend('desired CoM acc','ikin CoM acc')
end

ContFig = ContFig +3;

%% Feet errors
titFeet  = {'Error on left foot pos along x axis','Error on left foot pos along y axis','Error on left foot pos along z axis',...
            'Error on left foot ori along x axis','Error on left foot ori along y axis','Error on left foot ori along z axis',...
            'Error on right foot pos along x axis','Error on right foot pos along y axis','Error on right foot pos along z axis',...
            'Error on right foot ori along x axis','Error on right foot ori along y axis','Error on right foot ori along z axis'};
    
titYFeet = {'[m]' '[m]' '[m]' '[rad]' '[rad]' '[rad]'};

if     params.numConstraints == 1 
  
  for k=1:6

  figure(ContFig)
  subplot(3,2,k)
  plot(t,feetError(k,:))
  grid on
  xlabel('Time [s]')
  ylabel(titYFeet(k))
   
  if params.feet_on_ground(1) == 1
    
  title(titFeet(k))
  else
  title(titFeet(k+6)) 
  end
  end
 
ContFig = ContFig +1;

elseif params.numConstraints  == 2

for k=1:6

figure(ContFig)
subplot(3,2,k)
plot(t,feetError(k,:))
grid on
title(titFeet(k))
xlabel('Time [s]')
ylabel(titYFeet(k))

figure(ContFig+1)
subplot(3,2,k)
plot(t,feetError(k+6,:))
grid on
title(titFeet(k+6))
xlabel('Time [s]')
ylabel(titYFeet(k))
end

ContFig = ContFig +2;
end    

%% Momentum error
figure(ContFig)
subplot(2,1,1)
hold all
grid on
plot(t,momentumError(1:3,:))
xlabel('Time [s]')
ylabel('H_{Lin}-H_{Lin}^{d}')
title('Linear Momentum Error (Inverse Kinematics)')

subplot(2,1,2)
hold all
grid on
plot(t,momentumError(4:6,:))
xlabel('Time [s]')
ylabel('H_{Ang}-H_{Ang}^{d}')
title('Angular Momentum Error (Inverse Kinematics)')

ContFig = ContFig +1;
set(0,'DefaultFigureWindowStyle','Normal');

end
