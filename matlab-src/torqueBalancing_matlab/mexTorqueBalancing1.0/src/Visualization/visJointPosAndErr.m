%% visJointPosAndErr
% visualizes the joint position and the joint position error
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [] = visJointPosAndErr(t,caseJointPos,caseJointErr,qj,qjInit,qjErr)
%% Joints positions
if caseJointPos == 1
    
for k=1:5
  
% Robot arms    
figure(17)
subplot(3,2,k)
plot(t,qj(k+3,:))
hold on
plot(t,qjInit(k+3,:),'k')
grid on
xlabel('s')
ylabel('rad')
name = whatname('left_arm',k);
title(name)
legend('qj','qjDes')

figure(18)
subplot(3,2,k)
plot(t,qj(k+3+5,:))
hold on
plot(t,qjInit(k+3+5,:),'k')
grid on
xlabel('s')
ylabel('rad')
name = whatname('right_arm',k);
title(name)
legend('qj','qjDes')
end

for k=1:6

% Robot legs
figure(19)
subplot(3,2,k)
plot(t,qj(k+13,:))
hold on
plot(t,qjInit(k+13,:),'k')
grid on
xlabel('s')
ylabel('rad')
name = whatname('left_leg',k);
title(name)
legend('qj','qjDes')

figure(20)
subplot(3,2,k)
plot(t,qj(k+13+6,:))
hold on
plot(t,qjInit(k+13+6,:),'k')
grid on
xlabel('s')
ylabel('rad')
name = whatname('right_leg',k);
title(name)
legend('qj','qjDes')
end

for k=1:3
    
% Robot torso
figure(21)
subplot(3,1,k)
plot(t,qj(k,:))
hold on
plot(t,qjInit(k,:),'k')
grid on
xlabel('s')
ylabel('rad')
name = whatname('torso',k);
title(name)  
legend('qj','qjDes')
end

end

%% Errors at joints
if caseJointErr == 1
col = 'r';

for k=1:5
  
% Robot arms    
figure(22)
subplot(3,2,k)
plot(t,qjErr(k+3,:),col)
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('left_arm',k);
title(name)
legend('qjError')

figure(23)
subplot(3,2,k)
plot(t,qjErr(k+3+5,:),col)
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('right_arm',k);
title(name)
legend('qjError')
end

for k=1:6

% Robot legs
figure(24)
subplot(3,2,k)
plot(t,qjErr(k+13,:),col)
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('left_leg',k);
title(name)
legend('qjError')

figure(25)
subplot(3,2,k)
plot(t,qjErr(k+13+6,:),col)
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('right_leg',k);
title(name)
legend('qjError')
end

for k=1:3
    
% Robot torso
figure(26)
subplot(3,1,k)
plot(t,qjErr(k,:),col)
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('torso',k);
title(name)  
legend('qjError')
end
end

end
