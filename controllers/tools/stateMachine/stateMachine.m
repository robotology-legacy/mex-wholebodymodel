function CONFIG_updated = stateMachine(t,CONFIG)

global force_feet state com_error joint_error diffState t_switch contYoga;
import WBM.utilities.rotm2quat;
previousState  = state;
CONFIG_updated = CONFIG;

%% Tresholds
t_treshold     = CONFIG_updated.t_treshold;
f_treshold     = CONFIG_updated.f_treshold;
com_treshold   = CONFIG_updated.com_treshold;
joint_treshold = CONFIG_updated.joint_treshold;

%% Two feet balancing
if state == 1
   if t >= t_treshold(state)
       state    = 2;
       t_switch = t; 
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% CoM transition to stance leg
if state == 2
    if  (force_feet < f_treshold(state)) && (norm(com_error) < com_treshold(state))  
       state    = 3;
       t_switch = t;
       disp(['Event detected: switching to state ', num2str(state)])
    end
end

%% One foot balancing
if state == 3
   if  norm(joint_error) < joint_treshold(state)
       state = 4;
       t_switch = t;
       disp(['Event detected: switching to state ', num2str(state)])
       disp(['Current yoga phase: ', num2str(contYoga)])
   end
end

%% Yoga ++ demo
if state == 4  
    
   SM = initStateMachine(CONFIG,state,num2str(contYoga));
   alphaJoints  = (t-t_switch)/(SM.smoothingTimeJoints);
   alphaGains   = (t-t_switch)/(SM.smoothingTimeGains);
   alphaCoM     = (t-t_switch)/(SM.smoothingTimeCoM);

   CONFIG_updated.xCoMRef = (1-alphaCoM)*transpose(SM.xCoMRef_atPrevState) + alphaCoM*transpose(SM.xCoMRef) ;
   CONFIG_updated.qjRef   = (1-alphaJoints)*transpose(SM.qjRef_atPrevState) + alphaJoints*transpose(SM.qjRef);    
   gainsVector            = (1-alphaGains)*SM.gainsVector_atPrevState + alphaGains*SM.gainsVector;
   CONFIG_updated.gains   = reshapeGains(gainsVector,CONFIG);

   if  norm(joint_error) < joint_treshold(state) && (t-t_switch) > CONFIG_updated.tYoga
       contYoga = contYoga + 1;
       t_switch = t;
       disp(['Current yoga phase: ', num2str(contYoga)])
   end
   if  norm(joint_error) < joint_treshold(state) && contYoga == 8
       state = 5;
       t_switch = t;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% Looking for contact
if state == 5
   if  t_switch == 10000
       state = 6;
       t_switch = t;
   end
end

%% Smooth gains and references
if state > 1 && state ~= 4
   SM = initStateMachine(CONFIG,state,[]);

   alphaJoints  = (t-t_switch)/(SM.smoothingTimeJoints);
   alphaGains   = (t-t_switch)/(SM.smoothingTimeGains);
   alphaCoM     = (t-t_switch)/(SM.smoothingTimeCoM);

   CONFIG_updated.xCoMRef = (1-alphaCoM)*transpose(SM.xCoMRef_atPrevState) + alphaCoM*transpose(SM.xCoMRef) ;
   CONFIG_updated.qjRef   = (1-alphaJoints)*transpose(SM.qjRef_atPrevState) + alphaJoints*transpose(SM.qjRef);    
   gainsVector            = (1-alphaGains)*SM.gainsVector_atPrevState + alphaGains*SM.gainsVector;
   CONFIG_updated.gains   = reshapeGains(gainsVector,CONFIG);
end

%% Update diffState
diffState = (-1)^(state)*(state-previousState-1);

end