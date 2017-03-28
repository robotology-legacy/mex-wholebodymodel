function [MODEL_updated,INIT_CONDITIONS_updated] = finiteStateMachine(t,MODEL,INIT_CONDITIONS)
%FINITESTATEMACHINE update and configure the robot model and initial
%                   conditions according to the current state. It also
%                   defines the conditions for switching to the next state.
%
% Format: [MODEL_updated,INIT_CONDITIONS_updated] = FINITESTATEMACHINE(t,MODEL,INIT_CONDITIONS)
%
% Inputs:  - current time t [s];
%          - MODEL is a structure defining the robot model;
%          - INIT_CONDITIONS is a structure containing initial conditions
%            for integration.
%
% Output:  - MODEL_updated updated MODEL structure;
%          - INIT_CONDITIONS_updated updated INIT_CONDITIONS structure;
%
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
global state t_switch stanceFootForce diffState; 
persistent contYogaMovements prevCoMRef prevJointRef prevGains;
% update the previous state. It is used for detecting a finite event.
previousState           = state;
% configure model and initial conditions
INIT_CONDITIONS_updated = INIT_CONDITIONS;
MODEL_updated           = MODEL;
% this will avoid the state to be updated in advance
changeStateDetector     = 1;
changeYogaDetector      = 1;
% update CoM reference
INIT_CONDITIONS_updated.xCoMRef = INIT_CONDITIONS_updated.INITFORKINEMATICS.xCoM;

%% STATE 1: Two feet balancing
if state == 1
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,1];
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % initialize smoothing values
    prevCoMRef   = INIT_CONDITIONS_updated.xCoMRef;
    prevJointRef = INIT_CONDITIONS_updated.qjRef;
    prevGains    = SM.gainsVector;
    % switching conditions
    if t >= SM.t_treshold(state)
        state     = 2;
        changeStateDetector = 0;
        if MODEL_updated.CONFIG.demoOnlyRightFoot
            state = 8;
        end
        disp(['Event detected: switching to state ', num2str(state)])    
    end
end

%% STATE 2: CoM transition to left foot
if state == 2 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseLFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
    if (t-t_switch) >= SM.t_treshold && (stanceFootForce > SM.f_treshold) 
       state    = 3;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
    end
end

%% STATE 3: Left foot balancing
if state == 3 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,0];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseLFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    MODEL_updated.constraintLinkNames   = SM.constraintLinkNames;
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
    if  (t-t_switch) >= SM.t_treshold 
        state    = 4;
        contYogaMovements = 1;
        changeStateDetector = 0;
        % in case no yoga movements are required
        if ~MODEL_updated.CONFIG.yogaMovements
            state = 5;
            disp(['Event detected: switching to state ', num2str(state)])
        else
            disp(['Event detected: switching to state ', num2str(state)])
            disp(['Current yoga movement: ', num2str(contYogaMovements)])
        end        
    end
end

%% STATE 4: Yoga movements on left foot
if state == 4 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,0];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseLFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,num2str(contYogaMovements));
    % conditions for switching to the next movement
    if contYogaMovements < 8
       if (t-t_switch) > SM.t_yoga(contYogaMovements) 
           contYogaMovements  = contYogaMovements + 1;
           disp(['Current yoga movement: ', num2str(contYogaMovements)])
           changeYogaDetector = 0;
       end
    end
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,num2str(contYogaMovements));
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
    if  (t-t_switch) > SM.t_yoga(8) && contYogaMovements == 8 && changeYogaDetector
       state    = 5;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% STATE 5: Preparing for switching
if state == 5 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,0];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseLFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
    if  (t-t_switch) >= SM.t_treshold
       state    = 6;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% STATE 6: Looking for contact
if state == 6 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,0];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseLFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
   if  (t-t_switch) >= SM.t_treshold && stanceFootForce > SM.f_treshold
       state    = 7;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% STATE 7: Two feet balancing
if state == 7 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef = INIT_CONDITIONS_updated.INITFORKINEMATICS.xCoM;
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    MODEL_updated.constraintLinkNames   = SM.constraintLinkNames;
    % switching conditions
   if  (t-t_switch) >= SM.t_treshold
       state     = 8;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% STATE 8: CoM transition to right foot
if state == 8 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseRFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
    if (t-t_switch) >= SM.t_treshold && (stanceFootForce > SM.f_treshold) 
       state    = 9;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
    end
end

%% STATE 9: Right foot balancing
if state == 9 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [0,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseRFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    MODEL_updated.constraintLinkNames = SM.constraintLinkNames;
    % switching conditions
    if  (t-t_switch) >= SM.t_treshold
        state    = 10;
        contYogaMovements   = 1;
        changeStateDetector = 0;
        % in case no yoga movements are required
        if ~MODEL_updated.CONFIG.yogaMovements
            state = 11;
            disp(['Event detected: switching to state ', num2str(state)])
        else
            disp(['Event detected: switching to state ', num2str(state)])
            disp(['Current yoga movement: ', num2str(contYogaMovements)])
        end        
    end
end

%% STATE 10: Yoga movements on right foot
if state == 10 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [0,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseRFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,num2str(contYogaMovements));
    % conditions for switching to the next movement
    if contYogaMovements < 8
       if (t-t_switch) > SM.t_yoga(contYogaMovements) 
           contYogaMovements  = contYogaMovements + 1;
           disp(['Current yoga movement: ', num2str(contYogaMovements)])
           changeYogaDetector = 0;
       end
    end
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,num2str(contYogaMovements));
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
    if  (t-t_switch) > SM.t_yoga(8) && contYogaMovements == 8 && changeYogaDetector
       state    = 11;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% STATE 11: Preparing for switching
if state == 11 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [0,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseRFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
    if  (t-t_switch) >= SM.t_treshold
       state    = 12;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% STATE 12: Looking for contact
if state == 12 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [0,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef([1,2]) = INIT_CONDITIONS_updated.INITFORKINEMATICS.poseRFoot_qt([1,2]);
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    % switching conditions
   if  (t-t_switch) >= SM.t_treshold && stanceFootForce > SM.f_treshold
       state    = 13;
       changeStateDetector = 0;
       disp(['Event detected: switching to state ', num2str(state)])
   end
end

%% STATE 13: Two feet balancing
if state == 13 && changeStateDetector
    % update robot model with the number of feet on ground
    MODEL_updated.CONFIG.feet_on_ground = [1,1];
    % update CoM reference w.r.t. stance foot position
    INIT_CONDITIONS_updated.xCoMRef = INIT_CONDITIONS_updated.INITFORKINEMATICS.xCoM;
    % load values for the current state
    SM = initStateMachine(INIT_CONDITIONS_updated,MODEL_updated,state,'');
    % update CoM, joint and gains references
    INIT_CONDITIONS_updated.xCoMRef = SM.xCoMRef;
    INIT_CONDITIONS_updated.qjRef   = SM.qjRef;
    INIT_CONDITIONS_updated.GAINS   = reshapeGains(SM.gainsVector,MODEL_updated);
    MODEL_updated.constraintLinkNames = SM.constraintLinkNames;
end

%% Smooth gains and references
if state > 1 
    % define coefficients for smoothing
    % joint smoothing coefficient
    if t < (t_switch+SM.smoothingTimeJoints)
        alpha_joints = (t-t_switch)/(SM.smoothingTimeJoints);
    else
        alpha_joints = 1;
    end
    % Com smoothing coefficient
    if t < (t_switch+SM.smoothingTimeCoM)
        alpha_CoM = (t-t_switch)/(SM.smoothingTimeCoM);
    else
        alpha_CoM = 1;
    end
    % gains smoothing coefficient
    if t < (t_switch+SM.smoothingTimeGains)
        alpha_gains = (t-t_switch)/(SM.smoothingTimeGains);
    else
        alpha_gains = 1;
    end
    %% Smooth reference trajectories and gains
    INIT_CONDITIONS_updated.xCoMRef    = (1-alpha_CoM)*prevCoMRef        + alpha_CoM*INIT_CONDITIONS_updated.xCoMRef;
    INIT_CONDITIONS_updated.qjRef      = (1-alpha_joints)*prevJointRef   + alpha_joints*INIT_CONDITIONS_updated.qjRef;
    gainsVector                        = (1-alpha_gains)*prevGains       + alpha_gains*SM.gainsVector;
    INIT_CONDITIONS_updated.gainsInit  = reshapeGains(gainsVector,MODEL_updated);
end

%% Update finite event detector
diffState = (-1)^(state)*(state-previousState-1);
% update references for smoothing 
if diffState == 0
    prevCoMRef   = INIT_CONDITIONS_updated.xCoMRef;
    prevJointRef = INIT_CONDITIONS_updated.qjRef;
    prevGains    = gainsVector;
    t_switch     = t;
end
% update joint references for yoga states
if state == 4 || state == 8
    if (t-t_switch) > SM.t_yoga(contYogaMovements) 
        prevJointRef = INIT_CONDITIONS_updated.qjRef;
        t_switch     = t;
    end
end
end
