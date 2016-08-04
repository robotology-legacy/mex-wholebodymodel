function figureCont = visualizeLinearizAndGains(t,CONFIG,ddqjNonLin,ddqjLin)
%VISUALIZELINEARIZANDGAINS visualizes the results on the linearized joint space
%                          dynamics of robot iCub.
%
%   figureCont = VISUALIZELINEARIZANDGAINS(t,CONFIG,ddqjNonLin,ddqjLin)
%   takes as input the integration time T, the structure CONFIG containing all
%   the utility parameters, the joint linear and nonlinear accelerations.
%   It generates all the plots related to the
%   stability analysis, it verifies the soundness of the linearization
%   procedure and visualizes the gains matrices after optimization. The
%   output is a counter for the automatic correction of figures numbers in
%   case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
figureCont = CONFIG.figureCont;

%% Linearized joint dynamics
if CONFIG.linearizationDebug  == 1
    
    for k=1:5
        
        % LEFT ARM
        figure(figureCont)
        subplot(3,2,k)
        plot(t,ddqjLin(k+3,:))
        hold on
        plot(t,ddqjNonLin(k+3,:),'r')
        grid on
        xlabel('Time [s]')
        ylabel('Joint Acc [rad/s^2]')
        name = whatname('left_arm',k);
        title(name)
        legend('Lin Acc','NonLin Acc')
        
        % RIGHT ARM
        figure(figureCont+1)
        subplot(3,2,k)
        plot(t,ddqjLin(k+3+5,:))
        hold on
        plot(t,ddqjNonLin(k+3+5,:),'r')
        grid on
        xlabel('Time [s]')
        ylabel('Joint Acc [rad/s^2]')
        name = whatname('right_arm',k);
        title(name)
        legend('Lin Acc','NonLin Acc')
    end
    
    figureCont = figureCont +2;
    
    for k=1:6
        
        % LEFT LEG
        figure(figureCont)
        subplot(3,2,k)
        plot(t,ddqjLin(k+13,:))
        hold on
        plot(t,ddqjNonLin(k+13,:),'r')
        grid on
        xlabel('Time [s]')
        ylabel('Joint Acc [rad/s^2]')
        name = whatname('left_leg',k);
        title(name)
        legend('Lin Acc','NonLin Acc')
        
        % RIGHT LEG
        figure(figureCont+1)
        subplot(3,2,k)
        plot(t,ddqjLin(k+13+6,:))
        hold on
        plot(t,ddqjNonLin(k+13+6,:),'r')
        grid on
        xlabel('Time [s]')
        ylabel('Joint Acc [rad/s^2]')
        name = whatname('right_leg',k);
        title(name)
        legend('Lin Acc','NonLin Acc')
    end
    
    figureCont  = figureCont +2;
    
    for k=1:3
        
        % TORSO
        figure(figureCont)
        subplot(3,1,k)
        plot(t,ddqjLin(k,:))
        hold on
        plot(t,ddqjNonLin(k,:),'r')
        grid on
        xlabel('Time [s]')
        ylabel('Joint Acc [rad/s^2]')
        name = whatname('torso',k);
        title(name)
        legend('Lin Acc','NonLin Acc')
    end
    
    figureCont = figureCont +1;
end

%% Gains Tuning results
if CONFIG.gains_tuning == 1
    
    ACartesian            = CONFIG.linearization.ACartesian;
    BCartesian            = CONFIG.linearization.BCartesian;
    ANull                 = CONFIG.linearization.ANull;
    BNull                 = CONFIG.linearization.BNull;
    KSo                   = ACartesian*CONFIG.gain.intMomentumGains*BCartesian + ANull*CONFIG.gain.impedances*BNull;
    KDo                   = ACartesian*CONFIG.gain.MomentumGains*BCartesian + ANull*CONFIG.gain.dampings*BNull;
    KSdes                 = CONFIG.gainsInit.KSdes;
    KDdes                 = CONFIG.gainsInit.KDdes;
    KSn                   = CONFIG.linearization.KS;
    KDn                   = CONFIG.linearization.KD;
    
    figure(figureCont)
    image(KSo,'CDataMapping','scaled')
    colorbar
    title('Optimized KS')
    figureCont = figureCont +1;
    
    figure(figureCont)
    image(KDo,'CDataMapping','scaled')
    colorbar
    title('Optimized KD')
    figureCont = figureCont +1;
    
    figure(figureCont)
    image(KSn,'CDataMapping','scaled')
    colorbar
    title('Kronecker KS')
    figureCont = figureCont +1;
    
    figure(figureCont)
    image(KDn,'CDataMapping','scaled')
    colorbar
    title('Kronecker KD')
    figureCont = figureCont +1;
    
    % state matrix verification
    ndof       = CONFIG.ndof;
    AStateKron = [zeros(ndof), eye(ndof); -KSn   -KDn];
    AStateDes  = [zeros(ndof), eye(ndof); -KSdes -KDdes];
    AStateOpt  = [zeros(ndof), eye(ndof); -KSo   -KDo];
    
    figure(figureCont)
    plot(real(eig(AStateKron)),imag(eig(AStateKron)),'or')
    hold on
    grid on
    plot(real(eig(AStateDes)),imag(eig(AStateDes)),'ok')
    plot(real(eig(AStateOpt)),imag(eig(AStateOpt)),'ob')
    title('Joint space eigenvalues')
    figureCont = figureCont +1;
    
end
