function name = whatname(name_group,k)
% WHATNAME is used to insert titles in the joints plots. It assign the
% titles according to the joints configuration in yarpRobotInterface.ini

%% Torso
a1 = strcmp(name_group,'torso');

if a1 == 1
    
    if k==1
        
        name = 'torso pitch';
    elseif k==2
        
        name = 'torso roll';
    elseif k==3
        
        name = 'torso yaw';
    end
end

%% Rigth arm
a2 = strcmp(name_group,'right_arm');

if a2 == 1
    
    if k==1
        
        name = 'r shoulder pitch';
    elseif k==2
        
        name = 'r shoulder roll';
    elseif k==3
        
        name = 'r shoulder yaw';
    elseif k==4
        
        name = 'r elbow';
    elseif k==5
        
        name = 'r wrist prosup';
    end
end

%% Left arm
a3 = strcmp(name_group,'left_arm');

if a3 == 1
    
    if k==1
        
        name = 'l shoulder pitch';
    elseif k==2
        
        name = 'l shoulder roll';
    elseif k==3
        
        name = 'l shoulder yaw';
    elseif k==4
        
        name = 'l elbow';
    elseif k==5
        
        name = 'l wrist prosup';
    end
end

%% Left leg
a4 = strcmp(name_group,'left_leg');

if a4 == 1
    
    if k==1
        
        name = 'l hip pitch';
    elseif k==2
        
        name = 'l hip roll';
    elseif k==3
        
        name = 'l hip yaw';
    elseif k==4
        
        name = 'l knee';
    elseif k==5
        
        name = 'l ankle pitch';
    elseif k==6
        
        name = 'l ankle roll';
    end
end

%% Rigth leg
a5 = strcmp(name_group,'right_leg');

if a5 == 1
    
    if k==1
        
        name = 'r hip pitch';
    elseif k==2
        
        name = 'r hip roll';
    elseif k==3
        
        name = 'r hip yaw';
    elseif k==4
        
        name = 'r knee';
    elseif k==5
        
        name = 'r ankle pitch';
    elseif k==6
        
        name = 'r ankle roll';
    end
end

%% Check the input
atot = a1+a2+a3+a4+a5;

if atot == 0
    
    error('The input does not match any known joints name')
end

end
