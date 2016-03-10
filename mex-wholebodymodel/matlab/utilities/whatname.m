function name = whatname(name_group,k)
%% whatname
%  It assigns titles to graphics according to robot's joints distribution.
%  The inputs are the robot link name (torso, r_arm, l_arm,...) and the interested 
%  degree of freedom.

%% Torso graphics
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

%% Rigth arm graphics
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
        
%% Left arm graphics
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

%% Left leg graphics
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

%% Rigth leg graphics
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


%% Name check
atot = a1+a2+a3+a4+a5;

if atot ==0
    
    error('The name required as a title for some joints-related graphics does not match any known joints name')
    
end

end
