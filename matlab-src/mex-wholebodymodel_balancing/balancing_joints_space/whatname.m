function name = whatname(name_group,k)

% Torso names
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

%% Rigth arm names
a2 = strcmp(name_group,'r_arm');

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
        
%% Left arm names
a3 = strcmp(name_group,'l_arm');

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

%% Left leg names
a4 = strcmp(name_group,'l_leg');

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

%% Rigth leg names
a5 = strcmp(name_group,'r_leg');

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


%% Check
atot = a1+a2+a3+a4+a5;

if atot ==0
    
    error('name mismatch')
    
end


end
