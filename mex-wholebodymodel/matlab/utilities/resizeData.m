%% Utility for visualizeForwardDynamics.m
function [xout,tSpan,comDes,jetsIntensities] = resizeData(varargin)

    xoutIn          = varargin{1};
    tSpanIn         = varargin{2};
    comDesIn        = varargin{3};
    CONFIG          = varargin{4};
    
    tSpan           = 0:CONFIG.visualiser.timeStep:tSpanIn(end);
    xout            = interp1(tSpanIn,xoutIn,tSpan);
    comDes          = interp1(tSpanIn,comDesIn,tSpan); 
    
    if nargin == 5
    jetsIntensitiesIn = varargin{5};
    jetsIntensities   = interp1(tSpanIn,jetsIntensitiesIn,tSpan);
    else
    jetsIntensities   = 0;
    end

end