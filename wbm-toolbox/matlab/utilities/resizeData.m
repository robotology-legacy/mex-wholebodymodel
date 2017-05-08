function [xout,tSpan,comDes,jetsIntensities] = resizeData(xoutIn,tSpanIn,comDesIn,jetsIntensitiesIn,config)

    tSpan           = 0:config.visualiser.timeStep:tSpanIn(end);
    xout            = interp1(tSpanIn,xoutIn,tSpan); 
    jetsIntensities = interp1(tSpanIn,jetsIntensitiesIn,tSpan);
    comDes          = interp1(tSpanIn,comDesIn,tSpan); 
end