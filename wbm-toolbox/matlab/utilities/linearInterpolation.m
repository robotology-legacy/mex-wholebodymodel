function yInterp = linearInterpolation(tAfter,tInterp,tBefore,yAfter,yBefore)
%LINEARINTERPOLATION linear interpolation of a vector or a scalar given its 
%                    values before and after the current instant. 
%
% Format: yInterp = LINEARINTERPOLATION(tAfter,tInterp,tBefore,yAfter,yBefore)
%
% Inputs:  - tAfter   time after the current instant;
%          - tInterp  current instant;
%          - tBefore  time before the current instant;
%          - yAfter   value of a scalar or vector function at tAfter;
%          - yBefore  value of a scalar or vector function at tBefore;
%
% Output:  - yInterp  value of a scalar or vector function at tInterp;
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% define the coefficient m 
m       = (yAfter - yBefore)/(tAfter - tBefore); 
% define the coefficient q
q       = yAfter - m*tAfter;
% compute y = mx + q
yInterp = m*tInterp + q;

end
