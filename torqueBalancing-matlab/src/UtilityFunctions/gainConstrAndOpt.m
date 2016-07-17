function [dLMom,dRMom,dLNull,dRNull,XMomentum,XNull,V,Vdot]   ...
          = gainConstrAndOpt(LMom,RMom,LNull,RNull,Xini,ROBOT_DOF_FOR_SIMULINK,KLMom,KOMom,KLNull,KONull,AMom,BMom,ANull,BNull)
%% config parameters
ndof       = size(ROBOT_DOF_FOR_SIMULINK,1);
omegaNull  = (ndof*(ndof-1))/2;
omegaMom   = (6*(6-1))/2;
tol        = 0.1;

% multipliers of derivatives
normTrace      = abs(trace(Xini'*Xini))+tol;
LambdaMatrMom  = diag(LMom);
LambdaMatrNull = diag(LNull);
RMomt          = transpose(RMom);
RNullt         = transpose(RNull);

%% postural gains time derivatives
% first term
A11 = -BNull';
A22 =  BNull'*RNullt*expm(LambdaMatrNull);
A33 =  BNull'*RNullt*expm(LambdaMatrNull)*RNull;
A44 = -BNull'*RNullt*expm(LambdaMatrNull)*RNull*(ANull'*ANull);
A55 =  BNull'*RNullt*expm(LambdaMatrNull)*RNull*(ANull'*ANull)*RNullt*expm(LambdaMatrNull);
A66 =  BNull'*RNullt*expm(LambdaMatrNull)*RNull*(ANull'*ANull)*RNullt*expm(LambdaMatrNull)*RNull;

B11 =  RNullt*expm(LambdaMatrNull)*RNull*(ANull'*ANull)*RNullt*expm(LambdaMatrNull)*RNull*BNull;
B22 =  RNull*(ANull'*ANull)*RNullt*expm(LambdaMatrNull)*RNull*BNull;
B33 =  (ANull'*ANull)*RNullt*expm(LambdaMatrNull)*RNull*BNull;
B44 =  RNullt*expm(LambdaMatrNull)*RNull*BNull;
B55 =  RNull*BNull;
B66 =  BNull;

% second term
A77 =  2*Xini*ANull; 
A88 = -2*Xini*ANull*RNullt*expm(LambdaMatrNull);
A99 = -2*Xini*ANull*RNullt*expm(LambdaMatrNull);

B77 =  RNullt*expm(LambdaMatrNull)*RNull*BNull;
B88 =  RNull*BNull;
B99 =  BNull;

%% momentum gains time derivatives
% first term
Aaa = -BMom';
Abb =  BMom'*RMomt*expm(LambdaMatrMom);
Acc =  BMom'*RMomt*expm(LambdaMatrMom)*RMom;
Add = -BMom'*RMomt*expm(LambdaMatrMom)*RMom*(AMom'*AMom);
Aee =  BMom'*RMomt*expm(LambdaMatrMom)*RMom*(AMom'*AMom)*RMomt*expm(LambdaMatrMom);
Aff =  BMom'*RMomt*expm(LambdaMatrMom)*RMom*(AMom'*AMom)*RMomt*expm(LambdaMatrMom)*RMom;

Baa =  RMomt*expm(LambdaMatrMom)*RMom*(AMom'*AMom)*RMomt*expm(LambdaMatrMom)*RMom*BMom;
Bbb =  RMom*(AMom'*AMom)*RMomt*expm(LambdaMatrMom)*RMom*BMom;
Bcc =  (AMom'*AMom)*RMomt*expm(LambdaMatrMom)*RMom*BMom;
Bdd =  RMomt*expm(LambdaMatrMom)*RMom*BMom;
Bee =  RMom*BMom;
Bff =  BMom;

% second term
Agg =  2*Xini*AMom; 
Ahh = -2*Xini*AMom*RMomt*expm(LambdaMatrMom);
Aii = -2*Xini*AMom*RMomt*expm(LambdaMatrMom);

Bgg = RMomt*expm(LambdaMatrMom)*RMom*BMom;
Bhh = RMom*BMom;
Bii = BMom;

%% mixed terms
% first term
A1a = -BMom';
A2b =  BMom'*RMomt*expm(LambdaMatrMom);
A3c =  BMom'*RMomt*expm(LambdaMatrMom)*RMom;
A4d = -BMom'*RMomt*expm(LambdaMatrMom)*RMom*AMom'*ANull;
A5e =  BMom'*RMomt*expm(LambdaMatrMom)*RMom*AMom'*ANull*RNullt*expm(LambdaMatrNull);
A6f =  BMom'*RMomt*expm(LambdaMatrMom)*RMom*AMom'*ANull*RNullt*expm(LambdaMatrNull)*RNull;

B1a =  RMomt*expm(LambdaMatrMom)*RMom*AMom'*ANull*RNullt*expm(LambdaMatrNull)*RNull*BNull;
B2b =  RMom*AMom'*ANull*RNullt*expm(LambdaMatrNull)*RNull*BNull;
B3c =  AMom'*ANull*RNullt*expm(LambdaMatrNull)*RNull*BNull;
B4d =  RNullt*expm(LambdaMatrNull)*RNull*BNull;
B5e =  RNull*BNull;
B6f =  BNull;

%% Gain matrices
BTildeNull = (B11*A11+B33*A33+B44*A44+B66*A66+B77*A77+B99*A99+B4d*A4d+B6f*A6f)./normTrace;
ATildeNull = (B22*A22+B55*A55+B88*A88+B5e*A5e)./normTrace;

BTildeMom  = (Baa*Aaa+Bcc*Acc+Bdd*Add+Bff*Aff+Bgg*Agg+Bii*Aii+B1a*A1a+B3c*A3c)./normTrace;
ATildeMom  = (Bbb*Abb+Bee*Aee+Bhh*Ahh+B2b*A2b)./normTrace;

skewBMom   = (BTildeMom-transpose(BTildeMom))/2;
skewBNull  = (BTildeNull-transpose(BTildeNull))/2;

%% Derivative of lambdaMom, lambdaNull
aMom       =  diag(ATildeMom);
aNull      =  diag(ATildeNull);

dLMom      = -KLMom.*aMom;
dLNull     = -KLNull.*aNull;

%% Omega for the skew-symm matrix on null space
g          = 1;
uNull      = zeros(omegaNull,1);

for i = 1:ndof
    
    for j = 1:ndof
        
        if j<i
            
        uNull(g) = skewBNull(i,j);
        g        = g+1;
        end
    end
end

% generate omega
omegaNull  = KONull.*uNull;

g          =  1;
Lomega     =  zeros(ndof);

for i = 1:ndof
    
    for j = 1:ndof
        
        if j<i
            
            Lomega(i,j)   = omegaNull(g);
            g             = g+1;           
        end
    end
end

% generate the skew-symm matrix
Uomega         = transpose(Lomega);
skewOmegaNull  = Lomega-Uomega;

% Rotation matrix derivative
kCorr          = 1;
dRNull         = RNull*skewOmegaNull +kCorr*(eye(ndof)-RNull*RNullt)*RNull;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Omega for the skew-symm matrix on momentum
g             = 1;
uMom          = zeros(omegaMom,1);

for i = 1:6
    
    for j = 1:6
        
        if j<i
            
        uMom(g) = skewBMom(i,j);
        g       = g+1;
        end
    end
end

% Generate omega
omegaMom   = KOMom.*uMom;

g          =  1;
Lomega     =  zeros(6);

for i = 1:6
    
    for j = 1:6
        
        if j<i
            
            Lomega(i,j)   = omegaMom(g);
            g             = g+1;           
        end
    end
end

% generate the skew-symm matrix
Uomega       = transpose(Lomega);
skewOmegaMom = Lomega-Uomega;

% Rotation matrix derivative
kCorr        = 1;
dRMom        = RMom*skewOmegaMom +kCorr*(eye(6)-RMom*RMomt)*RMom;

%% Optimized matrix
XMomentum      = RMomt*expm(LambdaMatrMom)*RMom;
XNull          = RNullt*expm(LambdaMatrNull)*RNull;

T              = Xini-AMom*XMomentum*BMom-ANull*XNull*BNull;
V              = trace(T'*T)./normTrace;
Vdot           = trace(ATildeMom*diag(dLMom)+BTildeMom*skewOmegaMom)+trace(ATildeNull*diag(dLNull)+BTildeNull*skewOmegaNull);

end

