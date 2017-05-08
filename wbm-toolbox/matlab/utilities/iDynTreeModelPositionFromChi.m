function [trans,shape] = iDynTreeModelPositionFromChi(chi,MODEL) 
%iDynTreeModelPositionFromChi  extracts a couple (iDynTree.Transform, 
%                              iDynTree.VectorDynSize) from a chi vector. 
internalDofs  = MODEL.ndof;
pos           = iDynTree.Position(); 
pos.fromMatlab(chi(1:3));
quat          = iDynTree.Vector4();
quat.fromMatlab(chi(4:7)); 
rot           = iDynTree.Rotation(); 
rot.fromQuaternion(quat); 
trans         = iDynTree.Transform(rot,pos); 
shape         = iDynTree.VectorDynSize(internalDofs);
shape.fromMatlab(chi(8:7+internalDofs));

end