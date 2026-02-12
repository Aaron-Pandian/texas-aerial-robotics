function [R] = rotationMatrix(aHat,phi)
% rotationMatrix : Generates the rotation matrix R corresponding to a rotation
% through an angle phi about the axis defined by the unit
% vector aHat. This is a straightforward implementation of
% Euler’s formula for a rotation matrix.
%
% INPUTS
%
% aHat ------- 3-by-1 unit vector constituting the axis of rotation,
% synonmymous with K in the notes. 
%
% phi -------- Angle of rotation, in radians.
%
%
% OUTPUTS
%
% R ---------- 3-by-3 rotation matrix
%
%+------------------------------------------------------------------------------+
% References: None
%
%
% Author: Aaron Pandian 
%+==============================================================================+

function [uCross] = crossProductEquivalent(u)
u1 = u(1,1); 
u2 = u(2,1);
u3 = u(3,1);
uCross = [0 -u3 u2; u3 0 -u1; -u2 u1 0];
end

I = [1 0 0; 0 1 0; 0 0 1];
aHatTranspose = aHat.';
R1 = cos(phi)*I;
R2 = (1-cos(phi))*aHat*aHatTranspose;
R3 = sin(phi)*crossProductEquivalent(aHat);
R = R1+R2-R3;

end






