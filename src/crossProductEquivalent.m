function [uCross] = crossProductEquivalent(u)
% crossProductEquivalent : Outputs the cross-product-equivalent matrix uCross
% such that for arbitrary 3-by-1 vectors u and v,
% cross(u,v) = uCross*v.
%
% INPUTS
%
% u ---------- 3-by-1 vector
%
%
% OUTPUTS
%
% uCross ----- 3-by-3 skew-symmetric cross-product equivalent matrix
%
%+------------------------------------------------------------------------------+
% References: None
%
%
% Author: Aaron Pandian
%+==============================================================================+

u1 = u(1,1); % extracted values from row 1, column 1
u2 = u(2,1);
u3 = u(3,1);
uCross = [0 -u3 u2; u3 0 -u1; -u2 u1 0];

end







