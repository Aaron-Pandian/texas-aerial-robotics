function [RBI] = wahbaSolver(aVec,vIMat,vBMat)
% wahbaSolver : Solves Wahba's problem via SVD.  In other words, this
%               function finds the rotation matrix RBI that minimizes the
%               cost Jw:
%
%                     N
%    Jw(RBI) = (1/2) sum ai*||viB - RBI*viI||^2
%                    i=1
%
%
% INPUTS
%
% aVec ------- Nx1 vector of least-squares weights.  aVec(i) is the weight
%              corresponding to the ith pair of vectors 
%
% vIMat ------ Nx3 matrix of 3x1 unit vectors expressed in the I frame.
%              vIMat(i,:)' is the ith 3x1 vector.
%
% vBMat ------ Nx3 matrix of 3x1 unit vectors expressed in the B
%              frame. vBMat(i,:)' is the ith 3x1 vector, which corresponds to
%              vIMat(i,:)';
%
% OUTPUTS
% 
% RBI -------- 3x3 direction cosine matrix indicating the attitude of the
%              B frame relative to the I frame.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

% Find B
B = zeros(3);
for i=1:length(aVec)
    a = aVec(i);
    V = vIMat(i,:)';
    U = vBMat(i,:)';
    B = B + a.*U*V';
end

% Find U and V from B = USV' given B
[U,S,V] = svd(B);

% Initilize M 
M = [1 0 0; 0 1 0; 0 0 det(U)*det(V)];

% Solve for 
RBI = U*M*V';






