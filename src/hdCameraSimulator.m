function [rx] = hdCameraSimulator(rXI,S,P)
% hdCameraSimulator : Simulates feature location measurements from the
%                     quad's high-definition camera. 
%
%
% INPUTS
%
% rXI -------- 3x1 location of a feature point expressed in I in meters.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
% OUTPUTS
%
% rx --------- 2x1 measured position of the feature point projection on the
%              camera's image plane, in pixels.  If the feature point is not
%              visible to the camera (the ray from the feature to the camera
%              center never intersects the image plane, or the feature is
%              behind the camera), then rx is an empty matrix.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

rI = S.statek.rI;
RBI = S.statek.RBI;

rocB = P.sensorParams.rocB;
RCB = P.sensorParams.RCB;
Rc = P.sensorParams.Rc;
pixelSize = P.sensorParams.pixelSize;
K = P.sensorParams.K;
imagePlaneSize = P.sensorParams.imagePlaneSize;

X = [rXI;1];

% Solving for RCI
RCI = RCB * RBI;

% Solving for t
t = -RCI*(rI + ((RBI')*rocB));

% Solving for projection matrix PMat
zeroMatrix = [0 0 0];
K = [K,zeroMatrix'];
PMat = K*[RCI t; zeroMatrix 1];

% Solving for x using PX
xh = PMat*X;

% If feature is not in camera image plane, rx = []
rx = [];
if xh(3) <= 0 
    return % If x(3) is negative, the feature is behind the camera (z-plane), end script here
end

% Finding xc
x = xh(1)/xh(3); 
y = xh(2)/xh(3);
xc = [x y]';

% Finding noise vector wc
k = 1;
j = 1;
covarMatrix = Rc*eq(k,j); 
w = mvnrnd(zeros(2,1), covarMatrix)';

% Finding xctilde
xctilde = (1/pixelSize).*xc + w;

% Check if xctilde is inside the camera detection plane
imagePlaneX = imagePlaneSize(1)/2;
imagePlaneY = imagePlaneSize(2)/2;
featureCameraX = abs(xctilde(1));
featureCameraY = abs(xctilde(2));

if featureCameraX <= imagePlaneX && featureCameraY <= imagePlaneY
    rx = xctilde;
else 
    rx = [];
end





