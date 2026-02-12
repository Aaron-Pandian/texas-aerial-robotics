function [rpGtilde,rbGtilde] = gnssMeasSimulator(S,P)
% gnssMeasSimulator : Simulates GNSS measurements for quad.
%
%
% INPUTS
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
%
% OUTPUTS
%
% rpGtilde --- 3x1 GNSS-measured position of the quad's primary GNSS antenna,
%              in ECEF coordinates relative to the reference antenna, in
%              meters.
%
% rbGtilde --- 3x1 GNSS-measured position of secondary GNSS antenna, in ECEF
%              coordinates relative to the primary antenna, in meters.
%              rbGtilde is constrained to satisfy norm(rbGtilde) = b, where b
%              is the known baseline distance between the two antennas.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

rI = S.statek.rI;
RBI = S.statek.RBI;

RpL = P.sensorParams.RpL;
sigmab = P.sensorParams.sigmab;
r0G = P.sensorParams.r0G;
ra1B = P.sensorParams.raB(:,1); % primary antenna 3x1 position in B 
ra2B = P.sensorParams.raB(:,2); % secondary antenna 3x1 position in B 

%% Primary

% Transform the inertial ECEF frame to the ENU frame, where vEnu = R*vEcef
RLG = Recef2enu(r0G);

% Find RpG
RpG = inv(RLG)*RpL*inv(RLG'); 

% Finding coordinates of primary antenna in I
rpI = rI + (RBI')*ra1B;

% Finding coordinates of primary antenna in G
rpG = (RLG')*rpI;

% Simulate noise vector
k = 1;
j = 1;
covarMatrix = RpG*eq(k,j); 
w = mvnrnd(zeros(3,1), covarMatrix)';

% Finding rpGtilde
rpGtilde = rpG + w;

%% Baseline 

% Finding coordinates of secondary antenna in I
rbI = rI + (RBI')*ra2B;

% Finding coordinates of secondary antenna in G
rbG = (RLG')*rbI;

% Find RbG
epsilon = 10^(-9);
rubG = rbG./norm(rbG);
RbG = ((norm(rbG)^2)*(sigmab^2)).*(eye(3) - (rubG*(rubG'))) + epsilon.*eye(3);

% Simulate noise vector
k = 1;
j = 1;
covarMatrix = RbG*eq(k,j); 
w = mvnrnd(zeros(3,1), covarMatrix)';

% Finding rbGtilde with constraint norm(rbGtilde) = norm(rbG)
rbGtildeNoMag = rbG + w; 
rbGtildeUnit = rbGtildeNoMag./norm(rbGtildeNoMag);
rbGtilde = norm(rbG).*rbGtildeUnit;





















