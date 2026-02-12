function [ftildeB,omegaBtilde] = imuSimulator(S,P)
% imuSimulator : Simulates IMU measurements of specific force and
%                body-referenced angular rate.
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
%                  vI = 3x1 velocity of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%
%                  aI = 3x1 acceleration of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second^2.
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%           omegaBdot = 3x1 time derivative of omegaB, in radians per
%                       second^2.
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% ftildeB ---- 3x1 specific force measured by the IMU's 3-axis accelerometer
%
% omegaBtilde  3x1 angular rate measured by the IMU's 3-axis rate gyro
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+

% Initializing input vectors
rI = S.statek.rI;
vI = S.statek.vI;
aI = S.statek.aI;
RBI = S.statek.RBI;
omegaB = S.statek.omegaB;
omegaBdot = S.statek.omegaBdot;

% Initializing sensor parameters
lB = P.sensorParams.lB;
Sa = P.sensorParams.Sa;
Qa = P.sensorParams.Qa; 
sigmaa = P.sensorParams.sigmaa; 
alphaa = P.sensorParams.alphaa; 
Qa2 = P.sensorParams.Qa2;
Sg = P.sensorParams.Sg;
Qg = P.sensorParams.Qg; 
sigmag = P.sensorParams.sigmag; 
alphag = P.sensorParams.alphag; 
Qg2 = P.sensorParams.Qg2;

% Initializing constants
g = P.constants.g;

%% Acceleration

% Find RI double dot
RIdd = aI;

% Find noise vector
k = 1;
j = 1;
covarMatrix1 = Qa*eq(k,j); 
covarMatrix2 = Qa2*eq(k,j); 
va = mvnrnd(zeros(3,1), covarMatrix1)';
va2 = mvnrnd(zeros(3,1), covarMatrix2)';

% Find accelerometer bias, update upon each call to the model
persistent ba;

if (isempty(ba))
    % Set ba’s initial value
    QbaSteadyState = Qa2/(1 - alphaa^2);
    ba0 = mvnrnd(zeros(3,1), QbaSteadyState)';
    ba = [ba, ba0];
else
    bak1 = alphaa.*ba(:,end) + va2;
    ba = [ba, bak1];
end

% Find fBtilde
e3 = [0 0 1]';
ftildeB = RBI*(RIdd + g.*e3) + ba(:,end) + va;

%% Angular Rates

% Find noise vector
k = 1;
j = 1;
covarMatrix1 = Qg*eq(k,j); 
covarMatrix2 = Qg2*eq(k,j); 
vg = mvnrnd(zeros(3,1), covarMatrix1)';
vg2 = mvnrnd(zeros(3,1), covarMatrix2)';

% Find gyroscope bias, update upon each call to the model
persistent bg;

if(isempty(bg))
    % Set bg’s initial value
    QbgSteadyState = Qg2/(1 - alphag^2);
    bg0 = mvnrnd(zeros(3,1), QbgSteadyState)';
    bg = [ba, bg0];
else
    bgk1 = alphag.*bg(:,end) + vg2;
    bg = [bg, bgk1];
end

% Find omegaBtilde
omegaBtilde = omegaB + bg(:,end) + vg;












