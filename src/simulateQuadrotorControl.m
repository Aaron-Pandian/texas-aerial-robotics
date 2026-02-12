function [Q] = simulateQuadrotorControl(R,S,P)
% simulateQuadrotorControl : Simulates closed-loop control of a quadrotor
%                            aircraft.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%        rIstar = Nx3 matrix of desired CM positions in the I frame, in
%                 meters.  rIstar(k,:)' is the 3x1 position at time tk =
%                 tVec(k).
%
%        vIstar = Nx3 matrix of desired CM velocities with respect to the I
%                 frame and expressed in the I frame, in meters/sec.
%                 vIstar(k,:)' is the 3x1 velocity at time tk = tVec(k).
%
%        aIstar = Nx3 matrix of desired CM accelerations with respect to the I
%                 frame and expressed in the I frame, in meters/sec^2.
%                 aIstar(k,:)' is the 3x1 acceleration at time tk =
%                 tVec(k).
%
%        xIstar = Nx3 matrix of desired body x-axis direction, expressed as a
%                 unit vector in the I frame. xIstar(k,:)' is the 3x1
%                 direction at time tk = tVec(k).
%  
% S ---------- Structure with the following elements:
%
%  oversampFact = Oversampling factor. Let dtIn = R.tVec(2) - R.tVec(1). Then
%                 the output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.
%
%        state0 = State of the quad at R.tVec(1) = 0, expressed as a structure
%                 with the following elements:
%                   
%                   r = 3x1 position in the world frame, in meters
% 
%                   e = 3x1 vector of Euler angles, in radians, indicating the
%                       attitude
%
%                   v = 3x1 velocity with respect to the world frame and
%                       expressed in the world frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%       distMat = (N-1)x3 matrix of disturbance forces acting on the quad's
%                 center of mass, expressed in Newtons in the world frame.
%                 distMat(k,:)' is the constant (zero-order-hold) 3x1
%                 disturbance vector acting on the quad from R.tVec(k) to
%                 R.tVec(k+1).
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%  sensorParams = Structure containing sensor parameters, as defined in
%                 sensorParamsScript.m
%
%
% OUTPUTS
%
% Q ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 Q.tVec(1) = R.tVec(1), Q.tVec(M) = R.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%  
%         state = State of the quad at times in tVec, expressed as a
%                 structure with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the I frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the I frame
%                       and expressed in the I frame, in meters per
%                       second.
%                 
%           omegaBMat = Mx3 matrix composed such that omegaBMat(k,:)' is the
%                       3x1 angular rate vector expressed in the body frame in
%                       radians, that applies at tVec(k).
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+  

N = length(R.tVec);
dtIn = R.tVec(2) - R.tVec(1);
dtOut = dtIn/S.oversampFact;
RBIk = euler2dcm(S.state0.e);

% Initial angular rates in rad/s, initialize here as per lab document
S.state0.omegaVec = [0 0 0 0]';

Xk = [S.state0.r;S.state0.v;RBIk(:);S.state0.omegaB;S.state0.omegaVec]; 
Pa.quadParams = P.quadParams;
Pa.constants = P.constants;
Pa.sensorParams = P.sensorParams;

XMat = []; tVec = [];

% Initialize Sk values
Sk.statek.RBI = euler2dcm(S.state0.e); % Initial e to RBI for function input
Sk.statek.rI = S.state0.r;
Sk.statek.vI = S.state0.v;
Sk.statek.omegaB = S.state0.omegaB;
Sk.statek.omegaVec = S.state0.omegaVec;

for kk=1:N-1
  tspan = [R.tVec(kk):dtOut:R.tVec(kk+1)]';
  distVeck = S.distMat(kk,:)';
  % Get Rk values, P structure is constant
  Rk.tVec = R.tVec(kk);
  Rk.rIstark = R.rIstar(kk,:)';
  Rk.vIstark = R.vIstar(kk,:)';
  Rk.aIstark = R.aIstar(kk,:)';
  Rk.xIstark = R.xIstar(kk,:)';

  [Fk, zIstark] = trajectoryController(Rk,Sk,P); % Where S structure input needs to be updated after initial state

  Rk.zIstark = zIstark;

  [NBk] = attitudeController(Rk,Sk,P);
  [eak] = voltageConverter(Fk,NBk,P);
  eaVeck = eak;

  [tVeck,XMatk] = ...
      ode45(@(t,X) quadOdeFunctionHF(t,X,eaVeck,distVeck,Pa),tspan,Xk); % Think through omegaVec input between two simulators

  % Update Sk values after Quadrotor Function
  XstateNow = XMatk(end,:);
  Sk.statek.rI = XstateNow(1:3)';
  Sk.statek.vI = XstateNow(4:6)';
  Sk.statek.omegaB = XstateNow(16:18)';
  Sk.statek.RBI = [XstateNow(7) XstateNow(10) XstateNow(13); % 3 x 3
                   XstateNow(8) XstateNow(11) XstateNow(14); 
                   XstateNow(9) XstateNow(12) XstateNow(15)];
  % Can also add omegaVec here but not needed since already stored in XMat
  % and not needed elsewhere, since its not part of state. 

  if(length(tspan) == 2)
    tVec = [tVec; tVeck(1)];
    XMat = [XMat; XMatk(1,:)];
  else
    tVec = [tVec; tVeck(1:end-1)];
    XMat = [XMat; XMatk(1:end-1,:)];
  end
  Xk = XMatk(end,:)';
  if(mod(kk,10) == 0)
    RBIk(:) = Xk(7:15);
    [UR,SR,VR]=svd(RBIk);
    RBIk = UR*VR'; Xk(7:15) = RBIk(:);
  end
end
XMat = [XMat;XMatk(end,:)];
tVec = [tVec;tVeck(end,:)];

M = length(tVec);
Q.tVec = tVec;
Q.state.rMat = XMat(:,1:3);
Q.state.vMat = XMat(:,4:6);
Q.state.omegaBMat = XMat(:,16:18);
% Q.state.omegaVec = XMat(:,19:22); % OmegaVec not included in state but
% recorded in QuadODE, so can optionally include in output.
Q.state.eMat = zeros(M,3);
RBI = zeros(3,3);
for mm=1:M
  RBI(:) = XMat(mm,7:15);
  Q.state.eMat(mm,:) = dcm2euler(RBI)';  
end




