% Create a random euler angle vector
e = [0, pi/2, 0]';

% Create the resulting RBI
RBI = euler2dcm(e);

% Initializing weights 
aVec = ones(3);

%% No Noise Test
% Create random body frame unit vectors for function input
VB1 = rand(3,1); % 3X1
VB1 = VB1/norm(VB1);
VB2 = rand(3,1); 
VB2 = VB2/norm(VB2);
VB3 = rand(3,1); 
VB3 = VB3/norm(VB3);

% Calculate I frame vectors from body frame rotation using RBI
VI1 = RBI'*VB1; % 3x1
VI2 = RBI'*VB2;
VI3 = RBI'*VB3;

% Using predefined vectors to create VIMat and VBMat
VBMat = [VB1'; VB2'; VB3'];
VIMat = [VI1'; VI2'; VI3'];

% Attempt Wahba solver to get original RBI matrix
RBItest = wahbaSolver(aVec, VIMat, VBMat);

% disp(VBMat) % To show that the matrix values are changing
disp(RBItest)
disp(RBI)

%% Noise Test
% Create random body frame unit vectors for function input
VB1 = rand(3,1); % 3X1
VB1 = VB1/norm(VB1);
VB2 = rand(3,1); 
VB2 = VB2/norm(VB2);
VB3 = rand(3,1); 
VB3 = VB3/norm(VB3);

% Calculate I frame vectors from body frame rotation using RBI
VI1 = RBI'*VB1; % 3x1
VI2 = RBI'*VB2;
VI3 = RBI'*VB3;

% Add noise to the inertial frame vectors, after transformation
noiseVector1 = 0 + (0.5-0).*rand(3,1);
noiseVector2 = 0 + (0.5-0).*rand(3,1);
noiseVector3 = 0 + (0.5-0).*rand(3,1);
VI1 = VI1 + noiseVector1;
VI2 = VI2 + noiseVector2;
VI3 = VI3 + noiseVector3;

% Using predefined vectors to create VIMat and VBMat
VBMat = [VB1'; VB2'; VB3']; % NX3
VIMat = [VI1'; VI2'; VI3'];

% Attempt Wahba solver to get original RBI matrix
RBItest = wahbaSolver(aVec, VIMat, VBMat);

disp(RBItest)
disp(RBI)
