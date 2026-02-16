clc;
RestAreas = [60 80 7 20 zeros(1,124)]';
SlowStep = 0.001;

load('simparams_stamp22')

% MinPosLimit = -100;
% MaxPosLimit =100;
% TrMinPosLimit=MinPosLimit;
% TrMaxPosLimit=MaxPosLimit;

PositiveSpeedLimit = 80;  %%Daha sonrasnda end-damping set edicek
NegativeSpeedLimit = -80; %%Daha sonrasnda end-damping set edicek
MaximumAccLimit = 60;

UnbTorqueValue = 0;  
MaximumTorque = 3.17;      %% Normalde Power Limiter olsa idi dinamik set edilecekti.

%% POS PARAMETERS
PositionKp_Gmin = 1;
PositionKp_Gmax =  1;
PositionKp_K = 0.2;
PositionKp_Sat  = 6;

PositionKi_Gmin = 0;
PositionKi_Gmax =  0;
PositionKi_K = 1;
PositionKi_Sat  = 2;

PositionKi = 0;
PositionKd = 0;

PositionKp = [PositionKp_Gmin PositionKp_Gmax PositionKp_K PositionKp_Sat];
PositionKi = [PositionKi_Gmin PositionKi_Gmax PositionKi_K PositionKi_Sat];
%% SPEED PARAMETERS
SpeedKp_Gmin = 0.4;
SpeedKp_Gmax = 0.4;
SpeedKp_K =1.2;
SpeedKp_Sat  = 0.4;

SpeedKi_Gmin = 5;
SpeedKi_Gmax = 5;
SpeedKi_K = 1.25;
SpeedKi_Sat  = 15;

SpeedKd = 0.008;
SpeedKv = 0;
SpeedKp = [SpeedKp_Gmin SpeedKp_Gmax SpeedKp_K SpeedKp_Sat];
SpeedKi = [SpeedKi_Gmin SpeedKi_Gmax SpeedKi_K SpeedKi_Sat];

%%
FnotchFilterParametersSet1 = [10 0.8 0.8];
FnotchFilterParametersSet2 = [-1 1 1];

%%
EndDampingEnabled = 1;
% WZoneTraEn = 1;
% WZoneElvEn = 1;
%%Eksen2:

% MinPosLimit2 = -30;
% MaxPosLimit2= 60;

% ElMinPosLimit=MinPosLimit2;
% ElMaxPosLimit=MaxPosLimit2;

PositiveSpeedLimit2 = +80;  %%Daha sonrasnda end-damping set edicek
NegativeSpeedLimit2 = -80; %%Daha sonrasnda end-damping set edicek
MaximumAccLimit2 = 200;

UnbTorqueValue2 = 0;      %%Daha sonrasnda dinamik set edicek
MaximumTorque = 3.17;      %% Normalde Power Limiter olsa idi dinamik set edilecekti.

%% POS PARAMETERS
PositionKp_Gmin2 = 4.1;
PositionKp_Gmax2 =  10;
PositionKp_K2 = 1.2;
PositionKp_Sat2  = 11;

PositionKi_Gmin2= 0;
PositionKi_Gmax2=  0;
PositionKi_K2= 1;
PositionKi_Sat2 = 2;

PositionKi2 = 0;
PositionKd2 = 0;

PositionKp2 = [PositionKp_Gmin2  PositionKp_Gmax2  PositionKp_K2  PositionKp_Sat2];
PositionKi2 = [PositionKi_Gmin2  PositionKi_Gmax2  PositionKi_K2  PositionKi_Sat2];
%% SPEED PARAMETERS
SpeedKp_Gmin2 = 0.038;
SpeedKp_Gmax2 = 0.052;
SpeedKp_K2=1;
SpeedKp_Sat2  = 0.2;

SpeedKi_Gmin2 = 20;
SpeedKi_Gmax2 = 35;
SpeedKi_K2 = 6;
SpeedKi_Sat2  = 29;

SpeedKd2 = 0;
SpeedKv2 = 0;
SpeedKp2 = [SpeedKp_Gmin2  SpeedKp_Gmax2  SpeedKp_K2  SpeedKp_Sat2];
SpeedKi2 = [SpeedKi_Gmin2  SpeedKi_Gmax2  SpeedKi_K2  SpeedKi_Sat2];

%%
FnotchFilterParametersSet3 = [50 0.3 0.9];
FnotchFilterParametersSet4 = [-1 1 1];

%%
EndDampingEnabled2 = 1;

%%



              