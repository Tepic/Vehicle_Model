clear all
close all
clc

% load('wheel_speeds.mat');

% SIMULATION PARAMETERS
g = 9.80037;                % [m/s^2]
SIM_TIME = 3.5;              % [s]
dt = 1e-4;                  % [s]

% PI CONTROLER PARAMETERS
Kp = 5;
Ti = 0.08;
interrupt = 1e-3;           % [s]

% VEHICLE PARAMETERS
L = 1600*1e-3;               % [mm]
T_front = 1250*1e-3;         % [mm]
T_rear = 1200*1e-3;          % [mm]
CG_height = 340*1e-3;        % [mm]
a = 830*1e-3;                % [mm]
b = 770*1e-3;                % [mm]

unsprung_mass = 70;
sprung_mass = 174;
driver_mass = 75;
mass = sprung_mass+...
       unsprung_mass+...
       driver_mass;            % [kg]
m1_s = mass*(b/L)/2;         % [kg]
m2_s = mass*(b/L)/2;         % [kg]
m3_s = mass*(a/L)/2;         % [kg]
m4_s = mass*(a/L)/2;         % [kg]
max_steer = 135;             % [deg]
max_steerSpeed = 540;        % [deg/s]
max_Dsteer = max_steerSpeed*dt;

% ENGINE PARAMETERS
max_speed = 135;
accel_max = 10*dt;          % [m/s^2 * dt]
speed_max = 137.8;          % [kmh]
brake_max = 20*dt;          % [m/s^2 * dt] 

% TYRE PARAMETERS
load('tire_Avon.mat');

sliped = -tire_Avon(:,2);
lateralLoaded = tire_Avon(:,3).*1e3;

slipAngle = -12:1e-3:12;
lateralLoad = interp1(sliped,lateralLoaded,slipAngle,'pchip');
C_alpha = (2.7778/2)*1e-03; % [N/deg]
WHEEL_DIAMETER = 0.33/2;

beta_FL = 0;
beta_FR = 0;
beta_RL = 0;
beta_RR = 0;

%% USER DEFINED TEST
speed = 57;
steer = 0;
psi = 0;

[orientationVector timeVector] = generateOrientation(0,0,180,3.5,dt,'ramp');
[orientationVector timeVector] = ...
        generateOrientation(orientationVector,timeVector,180,0.5,dt,'step');

m1 = m1_s; m2 = m2_s; m3 = m3_s; m4 = m4_s;

vehicle_Params = [L T_front T_rear CG_height a b mass m1_s m2_s m3_s m4_s psi];
sym_Params = [steer(1) speed max_steer max_speed C_alpha g dt];
tire_Params = [beta_FL beta_FR beta_RL beta_RR];
wheel_location = [-a,0];
     
% INIT START LOCATION  

x_CG(1) = 0;
y_CG(1) = 0;

x_CC(1) = -b*cos(psi(1));
y_CC(1) = -b*sin(psi(1));

x_FR(1) = x_CG(1)+sqrt(a^2+(T_front/2)^2)*cos(psi(1)-atan((T_front/2)/a));
y_FR(1) = y_CG(1)+sqrt(a^2+(T_front/2)^2)*sin(psi(1)-atan((T_front/2)/a));

x_RR(1) = x_CG(1)-b*cos(psi(1))+(T_rear/2)*sin(psi(1));
y_RR(1) = y_CG(1)-b*sin(psi(1))-(T_rear/2)*cos(psi(1));

x_FL(1) = x_FR(1)-T_front*sin(psi(1));
y_FL(1) = y_FR(1)+T_front*cos(psi(1));

x_RL(1) = x_RR(1)-T_rear*sin(psi(1));
y_RL(1) = y_RR(1)+T_rear*cos(psi(1));

d_psi = 0;      sigma_CG = 0;

m1 = m1_s;      m2 = m2_s;
m3 = m3_s;      m4 = m4_s;

dm_longitudinal = [0; 0];
dm_lateral = [0; 0];

FL_speed = 0;
FR_speed = 0;
RL_speed = 0;
RR_speed = 0;
wheel_speeds = [FL_speed' FR_speed' RL_speed' RR_speed'];

acp_CG = 0;  acp_psi = 0;  

time = 0;
e = [];
u = [];
lastState = 0;
for position = 1:length(timeVector)
    if((mod(timeVector(position),interrupt))==0)
        e(position) = round((orientationVector(position)-psi(position)*180/pi)*1e4)/1e4;
        u(position) = Kp*e(position)+Kp*dt*e(position)/Ti;

        if(position>1)
            if(u(position)>0 && u(position-1)>0 &&...
               u(position)-u(position-1)>max_Dsteer)

                u(position) = u(position-1)+max_Dsteer;
            else if(u(position)>0 && u(position-1)>0 &&...
                    u(position)-u(position-1)<-max_Dsteer)
                      u(position) = u(position-1)-max_Dsteer;
                 end
            end
            if(u(position)<0 && u(position-1)<0 &&...
               u(position)-u(position-1)<-max_Dsteer)

                u(position) = u(position-1)-max_Dsteer;
            else if(u(position)<0 && u(position-1)<0 &&...
                    u(position)-u(position-1)>max_Dsteer)
                      u(position) = u(position-1)+max_Dsteer;
                 end
            end

            if(u(position)>0 && u(position-1)<0 &&...
               u(position)-u(position-1)>max_Dsteer)

                u(position) = u(position-1)+max_Dsteer;
            else if(u(position)<0 && u(position-1)>0 &&...
                    u(position)-u(position-1)<-max_Dsteer)
                      u(position) = u(position-1)-max_Dsteer;
                 end
            end
        else
            if(u(position)>max_Dsteer)
                u(position) = max_Dsteer;
            else if(u(position)<-max_Dsteer)
                    u(position) = -max_Dsteer;
                 end
            end
        end

        if(u(position)>135)
            u(position) = 135;
        else if(u(position)<-135)
                u(position) = -135;
             end
        end
    else
        e(position) = e(position-1);
        u(position) = Kp*e(position)+Kp*dt*e(position)/Ti;
    end
    steer = [steer; u(position)];
    
    vehicle_Params = [L T_front T_rear CG_height a b mass m1(position) m2(position) m3(position) m4(position) psi(position)];
    sym_Params = [steer(position) speed(1) max_steer max_speed C_alpha g dt];
    tire_Params = [beta_FL(position) beta_FR(position) beta_RL(position) beta_RR(position)];
    wheel_location = [x_CG(position) y_CG(position)...
                      x_FR(position) y_FR(position)...
                      x_RR(position) y_RR(position)...  
                      x_FL(position) y_FL(position)...
                      x_RL(position) y_RL(position)];
    [output lastState] = wheel_move(wheel_speeds,vehicle_Params,sym_Params,tire_Params,wheel_location,slipAngle,lateralLoad,lastState); 
    
    x_FL = [x_FL; output(2,1)];     y_FL = [y_FL; output(2,7)];
    x_FR = [x_FR; output(2,2)];     y_FR = [y_FR; output(2,8)];
    x_RL = [x_RL; output(2,3)];     y_RL = [y_RL; output(2,9)];
    x_RR = [x_RR; output(2,4)];     y_RR = [y_RR; output(2,10)];
    x_CG = [x_CG; output(2,5)];     y_CG = [y_CG; output(2,11)];
    x_CC = [x_CC; output(2,6)];     y_CC = [y_CC; output(2,12)];

    psi = [psi; output(2,13)];     d_psi = [d_psi; output(2,14)];

    beta_FL = [beta_FL; output(2,15)];
    beta_FR = [beta_FR; output(2,16)]; 
    beta_RL = [beta_RL; output(2,17)]; 
    beta_RR = [beta_RR; output(2,18)];

    sigma_CG = [sigma_CG; output(2,19)]; %time = [time; output(2,20)];

    m1 = [m1; output(2,21)];      m2 = [m2; output(2,22)];
    m3 = [m3; output(2,23)];      m4 = [m4; output(2,24)];
    dm_longitudinal = [dm_longitudinal output(:,27)];
    dm_lateral = [dm_lateral -output(:,28)];
    
    FL_speed = [FL_speed output(2,29)/WHEEL_DIAMETER];
    FR_speed = [FR_speed output(2,30)/WHEEL_DIAMETER];
    RL_speed = [RL_speed output(2,31)/WHEEL_DIAMETER];
    RR_speed = [RR_speed output(2,32)/WHEEL_DIAMETER];
    wheel_speeds = [FL_speed(end) FR_speed(end) RL_speed(end) RR_speed(end)];

    acp_CG = [acp_CG; output(2,25)];  acp_psi = [acp_psi; output(2,26)];
    time = [time time(end)+dt];
end

wheel_speeds = [FL_speed' FR_speed' RL_speed' RR_speed'];

x = [x_FL x_FR x_RL x_RR x_CG x_CC];
y = [y_FL y_FR y_RL y_RR y_CG y_CC];

m = [m1 m2 m3 m4 mass.*ones(length(time),1) dm_lateral' dm_longitudinal']; % size = length(time) x (4+1+2+2)
beta = [beta_FL beta_FR beta_RL beta_RR];

vehicle_Params = [L T_front T_rear CG_height a b g speed dt];
tire_Params = [C_alpha WHEEL_DIAMETER];

orientationVector = [orientationVector; orientationVector(end)];
timeVector = [timeVector; timeVector(end)];

data = [x y m beta wheel_speeds psi d_psi acp_psi acp_CG sigma_CG steer time' orientationVector timeVector];
disp(['Total time for ISO-standard test: ', num2str(max(time)),'s.']);
disp(['Distance: ', num2str(max(x_CG)),'m.']);
disp(['Max lateral G: ', num2str(max(acp_CG)/g),'G.']);

plotData(data,vehicle_Params,tire_Params,'etc');
