%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    IMAV-S - Flight dynamics and control simulator for MAVs 
%    Copyright (C) 2020  Aeronautics Institute of Technology
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program. If not, see <https://www.gnu.org/licenses/>.
%
%    Also add information on how to contact you by electronic and paper mail.
%    To contact the author, please use the electronic address davists@ita.br or 
%    send a letter to
%    
%    Prof. Dr. Davi Antonio dos Santos
%    Divisao de Engenharia Mecanica
%    Instituto Tecnologico de Aeronautica
%    Pra�a Marechal Eduardo Gomes, 50, Vila das Acacias, 12228-900, Sao Jose dos Campos,
%    SP, Brasil.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIL simulation with Unity 3D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:  Pure MATLAB simulation including: 
%
%           - plant and environment physics
%           - sensors
%           - guidance
%           - flight control laws
%           - attitude estimation (using mag, acc, gyro) and gps
%           - state machine
%           - joystick commands
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Add to path

addpath('Classes');
addpath('Conversions');


%% Clean and close

clc
clear
close all

tstart = tic;


%% Parameters

% Input parameters

Parameters;

% TCP socket with unity computer 

sSocket_unity.ip     = ip_unity;
sSocket_unity.port   = port_unity;
sSocket_unity.tout   = tout_unity;
sSocket_unity.role   = role_unity;
sSocket_unity.nele   = nele_unity;
sSocket_unity.border = byteorder_unity;

oSocket_unity    = CSocket( sSocket_unity );

% MAV 

sMav.nr   = nr;
sMav.kf   = kf;
sMav.kt   = kt;
sMav.wmax = wmax;
sMav.km   = km;
sMav.Tm   = Tm;
sMav.l    = l;
sMav.delta= delta;
sMav.m    = m;
sMav.JB   = JB;
sMav.Jr   = Jr;
sMav.g    = g;
sMav.h    = Ts;
sMav.w    = zeros(nr,1);
sMav.r    = zeros(3,1);
sMav.v    = zeros(3,1);
sMav.vp   = zeros(3,1);
sMav.D    = eye(3);
sMav.W    = zeros(3,1);


oMav = CMav( sMav );




% Disturbance/Uncertainty 


sUncer.alpha_f = alpha_f;
sUncer.alpha_t = alpha_t;
sUncer.mg = zeros(3,1);
sUncer.tint = 0;
sUncer.mi = 0;
sUncer.phi = 0;

oUncer = CUncer( sUncer );



% Sensor platform 


sSensors.ba  = ba0;
sSensors.bg  = bg0;
sSensors.bm  = bm0;
sSensors.g   = g;
sSensors.mg  = mg;   
sSensors.sa  = sa;
sSensors.sg  = sg;
sSensors.sm  = sm;
sSensors.sba = sba;
sSensors.sbg = sbg;
sSensors.sbm = sbm;
sSensors.sr  = sr;
sSensors.Ts  = Ts;
sSensors.ya  = [0;0;g];
sSensors.yg  = zeros(3,1);
sSensors.ym  = mg;
sSensors.yr  = zeros(3,1); 
sSensors.yrp = zeros(3,1);


oSensors = CSensors( sSensors );


% Joystick 

sJoy.vxmax = vxmax;
sJoy.vymax = vymax;
sJoy.vzmax = vzmax;
sJoy.wzmax = wzmax;

oJoy = CJoy( sJoy );


% Flight Control

sControl.K1      = K1;
sControl.K2      = K2;
sControl.K3      = K3;
sControl.K4      = K4;
sControl.Kc      = Kc;
sControl.JB      = JB;
sControl.Jr      = Jr;
sControl.m       = m;
sControl.g       = g;
sControl.nr      = nr;
sControl.l       = l;
sControl.delta   = delta;
sControl.kf      = kf;
sControl.kt      = kt;
sControl.k       = kt/kf;
sControl.Tmax    = Tmax;
sControl.Fmin    = Fmin;
sControl.Fmax    = Fmax;
sControl.zetamin = zetamin;
sControl.zetamax = zetamax;
sControl.r_      = zeros(3,1);
sControl.v_      = zeros(3,1);
sControl.p_      = 0;
sControl.wz_     = 0;
sControl.w_      = zeros(nr,1);
sControl.D_      = eye(3);
sControl.tau     = tau_ref_filter;
sControl.Ts      = Ts;

oControl = CControl( sControl );


% Trajectory planning/guidance 

sGuidance.nw    = nw;
sGuidance.wl    = wl;
sGuidance.Kpr   = Kpr;
sGuidance.Kpp   = Kpp;
sGuidance.Kdr   = Kdr;
sGuidance.Kdp   = Kdp;
sGuidance.rhor  = rhor;
sGuidance.rhop  = rhop;
sGuidance.dtl   = dtl;
sGuidance.Ts    = Ts;
sGuidance.dkl   = dtl/Ts;
sGuidance.l     = 1;
sGuidance.k     = 0;
sGuidance.r_    = zeros(3,1);
sGuidance.v_    = zeros(3,1);
sGuidance.p_    = 0;
sGuidance.wz_   = 0;
sGuidance.flag  = 0;


oGuidance = CGuidance( sGuidance );


% Navigation algorithm

sNavigation.tau = tau;
sNavigation.Ts  = Ts;
sNavigation.Ra  = Ra;
sNavigation.Rg  = Rg;
sNavigation.Rm  = Rm;
sNavigation.Rr  = Rr;
sNavigation.Qbg = Qbg;
sNavigation.mg  = mg;
sNavigation.x0  = x0;
sNavigation.P0  = P0;
            
oNavigation = CNavigation( sNavigation );


% Data manipulation

oData = CData( nr );


% State machine 

oState = CState;



%% Initial interface

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('IMAV-S: SIMULATION OF FLIGHT DYNAMICS AND CONTROL OF MAVS');
disp('Version 1.0');
disp('SIL simulation with Unity 3D');
disp(' ');
disp('Description: Pure MATLAB simulation including:'); 
disp(' ');
disp('             - plant and environment physics');
disp('             - sensors');
disp('             - guidance');
disp('             - flight control laws');
disp('             - attitude estimation (using mag, acc, gyro) and gps');
disp('             - state machine');
disp('             - joystick commands');
disp(' ');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('Author: Prof. Dr. Davi A. Santos (davists@ita.br)');
disp('Institution: Aeronautics Institute of Technology (ITA/Brazil)');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp(' ');


input('Press ENTER to start...');
disp(' ');


% State: INIT

oState.event = oState.TURN_ON;
oState = StateTransition( oState );


%% Connections

% Create joystick handle

oJoy = jopen( oJoy );

% Unity TCP socket setup and initialization

oSocket_unity = InitSocket( oSocket_unity );


%% Calibration


for k = 1:kfcalib 
    
    % Sensor measurements
    
    oSensors.vp = oMav.vp;
    oSensors.W  = oMav.W;
    oSensors.r  = oMav.r;
    oSensors.D  = oMav.D;
    
    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    
    
    % Navigation
    
    oNavigation.ya  = oSensors.ya;
    oNavigation.yg  = oSensors.yg;
    oNavigation.ym  = oSensors.ym;
    oNavigation.yr  = oSensors.yr;
    oNavigation.yrp = oSensors.yrp;
    
    
    oNavigation = NV( oNavigation );
    
    
end



% State: READY

oState.event = oState.INIT_END;
oState = StateTransition( oState );


%% Discrete-time loop

oJoy.flag = 0;

while( 1 )
    
    t1 = tic;
    oJoy = jread( oJoy );
    
    
    %% State machine
    
    % turn off command
    
    if oJoy.b(1) && oJoy.b(2), oState.event = oState.TURN_OFF; end
    
    % arm command
    
    if oJoy.a(2) == 1 && oJoy.a(4) == -1, oState.event = oState.ARM_CMD; end 
    
    % disarm command
    
    if oJoy.a(2) == -1 && oJoy.a(4) == 1, oState.event = oState.DISARM_CMD; end 

    % take off command
    
    if oNavigation.x(3)>(htakeoff-0.05) && oState.state == oState.TAKEOFF, oState.event = oState.TAKEOFF_END; end    
    
    if oJoy.b(3), oState.event = oState.TAKEOFF_CMD; end
     
    
    % land command
          
    if oMav.r(3) < 0.01 && oState.state == oState.LANDING, oState.event = oState.LAND_END; end
    
    if oJoy.b(4), oState.event = oState.LAND_CMD; end
    
    % waypoint command
    
    if oJoy.b(5), oState.event = oState.WAYPOINT_CMD; end
    
    % end of states (events)
    
   
    if norm(oJoy.a) > 0.1 && oState.state == oState.WAYPOINT, oState.event = oState.WAYPOINT_END; end
    
       
    oState = StateTransition( oState );
    oState = StateTime( oState, Ts );
    
    if oState.state == oState.OFF
        
        break;
    
    end
    
    
    %% Compute commands
    
    % State: MANUAL
    
    if oState.state == oState.MANUAL 
        
        oJoy = jcommand( oJoy );
        
        % Commands to the flight controllers 
        
        oControl.v_   =  [oJoy.vx,oJoy.vy,oJoy.vz]'; 
        oControl.wz_  =  oJoy.wz;
        
        %aux = oJoy.vz;
        %disp(aux)
        
        oControl = PBRefFilter ( oControl );
        
        oControl.r_   =  oControl.r_ + oControl.vcheck*Ts;
        oControl.p_   =  oControl.p_ + oControl.wzcheck*Ts;
        
        if oControl.p_ > pi/2
            
            oJoy.flag = 1; 
            oControl.wzcheck = 0;
            
        elseif oControl.p_ < -pi/2
            
            oJoy.flag = 2;
            oControl.wzcheck = 0;
            
        else
            
            oJoy.flag = 0;
        
        end
     
        
        
    end
        
     
    % State: WAYPOINT

    if oState.state == oState.WAYPOINT
        
        % input measurement

        oGuidance.r  = oNavigation.x(1:3); 
        oGuidance.v  = oNavigation.x(4:6); 
        aux          = D2a( q2D( oNavigation.x(10:13 )) );  
        oGuidance.p  = aux(3); 
        oGuidance.wz = oSensors.yg(3) - oNavigation.x(16); 
        
        
        % wayset verification

        oGuidance = wayset_verif( oGuidance );

        % wayset transition

        oGuidance = wayset_transit( oGuidance ); 

        % command generation

        oGuidance = plaw( oGuidance );

        
        % Commands to the flight controllers 
        
        oControl.r_    = oGuidance.r_;
        oControl.v_    = oGuidance.v_;
        oControl.p_    = oGuidance.p_; 
        oControl.wz_   = oGuidance.wz_;
        
    end  
    
    
    % State: TAKEOFF
    
    if oState.state == oState.TAKEOFF
         
        oControl.r_(3) = htakeoff;
        oControl.v_    = [0 0 1]';
        oControl.wz_   = 0;
        
        
    end
    
    % State: LANDING
    
    if oState.state == oState.LANDING
         
        oControl.r_(3) = 0;
        oControl.v_    = [0 0 -0.5]';
        oControl.wz_   = 0;
    
    end
    
    
    %% Flight control
    
    
    % Feedback variables 
    
    oControl.r     = oNavigation.x(1:3); 
    oControl.v     = oNavigation.x(4:6); 
    oControl.D     = q2D( oNavigation.x(10:13) );  
    oControl.W     = oSensors.yg - oNavigation.x(14:16);   
    
    
    % Position control law

    if oState.state == oState.TAKEOFF || ...
       oState.state == oState.LANDING || ...
       oState.state == oState.WAYPOINT 
        
        oControl = PC( oControl, 1 );
    
    elseif oState.state == oState.MANUAL
        
        oControl = PC( oControl, 0 );
    
    end
    
 
    % Attitude command computation
        
    
    oControl = ATC( oControl );
    
    
    % Attitude control law
    
    
    if oState.state == oState.TAKEOFF || ...
       oState.state == oState.LANDING || ...
       oState.state == oState.WAYPOINT 
   
   
        oControl = AC( oControl, 1 );
        
        
    elseif oState.state == oState.MANUAL
    
        oControl = AC( oControl, 0 );
        
    end
    
    
    % Control allocation algorithm
    
    oControl = CA( oControl );
    
    
    
    %% Environment and plant simulation
    
    % State: ARMED
    
    if oState.state == oState.ARMED
        
        oMav.r  = oMav.r;
        oMav.D  = oMav.D;
        oMav.v  = zeros(3,1);
        oMav.vp = zeros(3,1);  
        oMav.W  = zeros(3,1);
        oMav.w_ = 0.10*wmax*ones(nr,1);
       
        oMav = rotors( oMav );
     
    end
    
    
    % State: READY
    
    if oState.state == oState.READY
        
        oMav.r   = oMav.r;
        oMav.D   = oMav.D;
        oMav.v   = zeros(3,1);
        oMav.vp  = zeros(3,1);  
        oMav.W   = zeros(3,1);
        oMav.w_  = zeros(nr,1);
       
        oMav = rotors( oMav );
      
    end
    
    
    % State: TAKEOFF/LANDING/MANUAL/WAYPOINT
    
    if oState.state == oState.TAKEOFF || ...
       oState.state == oState.LANDING || ...
       oState.state == oState.MANUAL  || ...        
       oState.state == oState.WAYPOINT 
   
        % Disturbances
    
        oUncer  = disturbances( oUncer );
    
    
        % Equations of motion
    
        oMav.Fd = oUncer.Fd;
        oMav.Td = oUncer.Td;
        oMav.w_ = oControl.w_;
    
        oMav = propeller( oMav );
        oMav = efforts  ( oMav );
        oMav = dynamics ( oMav );
    
    end
    
    
    %% Sensor platform simulation
    
    oSensors.vp = oMav.vp;
    oSensors.W  = oMav.W;
    oSensors.r  = oMav.r;
    oSensors.D  = oMav.D;
    
    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    

    %% Navigation algorithm
   
    
    oNavigation.ya  = oSensors.ya;
    oNavigation.yg  = oSensors.yg;
    oNavigation.ym  = oSensors.ym;
    oNavigation.yr  = oSensors.yr;
    oNavigation.yrp = oSensors.yrp;
    
    oNavigation = NV( oNavigation );
    
    
    %% Animation
    
    % Initialize data members with true pose and power
    
    oData.r = oMav.r;
    oData.a = 180/pi*D2a( oMav.D' );
    
    if oState.state == oState.READY
        oData.power = 0; 
    else
        oData.power = 1;
    end
    
    oData.state = oState.state;
    
    
    % Pack and write data to Unity 3D
    
    oData               = PackUnity( oData );
    oSocket_unity.m_out = oData.m_out_unity;
    swrite( oSocket_unity );
   
    
    %% Waiting for sampling instant
    
    t2 = toc(t1);
    dt = uint64(1000*(Ts-t2)); 
    java.lang.Thread.sleep(dt);

    
    
    
end 


%% Simulation time

tend = toc(tstart);
disp('Simulation finished! Duration:');
disp(' ');
disp(tend);
    




