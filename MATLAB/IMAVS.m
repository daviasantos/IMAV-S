%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    IMAV-S - Flight dynamics and control simulator for MAVs 
%    Copyright (C) 2020  Aeronautics Institute of Technology
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

addpath('Classes\');
addpath('Conversions\');
addpath('Functions\');

%% Clean and close

clc
clear
close all


%% Sampling the initial simulation time

tstart = tic;


%% Call the parameters

Parameters;


%% Create the objects

oSocket_unity   = CSocket( sSocket_unity );
oMav            = CMav( sMav );
oUncer          = CUncer( sUncer );
oSensors        = CSensors( sSensors );
oJoy            = CJoy( sJoy );
oControl        = CControl( sControl );
oGuidance       = CGuidance( sGuidance );
oNav            = CNav( sNavigation );
oData           = CData( nr );
oState          = CState;



%% Initial interface and game

startthegame;

pause(10)

displayinit;






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
    

    oSensors = transferMav2Sensors( oSensors, oMav );
 
    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    
    
    % Navigation

    oNav = transferSensors2Nav( oNav, oSensors );   

    oNav = NV( oNav );
    
    
end


% State: READY

oState.event = oState.INIT_END;
oState = StateTransition( oState );


%% Discrete-time loop



while( 1 )
    
    t1 = tic;
    
    oJoy = jread( oJoy );
    
    

    
    %% State machine
    
    % turn off command
    
    if oJoy.a(2) == 1 && oJoy.a(5) == 1, oState.event = oState.TURN_OFF; end
    
    % arm command
    
    if oJoy.a(2) == 1 && oJoy.a(5) == -1, oState.event = oState.ARM_CMD; end 
    
    % disarm command
    
    if oJoy.a(2) == -1 && oJoy.a(5) == 1, oState.event = oState.DISARM_CMD; end 

    % take off command
    
    if oNav.x(3)>(htakeoff-0.05) && oState.state == oState.TAKEOFF, oState.event = oState.TAKEOFF_END; end    
    
    if oJoy.b(4), oState.event = oState.TAKEOFF_CMD; end
     
    
    % land command
          
    if oMav.r(3) < 0.01 && oState.state == oState.LANDING, oState.event = oState.LAND_END; end
    
    if oJoy.b(1), oState.event = oState.LAND_CMD; end
    
    % waypoint command
    
    if oJoy.b(3), oState.event = oState.WAYPOINT_CMD; end
    
    % end of states (events)
    
   
    if norm(oJoy.a) > 0.1 && oState.state == oState.WAYPOINT, oState.event = oState.WAYPOINT_END; end
    
       
    oState = StateTransition( oState );
    oState = StateTime( oState, Ts );
    

    %% End the simulation if state is off
    

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
        
       
        oControl = PBRefFilter ( oControl );
        
        oControl.r_   =  oControl.r_ + oControl.vcheck*Ts;
        oControl.p_   =  oControl.p_ + oControl.wzcheck*Ts;
        
       
        
    end
        
     
    % State: WAYPOINT

    if oState.state == oState.WAYPOINT
        
        % input measurement

        oGuidance = transferNav2Guidance( oGuidance, oNav, oSensors ); 

      
        
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
    
    oControl.r     = oNav.x(1:3); 
    oControl.v     = oNav.x(4:6); 
    oControl.D     = q2D( oNav.x(10:13) );  
    oControl.W     = oSensors.yg - oNav.x(14:16);   
    
    
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

    
    oSensors = transferMav2Sensors( oSensors, oMav );
   

    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    

    %% Navigation algorithm
   
    oNav = transferSensors2Nav( oNav, oSensors );
    
    oNav = NV( oNav );
    
    
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

displayend;

endthegame;




