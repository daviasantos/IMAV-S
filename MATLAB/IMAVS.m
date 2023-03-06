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

pause(15)

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


oNav = ImplementCalibration( oNav, oSensors, oMav );


% State: READY

oState.event = oState.INIT_END;

oState = StateTransition( oState );


%% Discrete-time loop



while( 1 )
    
    t1 = tic;
    
    
    oJoy = jread( oJoy );
        
    
    %% State machine

    oState = EventDetection ( oState, oJoy, oMav, oNav, oGuidance );

    oState = StateTransition( oState );

    oState = StateTime      ( oState, Ts );
    

    %% End the simulation if state is off
    

    if oState.state == oState.OFF
        
        break;
    
    end
    
    
    %% Compute control commands

    
    % State: MANUAL
    
    if oState.state == oState.MANUAL 
        
        
        % Read joystick

        oJoy = jcommand( oJoy );

        
        % Commands to the flight controllers 
        
        oControl = transferJoy2Control( oControl, oJoy, oMav );
  
        
    end
        

     
    % State: WAYPOINT

    if oState.state == oState.WAYPOINT
        
        oGuidance = ImplementGuidance( oGuidance, oNav, oSensors, oMav );
        
        % Commands to the flight controllers 

        oControl = transferGui2Control( oControl, oGuidance );
        
        
    end  
    
    
    % State: TAKEOFF
    
    if oState.state == oState.TAKEOFF
         
        oControl.r_(3) = oGuidance.htakeoff;
        oControl.v_    = oGuidance.vtakeoff;
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


    oControl = transferNavSensors2Control( oControl, oNav, oSensors, oMav );
    
       
    % Implement position control, attitude control, and control allocation


    oControl = implementControl( oControl, oState );

    
    %% Environment and plant simulation


    oMav = ImplementSimulation( oMav, oState, oControl, oUncer );
    

    %% Sensor platform simulation

    
    oSensors = transferMav2Sensors( oSensors, oMav );
   
    oSensors = getSensorData( oSensors );

    

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




