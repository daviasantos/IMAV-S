%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    IMAV-M - Flight dynamics and control simulator for MAVs 
%    Copyright (C) 2020  Aeronautics Institute of Technology
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CGuidance
% Description: guidance class. It implements a guidance algorithm,
% computing the position and heading commands on a straight-line constant-
% speed trajectory towards the next wayset.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


classdef CGuidance
    
    properties
         
        % parameters
        
        Ts             % sampling time
        wl             % position-heading waypoints
        nw             % number of waypoints
        Kpr            % proportional gain of guindance law of r
        Kpp            % || ... of psi
        Kdr            % derivative gains
        Kdp            % || 
        rhor           % ball radius for rl
        rhop           % interval radius for psil
        dtl            % dwell in l 
        dkl            % discrete dwell in l
        htakeoff       % altitude command for the auto takeoff
        vtakeoff       % velocity command for the auto takeoff
        navdata        % 1 if navdata feedback and 0 if true state feedback
        
        % variables
        
        l              % wayset index
        flag           % flag for wayset arrival
        r              % position
        v              % velocity
        p              % heading
        wz             % heading rate
        r_             % position command
        p_             % heading command
        v_             % velocity command
        wz_            % heading rate command
        k              % discrete time counter
           
    end
    
    
    methods
        
        %% Constructor
        
        function obj = CGuidance ( sGuidance )
        
            % Initialization
            
            obj.wl   = sGuidance.wl;
            obj.Kpr  = sGuidance.Kpr;
            obj.Kpp  = sGuidance.Kpp;
            obj.Kdr  = sGuidance.Kdr;
            obj.Kdp  = sGuidance.Kdp;
            obj.rhor = sGuidance.rhor;
            obj.rhop = sGuidance.rhop;
            obj.dtl  = sGuidance.dtl;
            obj.Ts   = sGuidance.Ts;
            obj.dkl  = sGuidance.dkl;
            obj.nw   = sGuidance.nw;
            obj.l    = sGuidance.l;
            obj.k    = sGuidance.k;
            obj.r_   = sGuidance.r_;
            obj.v_   = sGuidance.v_;
            obj.p_   = sGuidance.p_;
            obj.wz_  = sGuidance.wz_;
            obj.flag = sGuidance.flag;

            obj.htakeoff = sGuidance.htakeoff;
            obj.vtakeoff = sGuidance.vtakeoff;

            obj.navdata  = sGuidance.navdata;

            % Pre-computation (so far, no one!)
            
            

            
        end
        
        
        %% Generate the commands - proportional guidance law
        
        function obj = plaw( obj )
        
            % store previous values
            
            v_pre  = obj.v_;
            wz_pre = obj.wz_;
            r_pre  = obj.r_;
            p_pre  = obj.p_;
            
            % proportional law
            
            
            obj.r_ = obj.r + obj.Kpr*( obj.wl(1:3,obj.l) - obj.r );
            obj.p_ = obj.p + obj.Kpp*( obj.wl(4,obj.l) - obj.p );
            
           

            % command derivatives
            
            obj.v_  = (obj.r_ - r_pre)/obj.Ts;
            if norm(obj.v_) > 10
                obj.v_ = v_pre;
            end
            
            obj.wz_ = (obj.p_ - p_pre)/obj.Ts;
            if abs(obj.wz_) > 10
                obj.wz_ = wz_pre;
            end
            
                
        end
        
        
        
        %% Generate the commands - proportional-derivative guidance law
        
        function obj = pdlaw( obj )
        
            % store previous values
            
            obj.v_  = obj.r_;
            obj.wz_ = obj.p_;
            
            % proportional law
            
            obj.r_ = -obj.Kpr*obj.r + ( obj.Kpr + eye(3) )*obj.wl(1:3,obj.l) - obj.Kdr*obj.v;
            obj.p_ = -obj.Kpp*obj.p + ( obj.Kpp + 1 )*obj.wl(4,obj.l) - obj.Kdp*obj.wz;
            
           
            
            % command derivatives
            
            obj.v_  = (obj.r_ - obj.v_)/obj.Ts;
            obj.wz_ = (obj.p_ - obj.wz_)/obj.Ts;
            
            
                
        end
        
        
        
        
        %% Wayset verification
        
        function obj = wayset_verif( obj )
            
            if norm( obj.wl(1:3,obj.l)-obj.r ) < obj.rhor && ...
                  abs( obj.wl(4,obj.l)-obj.p ) < obj.rhop && ~obj.flag
              
                if obj.l < obj.nw
                    obj.flag = 1;         
                    obj.k = 0;
                else 
                    obj.flag = 2;         % end of trajectory
                end  
                
            end
            
        end
        
        
        
        
        %% Wayset evolution
        
        function obj = wayset_transit( obj )
        
            if obj.flag == 1
                
                if obj.k == obj.dkl(obj.l)
                    obj.l = obj.l + 1;
                    obj.flag = 0;
                else
                    obj.k = obj.k + 1;
                end
                
            end
     
        end


        function obj = transferNav2Guidance( obj, oNav, oSensors, oMav )
        
             if obj.navdata == 1

                 obj.r  = oNav.x(1:3); 
                 obj.v  = oNav.x(4:6); 
                 aux    = D2a( q2D( oNav.x(10:13 )) );  
                 obj.p  = aux(3); 
                 obj.wz = oSensors.yg(3) - oNav.x(16); 
        
             elseif obj.navdata == 0

                 obj.r  = oMav.r; 
                 obj.v  = oMav.v; 
                 aux    = D2a( oMav.D );  
                 obj.p  = aux(3); 
                 obj.wz = oMav.W(3);

             end



        end


        function obj = ImplementGuidance( obj, oNav, oSensors, oMav )

            % input measurement

            obj = transferNav2Guidance( obj, oNav, oSensors, oMav );  
            
            % wayset verification
    
            obj = wayset_verif( obj );
    
            % wayset transition
    
            obj = wayset_transit( obj ); 
    
            % command generation
    
            obj = plaw( obj );
        

        end


    end
    
    
end

    