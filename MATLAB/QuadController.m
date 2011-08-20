classdef QuadController < handle
    % Quadrotor controller, P and cascading P controllers for
    % R,P,Y and speed in U,W

    % ported to MATLAB by John Stowers, based on 
    % qrsim -- simple quad rotor simulator
    %          Arjan J.C. van Gemund
    %          Embedded Software Lab, TU Delft

    properties (Constant)
        NO_CONTROL = 0;
        LIFT_CONTROL = 1;
        LIFT_CONTROL_CP = 2;
        YAW_RATE_CONTROL = 3;
        ROLL_PITCH_CONTROL = 4;
        ROLL_PITCH_CONTROL_CP = 5;
        SPEED_CONTROL = 6;
    end
    
    properties
        lift = 0;   % altitude / thrust setpoint
        yaw = 0;    % yaw setpoint
        pitch = 0;  % pitch setpoint
        roll = 0;   % roll setpoint
        control_mode = QuadController.NO_CONTROL;
    end
    
    properties (Access = private)
    end
    
    methods
        function o = QuadController()
        end
        
        function next_state(obj, quadobj)
            % select control scenario
             
            a_lift = 0; a_roll = 0; a_pitch = 0; a_yaw = 0; % safe default
            switch obj.control_mode

                case QuadController.NO_CONTROL % no control 
                     
                    a_lift = obj.lift;
                    a_roll = obj.roll;
                    a_pitch = obj.pitch;
                    a_yaw = obj.yaw;

                case QuadController.LIFT_CONTROL % lift control using z 
                     % 1 P ctl only -> oscillation (z = int(int(lift))
                     % so need cascaded P control (or PD control)
                     
                    sp_z = QuadModel.Z_AT_GND - obj.lift; % setpoint altitude
                    a_lift = 10 * (sp_z - quadobj.z); 
                    a_lift = - a_lift; % pos lift -> neg z
                    a_roll = obj.roll;
                    a_pitch = obj.pitch;
                    a_yaw = obj.yaw;

                case QuadController.LIFT_CONTROL_CP % lift control using z 
                     % now with cascaded P control 
                     
                    sp_z = QuadModel.Z_AT_GND - obj.lift; % setpoint altitude
                    sp_w = 10 * (sp_z - quadobj.z); 
                    a_lift = 10 * (sp_w - quadobj.w); 
                    a_lift = - a_lift; % pos lift -> neg z
                    a_roll = obj.roll;
                    a_pitch = obj.pitch;
                    a_yaw = obj.yaw;

                case QuadController.YAW_RATE_CONTROL % add yaw control 
                     % yaw uses rate setpoint so 1 P controller 
                     
                    sp_z = QuadModel.Z_AT_GND - obj.lift; 
                    sp_w = 10 * (sp_z - quadobj.z); 
                    a_lift = 20 * (sp_w - quadobj.w); 
                    a_lift = - a_lift; % pos lift -> neg z
                    a_roll = obj.roll;
                    a_pitch = obj.pitch;
                    sp_r = 5 * obj.yaw; % setpoint is angular rate
                    a_yaw = 5 * (sp_r - quadobj.r);

                case QuadController.ROLL_PITCH_CONTROL % add roll & pitch control
                     % 1 P ctl so oscillation
                     
                    sp_z = QuadModel.Z_AT_GND - obj.lift; 
                    sp_w = 10 * (sp_z - quadobj.z); 
                    a_lift = 20 * (sp_w - quadobj.w); 
                    a_lift = - a_lift; % pos lift -> neg z
                    sp_phi = obj.roll; % setpoint is angle
                    a_roll = 10 * (sp_phi - quadobj.phi);
                    sp_theta = obj.pitch;
                    a_pitch = 10 * (sp_theta - quadobj.theta);
                    sp_r = 5 * obj.yaw; 
                    a_yaw = 5 * (sp_r - quadobj.r);

                case QuadController.ROLL_PITCH_CONTROL_CP % add roll & pitch control
                     % now with cascaded P ctl 
                     
                    sp_z = QuadModel.Z_AT_GND - obj.lift; 
                    sp_w = 10 * (sp_z - quadobj.z); 
                    a_lift = 20 * (sp_w - quadobj.w); 
                    a_lift = - a_lift; % pos lift -> neg z
                    sp_phi = obj.roll; 
                    sp_p = 10 * (sp_phi - quadobj.phi);
                    a_roll = 10 * (sp_p - quadobj.p);
                    sp_theta = obj.pitch;
                    sp_q = 10 * (sp_theta - quadobj.theta);
                    a_pitch = 10 * (sp_q - quadobj.q);
                    sp_r = 2 * obj.yaw; 
                    a_yaw = 5 * (sp_r - quadobj.r);

                case QuadController.SPEED_CONTROL % add u/v speed control on top of roll/pitch control
                     % Note: QR should have speed sensors for this ..
                     
                    sp_z = QuadModel.Z_AT_GND - obj.lift; 
                    sp_w = 10 * (sp_z - quadobj.z); 
                    a_lift = 20 * (sp_w - quadobj.w); 
                    a_lift = - a_lift; % pos lift -> neg z
                    sp_v = 10 * obj.roll; 
                    sp_phi = 0.1 * (sp_v - quadobj.v);
                    sp_p = 10 * (sp_phi - quadobj.phi);
                    a_roll = 10 * (sp_p - quadobj.p);
                    sp_u = - 10 * obj.pitch; % nose down is pos u
                    sp_theta = - 0.1 * (sp_u - quadobj.u); % idem
                    sp_q = 10 * (sp_theta - quadobj.theta);
                    a_pitch = 10 * (sp_q - quadobj.q);
                    sp_r = 5 * obj.yaw; 
                    a_yaw = 5 * (sp_r - quadobj.r);

            end

            % we only want positive lift so clip lift
             
            if (a_lift < 0), a_lift = 0; end

            % map lift, roll, pitch, yaw to rotor actuator vars ai
            % so we need to solve for ai:
            %
            % b * (o1*o1 + o2*o2 + o3*o3 + o4*o4) = lift;
            % b * (- o2*o2 + o4*o4) = roll;
            % b * (o1*o1 - o3*o3) = pitch;
            % d * (- o1*o1 + o2*o2 - o3*o3 + o4*o4) = yaw;
            %
            % let ooi be oi*oi. then we must solve
            %
            % [  1  1  1  1  ] [ oo1 ]   [lift/b]
            % [  0 -1  0  1  ] [ oo2 ]   [roll/b]
            %                          = 
            % [ -1  0  1  0  ] [ oo3 ]   [pitch/b]
            % [ -1  1 -1  1  ] [ oo4 ]   [yaw/d]
            %
            % the inverse matrix is
            %
            % [  1  0  2 -1  ]
            % [  1 -2  0  1  ]
            %                 % 1/4
            % [  1  0 -2 -1  ]
            % [  1  2  0  1  ]
            %
            % so with b = d = 1 we have
             
            oo1 = (a_lift + 2 * a_pitch - a_yaw) / 4;
            oo2 = (a_lift - 2 * a_roll + a_yaw) / 4;
            oo3 = (a_lift - 2 * a_pitch - a_yaw) / 4;
            oo4 = (a_lift + 2 * a_roll + a_yaw) / 4;

            % clip ooi as rotors only provide prositive thrust
             
            if (oo1 < 0), oo1 = 0; end
            if (oo2 < 0), oo2 = 0; end
            if (oo3 < 0), oo3 = 0; end
            if (oo4 < 0), oo4 = 0; end

            % with ai = oi it follows
             
            a1 = sqrt(oo1);
            a2 = sqrt(oo2);
            a3 = sqrt(oo3);
            a4 = sqrt(oo4);

            % print controller and quad rotor state
             
            if (quadobj.sim_mode.PRINT)
                disp(sprintf('%.1f ', quadobj.t));
                disp(sprintf('%d ', obj.control_mode));
                if (obj.control_mode == QuadController.NO_CONTROL)
                    disp(sprintf('a1 %.1f  a2 %.1f  a3 %.1f  a4 %.1f',...
                    a1,a2,a3,a4));
                else
                    disp(sprintf('ul %.1f  l %.1f  r %.1f  p %.1f  y %.1f',...
                    obj.lift,a_lift,a_roll,a_pitch,a_yaw));
                end
                disp(sprintf('  '));
                disp(sprintf('z %.2f  psi %.2f  the %.2f  phi %.2f',...
                    quadobj.z - QuadModel.Z_AT_GND,quadobj.psi,...
                    quadobj.theta,quadobj.phi));
                disp(sprintf('  '));
                disp(sprintf('u %.2f  v %.2f  w %.2f',...
                    quadobj.u,quadobj.v,quadobj.w));
                disp(sprintf('\n'));
            end

            % connect controller output to quad rotor actuators
             
            quadobj.a1 = a1;
            quadobj.a2 = a2;
            quadobj.a3 = a3;
            quadobj.a4 = a4;
        end
    end
    
    methods (Access = private)
    end
    
end
