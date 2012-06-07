classdef QuadModel < handle
    % Quad-rotor model with full dynamics and kinematics equations
    
    % ported to MATLAB by John Stowers, based on 
    % qrsim -- simple quad rotor simulator
    %          Arjan J.C. van Gemund
    %          Embedded Software Lab, TU Delft
    
    % disable m-lint warnings= for variable with same name as class
    % property
    %#ok<*PROP>

    properties (Constant)
        Z_AT_GND = 0.0; % default ground level z value for QR.
        DT = 0.01;%004; % default simultion time step (0.002 is ub)
    end
    
    properties
        sim_mode = struct('EULER', 0, 'PAUSE', 0,...
            'VERBOSE', 0, 'GRAVITY', 0, 'PRINT', 0);
        
        t = 0;		% simulation time 

        u = 0;		% airspeed (body axis x) 
        v = 0;		% airspeed (body axis y) 
        w = 0;		% airspeed (body axis z) 
        p = 0;		% angular rotation speed phi (body axis x) 
        q = 0;		% angular rotation speed theta (body axis y) 
        r = 0;		% angular rotation speed psi (body axis z) 
        phi = 0;	% roll angle (body axis x) 
        theta = 0;	% pitch angle (body axis y) 
        psi = 0;	% yaw angle (body axis z) 
        x = 0;		% position coordinate (earth axis x) 
        y = 0;		% position coordinate (earth axis y) 
        z = QuadModel.Z_AT_GND;	% position coordinate (earth axis z) 
        mx = 0;
        my = 0;

        % actuators 
        a1 = 0;	% rotor 1 
        a2 = 0;	% rotor 2 
        a3 = 0;	% rotor 3 
        a4 = 0;	% rotor 4 
        leds = 0;	% reset leds 
        tleds = 0;	% reset led blink time 

        % sensors 
        sp = 0;	% gyro measrement of p 
        sq = 0;	% gyro measrement of q 
        sr = 0;	% gyro measrement of r 
        sx = 0;	% acceleration (body axis x) 
        sy = 0;	% acceleration (body axis y) 
        sz = 0;	% acceleration (body axis z)        
    end
    properties (Access = private)
    end
    
    methods
        function o = QuadModel(x,y,z)
            %QUADMODEL(x,y,z)
            % Create a quadrotor model
            % INPUTS
            %   x:      starting position (earth axis x)
            %   y:      starting position (earth axis y)
            %   z:      starting position (earth axis z, i.e negative
            %           values are above ground)
            if nargin < 1, x = 0.0; end
            if nargin < 2, y = 0.0; end
            if nargin < 3, z = 0.0; end
            o.x = x; o.y = y; o.z = z;
        end
        
        function next_state(obj, DT)
            % gravity force
            if (~obj.sim_mode.GRAVITY)
                g = 10; % 9.81; 
            else
                g = 0;
            end

            % quad rotor constants (SI units)
             
            b = 1.0;
            d = 10.0; % 10*b: avoid lots of Z thrust when yawing
            m = 1.0;
            Ix = 1.0;
            Iy = 1.0;
            Iz = 2.0;
            Izx = 0.0;

            % copy from struct
             
            t = obj.t;
            a1 = obj.a1;
            a2 = obj.a2;
            a3 = obj.a3;
            a4 = obj.a4;
            leds = obj.leds;
            tleds = obj.tleds;

            u = obj.u;
            v = obj.v;
            w = obj.w;
            p = obj.p;
            q = obj.q;
            r = obj.r;
            phi = obj.phi;
            theta = obj.theta;
            psi = obj.psi;
            x = obj.x;
            y = obj.y;
            z = obj.z;

            % optimize a bit
             
            sinphi = sin(phi); sintheta = sin(theta); sinpsi = sin(psi);
            cosphi = cos(phi); costheta = cos(theta); cospsi = cos(psi);
            if (costheta == 0)
                disp('singularity in Euler angles: cos(theta) = 0\n');
                assert(0);
            end
            tantheta = sintheta / costheta;




            % first part of the quad rotor model (specific):
            % convert actuators signals to forces and moments
            % (in terms of body axes)
            %
            % NOTE: OVERLY SIMPLE FOR NOW!


            % clip rotor thrusts
             
            % if (a1 < 0) a1 = 0; 
            % if (a1 > 100) a1 = 100;
            % if (a2 < 0) a2 = 0; 
            % if (a2 > 100) a2 = 100;
            % if (a3 < 0) a3 = 0; 
            % if (a3 > 100) a2 = 100;
            % if (a4 < 0) a4 = 0; 
            % if (a4 > 100) a2 = 100;

            % compute rotor speed
             
            o1 = a1; % front, turning clockwise
            o2 = a2; % starboard, turning counter-clockwise
            o3 = a3; % aft, turning clockwise
            o4 = a4; % port, turning counter-clockwise

            % compute longitudal thrust (body axis)
             
            X = 0;

            % compute lateral thrust (body axis)
             
            Y = 0;

            % compute vertical thrust (body axis)
            % function of 4 rotors
             
            Z = - b * (o1*o1 + o2*o2 + o3*o3 + o4*o4);

            % compute roll moment (body axis)
             
            L = b * (o4*o4 - o2*o2); 

            % compute pitch moment (body axis)
             
            M = b * (o1*o1 - o3*o3); 

            % compute yaw moment (body axis)
             
            N = d * (o2*o2 + o4*o4 - o1*o1 - o3*o3); 

            % trace control data
              
            if (obj.sim_mode.VERBOSE)
                disp(sprintf('t    a1    a2    a3    a4       X    Y    Z      L    M    N\n'));
                disp(sprintf('%.1f  %.1f  %.1f  %.1f  %.1f    %.1f  %.1f  %.1f     %.1f  %.1f  %.1f\n',...
                    t,a1,a2,a3,a4,X,Y,Z,L,M,N));
            end




            % second part of the model (generic):
            % simulate a free body in space [Etkin page 104]
            % given the applied forces (X,Y,Z) and moments (L,M,N)
            % (simple Euler integration)
             

            % compute accelerations (body axes)
             
            du = (X / m) - g * sintheta - q * w + r * v;
            dv = (Y / m) + g * costheta * sinphi - r * u + p * w;
            dw = (Z / m) + g * costheta * cosphi - p * v + q * u;

            % compute angular accelerations (body axes)
            % we must also solve a system of 2 eqns for dp and dr
            % we use the solution of
            % ax + by = e
            % cx + dy = f
            % which is (e.g., Cramer's rule)
            % x = (de - bf) / (ad - bc)
            % y = (af - ce) / (ad - bc)
            % to compute dp and dr
             
            a_ = Ix; b_ = -Izx; e_ = L - q * r * (Iz - Iy) + Izx * p * q;
            c_ = -Izx; d_ = Iz; f_ = N - p * q * (Iy - Ix) - Izx * q * r;
            if (a_*d_ - b_*c_ == 0)
                disp('singularity in (p,q) computation: zero determinant\n');
                assert(0);
            end
            dp = (d_ * e_ - b_ * f_) / (a_ * d_ - b_ * c_);
            dq = (M - r * p * (Ix - Iz) + Izx * (p * p - r * r)) / Iy;
            dr = (c_ * e_ - a_ * f_) / (b_ * c_ - a_ * d_);

            % compute angular velocities (Euler angles)
             
            dphi = p + (q * sinphi + r * cosphi) * tantheta;
            dtheta = q * cosphi - r * sinphi;
            dpsi = (q * sinphi + r * cosphi) / costheta;

            % compute velocities (earth axes)
             
            dx = u * costheta * cospsi + ... 
                 v * (sinphi * sintheta * cospsi - cosphi * sinpsi) + ...
                 w * (cosphi * sintheta * cospsi + sinphi * sinpsi);
            dy = u * costheta * sinpsi + ...
                 v * (sinphi * sintheta * sinpsi + cosphi * cospsi) + ...
                 w * (cosphi * sintheta * sinpsi - sinphi * cospsi);
            dz = -u * sintheta + ...
                 v * sinphi * costheta + ...
                 w * cosphi * costheta;

            % integrate the system of equations
             
            u = u + du * DT; v = v + dv * DT; w = w + dw * DT;
            p = p + dp * DT; q = q + dq * DT; r = r + dr * DT;
            phi = phi + dphi * DT; theta = theta + dtheta * DT; psi = psi + dpsi * DT;
            x = x + dx * DT; y = y + dy * DT; z = z + dz * DT;


            % don't go through the ground
             
            if (z > QuadModel.Z_AT_GND)
                z = QuadModel.Z_AT_GND;
                w = 0; % avoid integration windup
            end




            if (obj.sim_mode.VERBOSE)
                disp(sprintf('t   u    v    w       p    q    r       phi  thet psi     x    y    z\n'));
                disp(sprintf('%.1f  %.1f  %.1f  %.1f     %.1f  %.1f  %.1f     %.1f  %.1f  %.1f     %.1f  %.1f  %.1f\n',...
                    t,u,v,w,p,q,r,phi,theta,psi,x,y,z));
            end
            if (obj.sim_mode.PAUSE)
                pause(); 
                obj.sim_mode.PAUSE = ~obj.sim_mode.PAUSE;
            end

            % update simulation time
             
            t = t + DT;

            % update blinking led[0]
            %if (t - tleds > 0.5) {
            %    leds = (leds ^ 0x0001);
            %    tleds = t;
            %}

            % write back to struct
             
            obj.t = t;
            obj.u = u;
            obj.v = v;
            obj.w = w;
            obj.p = p;
            obj.q = q;
            obj.r = r;
            obj.phi = phi;
            obj.theta = theta;
            obj.psi = psi;
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.leds = leds;
            obj.tleds = tleds;
        end
        
        function update_euler(obj, p, q, r)
            % update earth orientation (Euler angles) from body
            % angle increments [Etkin page 104]
            %
            % angle increments in terms of body frame (-> xformation)
 
            phi = obj.phi;
            theta = obj.theta;
            psi = obj.psi;

            if (obj.sim_mode.EULER)
                d_phi = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
                d_theta = q * cos(phi) - r * sin(phi);
                d_psi = (q * sin(phi) + r * cos(phi)) / cos(theta);
            else
                % ABSOLUTE EULER
                d_phi = p;
                d_theta = q;
                d_psi = r;
            end

            obj.phi = obj.phi + d_phi;
            obj.theta = obj.theta + d_theta;
            obj.psi = obj.psi + d_psi;
        end
    end
    
    methods (Access = private)
    end
    
end
