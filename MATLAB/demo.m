%Demonstration of the quadrotor model and plotting
%
%needs jtrajectory3 from jlib
%https://github.com/nzjrs/jlib

m = QuadModel();
c = QuadController();
c.control_mode = QuadController.SPEED_CONTROL;

m.sim_mode.PRINT = 0;

c.lift = 1.0;
c.roll = -.2;   %speed left
c.pitch = -.2;  %speed forward

time = 0:QuadModel.DT:5; %seconds

n = length(time);
x = zeros(n);
y = zeros(n);
z = zeros(n);
pitch = zeros(n);
roll = zeros(n);
yaw = zeros(n);

i = 1;
for t=time
    c.next_state(m);
    m.next_state(QuadModel.DT);
    x(i) = m.x;
    y(i) = -m.y;
    z(i) = -m.z; %from body to world frame
    pitch(i) = m.theta;
    roll(i) = m.phi;
    yaw(i) = m.psi;
    i = i + 1;
end

figure
hold on
jtrajectory3(x,y,z,pitch,roll,yaw,1,floor(n/15),'gripen', [22 34]);
jtrajectory3(x+9,y,z,pitch,roll,yaw,1,floor(n/15),'gripen', [22 34]);

figure
hold on
jtrajectory3(x,y,z,pitch,roll,yaw,1,floor(n/15),'none', [22 34]);
jtrajectory3(x,y,z-1,pitch,roll,yaw,1,floor(n/15),'none', [22 34]);

figure
jtrajectory3(x,y,z,pitch,roll,yaw,1,floor(n/10),'mikrokopter', [22 34]);
