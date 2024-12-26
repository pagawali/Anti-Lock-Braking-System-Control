    % Huristic antilock braking algorithm test.
clear
% Select the road profile: Dry or Wet
choice = menu('Road Profile', 'Dry', 'Wet');

% Set the final time
Tf = 5;
% Set the sample time
dt = 0.001;
% Generate a time vector for plotting
t = 0:dt:Tf;
% Initialize torque
Torque=zeros(1,length(t));
Torque(1) = -800;
% Initialize the slip ratio
lambda = zeros(1,length(t));
% Initialize the traction coefficient
mu = zeros(1,length(t));
% Define the mu-lambda curves:
mudryp=[10*(0:0.001:0.07) 3*(0.001:0.001:0.05)+0.7 (0.001:0.001:0.03)+0.85 -0.25*(0.001:0.001:0.85)+0.88];
mudry=[mudryp(length(mudryp):-1:2) mudryp];
muwetp=[7*(0:0.001:0.09) 2*(0.001:0.001:0.06)+0.63 (0.001:0.001:0.06)+0.75 0.81*ones(1,40) -0.4*(0.001:0.001:0.75)+0.81];
muwet=[muwetp(length(mudryp):-1:2) muwetp];
% Variable to display braking time.
flag = 0;
if choice == 1
    mup = mudryp;
else
    mup = muwetp;
end
% Perform the closed-loop simulation
% Set the state vector and initialize
% The state is vehicle speed and wheel speed (both in wheel speed units)
x=zeros(2,length(t));
x(:,1)=[100.0 ; 100.0];
%Define the simulation parameters:
m=1400;             % Vehicle mass
rho=1.202;          % Air density
Cd=0.5;             % Drag coefficient
A=1.95;             % Vehicle area
g=9.81;             % Graviational constant
Theta=0.0;          % Road gradient
bw=0.0;             % Wheel rotary friction coefficient
f=0.01;             % Rolling resistance coefficient
uw=0.0;             % Wind speed
fw=0.0;             % Wheel coulumb friction
Iw=0.65;            % Wheel moment of inertia
rw=0.31;            % Wheel radius
Fz=3560.0;          % Downward force on each wheel
% For braking: Ie=0, Nw=4; and for accel: Ie = 0.429, Nw=2.
rg=9.5285;          % Gear ratio
Ie=0.0;             % Inertia of the engine (engine disengaged when braking)
Nw=4;               % Number of wheels
Jw = Iw +(rg^2)*Ie; % Equivalent wheel moment of inertia when driving engine

% Perform the simulation loop
for i=1:(length(t)-1)
     % Limit the wheel speed and the vehicle speed to be non-negative
    if x(1,i) <= 0.0
        x(1,i)=0.0;
    end
    if x(2,i) <= 0.0
        x(2,i)=0.0;
    end
    % Define the wheel slip lambda:
    lambda(i)=(x(1,i)-x(2,i))/max(x(1,i),1e-3);
    % Limit the wheel slip
    if lambda(i) > 1
        lambda(i) = 1;
    elseif lambda(i) < -1
        lambda(i) = -1;
    end
    % Calculate the coefficient mu
    % Limit lambda in this calculation due to limits on array
    if lambda(i) > 1000
        lambda(i) = 1000;
    elseif lambda(i) < 0
        lambda(i) = 0;
    end
    mu(i) = mup(round(lambda(i)*1000)+1);
    % Calculate the derivative of the state
    xdot=[(-(0.5*rho*Cd*A)*(uw+rw*x(1,i))^2-Nw*Fz*mu(i)-f*m*g*cos(Theta)-m*g*sin(Theta))/(m*rw)
                             (fw*Fz-bw*x(2,i)+Fz*rw*mu(i)+Torque(i))/Jw                           ];
    x(:,i+1) = x(:,i) + dt*xdot;
    RuleBasedAntilockBrakeController
    % Display the time to stop.
    if (x(1,i) < 5 && flag == 0)
        if choice == 1
            disp(['Time to stop on dry road = ' num2str(i*dt)])
        else
            disp(['Time to stop on wet road = ' num2str(i*dt)])
        end
    end
    % Turn off antilock brakes
    if x(1,i) < 5
        Torque(i+1) = 0;
        flag = 1;
    end
end

% Add final values for plotting.
mu(length(t)) = mu(length(t)-1);
lambda(length(t)) = lambda(length(t)-1);

% Plot the results:
figure(1)
clf
subplot(311), plot(t,x(1,:),t,x(2,:));
title('Vehicle Simulation with ABS');
xlabel('Time (sec)');
ylabel('Rotational velocity');
legend('Vehicle', 'Wheel');
grid
subplot(312), plot(t,lambda,'r');
xlabel('Time (sec)');
ylabel('Lambda');
%ylim([-.25 0])
grid
subplot(313), plot(t,Torque, 'r');
xlabel('Time(sec)');
ylabel('Torque (N-m)');
grid

figure(2)
plot(t,mu)
xlabel('Time(sec)');
ylabel('Adhesion Coefficient (\mu)');

% Plot the mu-lambda curves to confirm mmu)lambda) is correct.
figure(3)
clf
plot(-0.001*(length(mudryp)-1):0.001:0.001*(length(mudryp)-1),mudry,-0.001*(length(muwetp)-1):0.001:0.001*(length(muwetp)-1), muwet)
