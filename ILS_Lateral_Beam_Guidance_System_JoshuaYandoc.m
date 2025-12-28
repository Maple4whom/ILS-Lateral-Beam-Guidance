% Joshua Yandoc 
% ILS Lateral Beam Guidance System Simulation

%% Phase 0 Part 1 - MATLAB Model

clear;
clc;
close all;
clear global;

% Initialising Everything
% initialize the parameter values as global variables (i.e. defining model
% paramters)
global B_SM g G_c J_M K_A K_D K_E K_P K_R K_T K_V L_A R_A T_A V_T R K_I

% Appendix A: Given Parameters for the Lateral Beam Guidance System
B_SM = 0.7;
g = 9.81; % m/s^2
% Original coupler gain
%G_c = 45.5;
% Reutned coupler gain for Q7
G_c = 25.5;
J_M = 0.006;
K_A = 1.2;
K_D = 0.9;
K_E = 0.9;
K_P = 52.5;
K_R = 1.2;
K_T = 1.7;
K_V = 1.3;
L_A = 0.2; % H
R_A = 10; % Ohms
T_A = 2.0; % s
% Introduction of integral term
K_I = 0.9;

%----------------------------CAN CHANGE THIS---------------------------
% Initial conditions of inputs, states, and state derivatives given 
psi_0 = deg2rad(-20); % initial heading angle given in [deg], but convert to [rad]
phi_0 = deg2rad(0); % initial bank angle given in [deg], but convert to [rad]
R_0 = 6000; % initial distance from craft to end of runway [m]
R = R_0;
Y_R0 = 150; % inital lateral distance of craft to centre line [m]
V_T = 60; % initial forward velocity [m/s]
% These initial conditions are not explicity given, but assumed
%----------------------------CAN CHANGE THIS---------------------------

% Initial conditions of parameters which aren't explicitly given

% current  - initially, aileron is in neutral position as the guidance system hasn't provided
% banking angle, roll rate, etc, needed that the actuator will then
% take to control the aileron - hence, aileron isn't moving
% initially, meaning actuator isn't operating at time t = 0^- (i.e. i_0^- = 0);
i_0 = 0; % [A]
% if the ailerons aren't operating initially, then deflection angle is 0
% deg, which also implies deflection rate is also 0
delta_a_0 = 0; %[rad]
delta_a_dot_0 = 0; %[rad/s]
% if phi_0 is 0, then roll rate, dphi/dt = p, is also 0
p_0 = deg2rad(0); % given in [deg/s], but convert to [rad/s]


% Initialize state array and state derivative array for t = 0
x = [i_0, delta_a_0, delta_a_dot_0, phi_0, p_0, psi_0, Y_R0]; % initializing state variable array
xdot = zeros(7,1); % initializing state derivative array

% Making the simulation

% Defining the simulation parameters
stepsize = 0.01;
endtime = 250;
counter = 0;
comminterval = 0.01;

% Dynamic Segment
for time = 0:stepsize:endtime
    % store the time, state, and state derivative data point at every
    % stepsize
    if rem(time, comminterval) ==0
        counter = counter + 1; % incr counter
        tout(counter) = time; % store time
        xout(:,counter) = x; % store state
        xdout(:, counter) = xdot; % store state deriv.
    end

    % Derivative Section
    xdot = Lateral_Beam_Guidance_System_State_Deriv_JoshuaYandoc(x);

    % Integral Section - Using 4th Order Runge-Kutta integration
    k1 = stepsize * Lateral_Beam_Guidance_System_State_Deriv_JoshuaYandoc(x);		% evaluate derivative k1
    k2 = stepsize * Lateral_Beam_Guidance_System_State_Deriv_JoshuaYandoc(x+(k1/2));	% evaluate derivative k2
    k3 = stepsize * Lateral_Beam_Guidance_System_State_Deriv_JoshuaYandoc(x+(k2/2));	% evaluate derivative k3
    k4 = stepsize * Lateral_Beam_Guidance_System_State_Deriv_JoshuaYandoc(x+k3);		% evaluate derivative k4
    x = x + ((k1 + 2*k2 + 2*k3 + k4)/6);		% averaged output

    
end


% convert the state variables with units of [rad] back to units of [deg]
xout(2:6, :) = rad2deg(xout(2:6, :));

figure()

tiledlayout(3,3)
title(tiledlayout(3,3), "State variables of guidance system sim(Retuned) - Joshua Yandoc", 'Color', [1 0 0]);
Title.FontSize = 18;
Title.FontWeight = 'bold';

ax11 = nexttile(1);
plot(tout, xout(1,:), 'linewidth', 2)
title('Simulation of current, i')
xlabel('time[s]')
ylabel('Current i [A]')
ylim(ax11, [-0.5 0.25])

ax12 = nexttile(2);
plot(tout, xout(2,:), 'linewidth', 2)
title('Simulation of aileron deflection angle, \delta_a')
xlabel('time[s]')
ylabel('deflection angle \delta_a [deg]')

ax13 = nexttile(3);
plot(tout, xout(3,:), 'linewidth', 2)
title('Simulation of aileron deflection angle rate, d\delta_a/dt')
xlabel('time[s]')
ylabel('def. angle rate d\delta_a/dt [deg/s]')
ylim(ax13, [-50 25])

ax21 = nexttile(4);
plot(tout, xout(4,:), 'linewidth', 2)
title('Simulation of bank angle, \phi')
xlabel('time[s]')
ylabel('bank angle \phi [deg]')

ax22 = nexttile(5);
plot(tout, xout(5,:), 'linewidth', 2)
title('Simulation of roll rate, d\phi/dt')
xlabel('time[s]')
ylabel('roll rate d\phi/dt [deg/s]')

ax23 = nexttile(6);
plot(tout, xout(6,:), 'linewidth', 2)
title('Simulation of heading angle, \psi')
xlabel('time[s]')
ylabel('heading angle \psi [deg]')

ax32 = nexttile(8);
plot(tout, xout(7,:), 'linewidth', 2)
title('Simulation of Lateral Displacement, Y_R')
xlabel('time[s]')
ylabel('lateral displacement, Y_R [m]')

%% Phase 0 Part 2 - SIMULINK Model

% Plotting the SIMULINK Model 

model = 'Lateral_Beam_Guidance_System_Block_Diagram';
% For saturated system 
USE_ACTUATOR_LIMITS = true;
% Load the SIMULINK model
load_system(model);
simout = sim(model);
% Get the data from the SIMULINK model
block_time = simout.tout;
block_Y_R = simout.Y_R;
block_p = simout.p;
block_phi = simout.phi;
block_psi = simout.psi;
% Use for plotting saturated system
block_del_a_sat = simout.del_a_sat;
block_del_a_unsat = simout.del_a_unsat;
block_del_a_rate = simout.del_a_rate;

figure()
tiledlayout(2,2)
title(tiledlayout(2,2), 'State outputs from SIMULINK Model - Joshua Yandoc', 'Color', [1 0 0])

nexttile
plot(block_time, block_phi, 'linewidth', 2)
title('SIMULINK Simulation of bank angle, \phi')
xlabel('time[s]')
ylabel('bank angle \phi [deg]')

nexttile
plot(block_time, block_p, 'linewidth', 2)
title('SIMULINK Simulation of roll rate, d\phi/dt')
xlabel('time[s]')
ylabel('roll rate d\phi/dt [deg/s]')

nexttile
plot(block_time, block_psi, 'linewidth', 2)
title('SIMULINK Simulation of heading angle, \psi')
xlabel('time[s]')
ylabel('heading angle \psi [deg]')

nexttile()
plot(block_time, block_Y_R, 'linewidth', 2)
title('SIMULINK Simulation of Lateral Displacement, Y_R')
xlabel('time[s]')
ylabel('lateral displacement, Y_R [m]')


% Saturation system comparison to ideal system
figure()
tiledlayout(2,2);

% Plotting 'ideal' deflection angle (unsaturated, no slew rate)
ax1 = nexttile([2 1]);
plot(block_time, block_del_a_unsat, 'linewidth', 2)
title('Ideal deflection angle, \delta_a (unsat + no rate limit)')
xlabel('time[s]')
ylabel('deflection angle \delta_a [deg]')

% Compare to system w sat + slew rate
ax12 = nexttile;
plot(block_time, block_del_a_sat, 'linewidth', 2)
title('Saturated deflection angle, \delta_a')
xlabel('time[s]')
ylabel('deflection angle \delta_a [deg]')

ax22 = nexttile;
plot(block_time, block_del_a_rate, 'linewidth', 2)
title('Slew rate of saturated deflection angle, d\delta_a/dt')
xlabel('time[s]')
ylabel('def. angle rate d\delta_a/dt [deg/s]')

%% After Phase 0

% create array for the amplitutde limit and rate limit for each aileron
% given in units of [deg] and [deg/s] - convert both to [rad] and [rad/s]
% Amplitude_limit = [10, 15, 20];
% Rate_limit = [5, 7.5, 10];
% colors = {'r', 'g', 'k'};
% 
% figure()
% tiledlayout(2,1)
% nexttile
% hold on
% plot(tout, xout(2,:))
% for actuator_number = 1:1:3
%     yline(Amplitude_limit(actuator_number), colors{1, actuator_number})
% end
% hold off
% ylim([-50 25])
% title('Simulation of aileron deflection angle, \delta_a')
% xlabel('time[s]')
% ylabel('deflection angle \delta_a [deg]')
% legend('\delta_a', 'actuator 1 - 10 deg', 'actuator 2 - 15 deg', 'actuator 3 - 20 deg')
% 
% nexttile
% hold on
% plot(tout, xout(3,:))
% for actuator_number = 1:1:3
%     yline(Rate_limit(actuator_number), colors{1, actuator_number})
% end
% hold off
% ylim([-5, 15])
% title('Simulation of aileron deflection angle rate, d\delta_a/dt')
% xlabel('time[s]')
% ylabel('deflection angle rate d\delta_a/dt [deg/s]')
% legend('d\delta_a/dt', 'actuator 1 - 5 deg/s', 'actuator 2 - 7.5 deg/s', 'actuator 3 - 10 deg/s')
