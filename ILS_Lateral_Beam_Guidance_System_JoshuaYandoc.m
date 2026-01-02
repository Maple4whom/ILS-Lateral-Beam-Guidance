% Joshua Yandoc 
% ILS Lateral Beam Guidance System Simulation

%% Phase 0 Part 1 - MATLAB Model

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
% Plot the states
plot_script_sim(tout, xout)


%% Phase 0 Part 2/Phase 1 part 1 - SIMULINK Model


% init model str to load later
model = 'Lateral_Beam_Guidance_System_Block_Diagram';

% For use with actuator limits; -1 indicates 'ideal', unconstrained case
USE_ACTUATOR_CASE = [-1 1 2 3];
% for use in plot legend
act_limits_str =... 
    ["unconstrained amplitude and rate limit", ...
    "amp limit: 10" + char(176) + ", rate limit: 5" + char(176) + "/s",...
    "amp limit: 15" + char(176) + ", rate limit: 7.5" + char(176) + "/s",...
    "amp limit: 20" + char(176) + ", rate limit: 10" + char(176) + "/s"];
% for use in plot title
names = ["unconstrained case", "actuator 1", "actuator 2", "actuator 3"];

% Load the SIMULINK model if not already loaded
if ~bdIsLoaded(model), load_system(model); end

% Go through each actuator case
for actuator = 1:numel(USE_ACTUATOR_CASE)

    USE_ACTUATOR = USE_ACTUATOR_CASE(actuator);

    % Run the model and compare against all actuator limits
    simout = sim(model);
    
    % Use for plotting saturated system
    block_del_a_rate = simout.del_a_rate * (180/pi); % Convert to deg
    block_del_a_sat = simout.del_a * (180/pi); % Convert to deg
    if actuator == 1
        block_del_a_ideal = simout.del_a_ideal * (180/pi); % Convert to deg
    end

    % Plot some states from simulink sim 
    [tl, ax, h] = plot_simulink_sim_states(simout, names(actuator));
    % make legend for each tiled plot
    for j = 1:4
        legend(ax(j), h(j), act_limits_str(actuator));
    end
    
    
    % Saturation system comparison to ideal system
    sat_fig = figure();
    t1 = tiledlayout(sat_fig, 2,2);

    title(t1, 'SIMULINK Model deflection angle(' + ...
        names(actuator) + ') - Joshua Yandoc', 'Color', [1 0 0])
    Title.FontSize = 18;
    Title.FontWeight = 'bold';
    
    % Plotting 'ideal' deflection angle (unsaturated, no slew rate)
    ax1 = nexttile(t1, [2 1]);
    plot(ax1, block_time, block_del_a_ideal, 'linewidth', 2)
    grid on
    title('Ideal(unconstrained) deflection angle, \delta_a')
    xlabel('time[s]')
    ylabel('deflection angle \delta_a [deg]')
    
    % Compare to system w actuator limits
    ax12 = nexttile(t1);
    plot(ax12, block_time, block_del_a_sat, 'linewidth', 2)
    grid on
    legend(act_limits_str(actuator))
    title('Saturated deflection angle, \delta_a')
    xlabel('time[s]')
    ylabel('deflection angle \delta_a [deg]')
    
    ax22 = nexttile(t1);
    plot(ax22, block_time, block_del_a_rate, 'linewidth', 2)
    grid on
    legend(act_limits_str(actuator))
    title('Slew rate of saturated deflection angle, d\delta_a/dt')
    xlabel('time[s]')
    ylabel('def. angle rate d\delta_a/dt [deg/s]')

    % Store current states for current actuator for the purposes of
    % plotting a comparison plot after
    block_del_a_sat_AllCases(actuator, :) = block_del_a_sat';
    block_phi_AllCases(actuator, :) = simout.phi';
    block_p_AllCases(actuator, :) = simout.p';

end


compare_fig = figure();
t2 = tiledlayout(compare_fig, 2,2);
title(t2, 'Model comparisons, \delta_a - Joshua Yandoc', 'Color', [1 0 0])
Title.FontSize = 18;
Title.FontWeight = 'bold';

ax1 = nexttile(t2, [2 1]);
plot(ax1, block_time, block_del_a_ideal, 'linewidth', 2)
grid on
hold on
for i = 2:1:4
    plot(ax1, block_time, block_del_a_sat_AllCases(i,:))
end
title("Comparing deflection angle, \delta_a")
legend(act_limits_str)
xlabel('time[s]')
ylabel('deflection angle \delta_a [deg]')

ax12 = nexttile(t2);
plot(ax12, block_time, block_phi_AllCases(1,:), 'linewidth', 2)
grid on
hold on
for i = 2:1:4
    plot(ax12, block_time, block_phi_AllCases(i,:))
end
title("Comparing bank angle, \phi")
legend(act_limits_str)
xlabel('time[s]')
ylabel('bank angle \phi [deg]')

ax22 = nexttile(t2);
plot(ax22, block_time, block_p_AllCases(1,:), 'linewidth', 2)
grid on
hold on
for i = 2:1:4
    plot(ax22, block_time, block_p_AllCases(i,:))
end
title("Comparing roll rate, d\phi/dt")
legend(act_limits_str)
xlabel('time[s]')
ylabel('roll rate d\phi/dt [deg/s]')
