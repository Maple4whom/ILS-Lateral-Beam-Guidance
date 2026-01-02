function [tl, ax, h] = plot_simulink_sim_states(simout, title_str_concat)

    % tl = tiled layout handle
    % ax = axes handles
    % h = line handles

    % Get the data from the SIMULINK model
    block_time = simout.tout;
    block_Y_R = simout.Y_R;
    block_p = simout.p;
    block_phi = simout.phi;
    block_psi = simout.psi;

    state_fig = figure();
    
    tl = tiledlayout(state_fig, 2,2);

    if nargin < 2 
        title_str_concat = '';
    end
    
    title(tl, 'SIMULINK Model States(' + ...
        title_str_concat + ') - Joshua Yandoc', 'Color', [1 0 0])
    Title.FontSize = 18;
    Title.FontWeight = 'bold';
    
    ax(1) = nexttile(tl);
    h(1) = plot(ax(1), block_time, block_phi, 'linewidth', 2);
    grid on
    %legend(act_limits_str(actuator))
    title('SIMULINK Simulation of bank angle, \phi')
    xlabel('time[s]')
    ylabel('bank angle \phi [deg]')
    
    ax(2) = nexttile(tl);
    h(2) = plot(ax(2), block_time, block_p, 'linewidth', 2);
    grid on
    %legend(act_limits_str(actuator))
    title('SIMULINK Simulation of roll rate, d\phi/dt')
    xlabel('time[s]')
    ylabel('roll rate d\phi/dt [deg/s]')
    
    ax(3) = nexttile(tl);
    h(3) = plot(ax(3), block_time, block_psi, 'linewidth', 2);
    grid on
    %legend(act_limits_str(actuator))
    title('SIMULINK Simulation of heading angle, \psi')
    xlabel('time[s]')
    ylabel('heading angle \psi [deg]')
    
    ax(4) = nexttile(tl);
    h(4) = plot(ax(4), block_time, block_Y_R, 'linewidth', 2);
    grid on
    %legend(act_limits_str(actuator))
    title('SIMULINK Simulation of Lateral Displacement, Y_R')
    xlabel('time[s]')
    ylabel('lateral displacement, Y_R [m]')

end