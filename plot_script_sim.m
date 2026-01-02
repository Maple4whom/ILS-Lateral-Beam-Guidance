function plot_script_sim(script_time, script_states)

    % Take the script_states array and plot each individual state
    
    xout = script_states;
    tout = script_time;
    
    figure()
    
    tiledlayout(3,3)
    title(tiledlayout(3,3), "State variables of guidance system sim" + ...
        "(Retuned) - Joshua Yandoc", 'Color', [1 0 0]);
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

end