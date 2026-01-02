function [ax, h] = plot_comparison(simout,state_arr, ax, state_str)

    % Used for plotting a comparison plot for a specified state
    
    % ax = axes handles
    % h = line handles

    % Check if an axes handle was passed; if not, create one
    if nargin > 2
        hold(ax, 'on')
    else
        ax = axes; % Create a new axes if none was provided
    end
   
    if nargin < 4
        state_str = "<undefined state>";
    end

    block_time = simout.tout; 
    h(1) = plot(ax, block_time, state_arr(1,:), 'linewidth', 2);
    grid (ax, 'on')
    hold on
    for i = 2:1:size(state_arr, 1)
       h(i) = plot(ax, block_time, state_arr(i,:));
    end

    title(ax, "Comparing " + state_str)
    xlabel(ax, 'time[s]')
    ylabel(ax, state_str)

end