function [d] = plot_reachability_data(fname)
    data = dlmread(fname,',',1,0);
    [r, c] = size(data);
    d = zeros(r,3); % Data to plot
    for s = 1:r
        d (s,1) = data(s,1);
        d(s,2) = data(s,2);
        d(s,3) = data(s,5);
    end
    % Green if reachable, black otherwise.
    scatter(d(:,1),d(:,2),15,[zeros(size(d,1),1),d(:,3),zeros(size(d,1),1)],'filled')
    