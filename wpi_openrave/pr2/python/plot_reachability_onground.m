%fname = 'reachability_data/2013-02-07_17:34:01_config_finder.csv';
%fname = 'reachability_data/2013-02-07_19:06:52_config_finder.csv';
%fname = 'reachability_data/2013-02-07_19:42:38_config_finder.csv';
fname = 'reachability_data/2013-02-07_19:54:16_config_finder.csv';

data = dlmread(fname,',',1,0);
[r, c] = size(data);
d = zeros(r,3); % Data to plot
% for s = 1:r
%     d (s,1) = data(s,1);
%     d(s,2) = data(s,2);
%     d(s,3) = data(s,5);
% end
% % Green if reachable, black otherwise.
% scatter(d(:,1),d(:,2),15,[zeros(size(d,1),1),d(:,3),zeros(size(d,1),1)],'filled')

min_max = zeros(c,2);

for i=1:c
    min_max(i,1) = min(data(:,i));
    min_max(i,2) = max(data(:,i));
end

% Get the valid configurations
valid_config = zeros(r,c-1);
k = 1;

for i=1:r
    if data(i,5) == 1
        valid_config(k,1:c-1) = data(i,1:c-1);
        k = k +1;
    end
end

config = valid_config(1:k,1:c-1);

% subplot(2,1,1)
% scatter(data(:,1),data(:,2),5,data(:,5))
% ylabel('y','fontsize',16); 
% xlabel('x','fontsize',16); 
% subplot(2,1,2)
% scatter(data(:,3),data(:,4),5,data(:,5))
% ylabel('gamma','fontsize',16); 
% xlabel('beta','fontsize',16);

% figure
% subplot(2,1,1)
% scatter(config(:,1),config(:,2))
% ylabel('y','fontsize',16); 
% xlabel('x','fontsize',16);
% axis([min_max(1,1) min_max(1,2) min_max(2,1) min_max(2,2)])
% subplot(2,1,2)
% scatter(config(:,3),config(:,4))
% ylabel('gamma','fontsize',16); 
% xlabel('beta','fontsize',16);
% axis([min_max(3,1) min_max(3,2) min_max(4,1) min_max(4,2)])



