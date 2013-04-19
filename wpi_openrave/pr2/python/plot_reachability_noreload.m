% Perform a knnsearch between x and the query points in y,
% using first Minkowski then Chebychev distance metrics.

% ----------------------------------------------
% fname = 'reachability_data/config.csv';
% config = dlmread(fname,',',1,0);
% [r,c] = size(config);
% min_max = zeros(c,2);
% 
% for i=1:c
%     min_max(i,1) = min(config(:,i));
%     min_max(i,2) = max(config(:,i));
% end
% ----------------------------------------------

scale = [1,1,0.0001,0.0001];
y = [-0.5,0,-3,0];
[n,d]=knnsearch(config,y,'k',10,'distance','seuclidean','scale',scale);

subplot(2,1,1)
hold on
scatter(config(n,1),config(n,2),30,'r')
subplot(2,1,2)
hold on
scatter(config(n,3),config(n,4),30,'r')

scale = [1,1,1,1];
y = [-0.5,0.5,-3,0];
[n,d]=knnsearch(config,y,'k',10,'distance','seuclidean','scale',scale);

subplot(2,1,1)
hold on
scatter(config(n,1),config(n,2),30,'g')
subplot(2,1,2)
hold on
scatter(config(n,3),config(n,4),30,'g')


subplot(2,1,1)
scatter(config(:,1),config(:,2),10,'b')
ylabel('y','fontsize',16); 
xlabel('x','fontsize',16);
axis([min_max(1,1) min_max(1,2) min_max(2,1) min_max(2,2)])
subplot(2,1,2)
scatter(config(:,3),config(:,4),10,'b')
ylabel('beta','fontsize',16); 
xlabel('gama','fontsize',16);
axis([min_max(3,1) min_max(3,2) min_max(4,1) min_max(4,2)])

csvwrite('reachability_data/config.csv',config); 