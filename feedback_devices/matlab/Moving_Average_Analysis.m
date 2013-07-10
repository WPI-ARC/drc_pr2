%% Moving (2) Average

figure(2);

subplot(2,1,1)
title('Moving (2) Average');

data_move = zeros(size(data_raw));

for i=1:size(data_move,2),

    data_move(:, i) = tsmovavg(data_raw(:,i), 's', 2, 1);

end

D = linspace(1,size(data_move,1),size(data_move,1));

plot(D, data_move);

% ===========================================================

subplot(2,1,2);
title('Derivative Data');
hold all

for i = 2:size(data_move,1),
    data_d(i-1,:) = data_move(i, :) - data_move(i-1, :);
end

C = linspace(1,size(data_d,1),size(data_d,1));

plot(C, data_d);

%% Moving (5) Average

figure(3);

subplot(2,1,1)
title('Moving (5) Average');

data_move = zeros(size(data_raw));

for i=1:size(data_move,2),

    data_move(:, i) = tsmovavg(data_raw(:,i), 's', 5, 1);

end

D = linspace(1,size(data_move,1),size(data_move,1));

plot(D, data_move);

% ===========================================================

subplot(2,1,2);
title('Derivative Data');
hold all

for i = 2:size(data_move,1),
    data_d(i-1,:) = data_move(i, :) - data_move(i-1, :);
end

C = linspace(1,size(data_d,1),size(data_d,1));

plot(C, data_d);

%% Moving (10) Average

figure(4);

subplot(2,1,1)
title('Moving (10) Average');

data_move = zeros(size(data_raw));

for i=1:size(data_move,2),

    data_move(:, i) = tsmovavg(data_raw(:,i), 's', 10, 1);

end

D = linspace(1,size(data_move,1),size(data_move,1));

plot(D, data_move);

% ===========================================================

subplot(2,1,2);
title('Derivative Data');
hold all

for i = 2:size(data_move,1),
    data_d(i-1,:) = data_move(i, :) - data_move(i-1, :);
end

C = linspace(1,size(data_d,1),size(data_d,1));

plot(C, data_d);

%% Moving (20) Average

figure(5);

subplot(2,1,1)
title('Moving (20) Average');

data_move = zeros(size(data_raw));

for i=1:size(data_move,2),

    data_move(:, i) = tsmovavg(data_raw(:,i), 's', 20, 1);

end

D = linspace(1,size(data_move,1),size(data_move,1));

plot(D, data_move);

% ===========================================================

subplot(2,1,2);
title('Derivative Data');
hold all

for i = 2:size(data_move,1),
    data_d(i-1,:) = data_move(i, :) - data_move(i-1, :);
end

C = linspace(1,size(data_d,1),size(data_d,1));

plot(C, data_d);