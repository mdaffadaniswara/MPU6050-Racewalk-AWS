x2 = [];

x1 = [];
% Create time series object for filtered signal
%ts1 = timeseries(x1, 1:1416);
%ts2 = timeseries(x2, 1:1416);

y1 = 1:1:997;
y2 = 1:1:997;

% Plot the filtered signal in the time domain
figure
% Create tiled layout
t = tiledlayout(1,1);

% First axis (black signal)
ax2 = axes(t);
h2 = plot(ax2, y2, x2, '-k'); % Store handle to plot for legend
ax2.XColor = 'k';
ax2.YColor = 'k';
ylabel(ax2, 'Kecepatan Sudut Bidang Z (deg/s)'); % Left y-axis label
%ylabel(ax2, 'Percepatan Bidang Z (m/s^2');

% Second axis (red signal)
ax1 = axes(t);
h1 = plot(ax1, y1, x1, '-r'); % Store handle to plot for legend
ax1.YColor = 'r';
ax1.YAxisLocation = 'right';
ax1.Color = 'none';
ax1.Box = 'off';
ylabel(ax1, 'Batas Langkah'); % Left y-axis label

% Remove box from the first axis to match
ax2.Box = 'off';

% Add legend with labels for each signal
% Add legend with labels for each signal

% Label the x-axis
xlabel(ax2, 'Time (milliseconds)');