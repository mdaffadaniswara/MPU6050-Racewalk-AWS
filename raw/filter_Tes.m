%% Original Signal
fc = 20;
fs = 100;
x = [ ];
% Create time series object
ts1 = timeseries(x, 2:14609);

% Plot the original signal in the time domain
figure(1)
plot(ts1)
xlabel('Time')
ylabel('Amplitude')
title('Original Signal in Time Domain')

% Apply Butterworth filter
[bb, a] = butter(3, fc / (fs/2), 'low');
filtered = filtfilt(bb, a, x);

% Create time series object for filtered signal
ts2 = timeseries(filtered, 2:14609);

% Plot the filtered signal in the time domain
figure(2)
plot(ts2, 'r')
xlabel('Time')
ylabel('Amplitude')
title('Filtered Signal in Time Domain')

% Frequency domain analysis
figure(3)
% Compute the frequency spectrum of the original signal
frequencies_original = fftshift(fft(x));
% Create frequency axis
f_axis_original = linspace(-fs/2, fs/2, length(frequencies_original));
plot(f_axis_original, abs(frequencies_original))
xlabel('Frequency (Hz)')
ylabel('Magnitude')
title('Original Signal in Frequency Domain')

figure(4)
% Compute the frequency spectrum of the filtered signal
frequencies_filtered = fftshift(fft(filtered));
% Create frequency axis
f_axis_filtered = linspace(-fs/2, fs/2, length(frequencies_filtered));
plot(f_axis_filtered, abs(frequencies_filtered), 'r')
xlabel('Frequency (Hz)')
ylabel('Magnitude')
title('Filtered Signal in Frequency Domain')