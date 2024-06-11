%% Original Signal
fc = 20;
fs = 100;
x = [-7.3087
-7.5127
-7.7569
-7.8786
-8.247
-9.0628
-10.0019
-10.735
-11.2998
-11.8403
-12.3524
-12.6775
-12.5998
-12.1037
-11.4262
-10.8113
-10.3698
-9.9256
-9.1769
-8.2019
-7.3898
-6.9428
-6.7753
-6.8064
-7.0323
-7.3974
-7.8605
-8.4439
-9.0962
-9.7039
-10.2216
-10.6527
-10.9702
-11.1045
-11.0618
-10.9277
-10.7586
-10.5767
-10.349
-10.0516
-9.7898
-9.667
-9.6267
-9.6149
-9.6167
-9.4971
-9.1423
-8.6451
-8.1882
-7.8374
-7.5665
-7.3233
-7.0263
-6.6744
-6.3347
-6.0477
-5.8446
-5.7766
-5.837
-5.9071
-5.9424
-6.0268
-6.2812
-6.8457
-7.7293
-8.6639
-9.1662
-8.8906
-8.1615
-7.7604
-7.848
-7.759
-6.9772
-5.6659
-4.3508
-3.4043
-2.8035
-2.3741
-2.0738
-1.9635
-2.1301
-2.6005
-3.2677
-4.0159
-4.8426
-5.7683
-6.691
-7.4497
-8.0244
-8.4776
-8.7817
-8.9097
-8.9561
-9.0347
-9.0948
-8.926
-8.394
-7.6305
-6.9053
-6.2223
-5.3403
-4.2332
-3.1319
-2.2458
-1.5905
-1.0156
-0.3688
0.4037
1.1897
1.7692
2.1389
2.596
3.3486
4.2309
4.8785
5.1678
5.3173
5.5587
5.8995
6.2886
6.8613
7.572
7.8393
7.2762
6.4317
6.096
6.1607
5.8889
5.0615
4.1788
3.6435
3.2151
2.5166
1.6367
0.891
0.4042
0.1266
0.0213
-0.0324
-0.2874
-0.7999
-1.3567
-1.8237
-2.2934
-2.8893
-3.645
-4.661
-6.0559
-7.5926
-8.9171
-10.204
-11.8705
-14.0519
-16.9518
-20.507
-23.6098
-25.3789
-26.5384
-28.1028
-29.7099
-30.1573
-28.7412
-25.3813
-20.0723
-13.1844
-5.8302
1.0458
7.1305
11.9529
14.9517
16.3598
17.2802
18.6574
20.3122
21.5219
22.0946
22.0126
20.2261
15.1207
7.5088
1.8768
2.3015
7.7174
13.2654
15.6723
15.2594
13.6789
12.0158
10.7456
10.0413
9.5927
8.5455
6.4825
4.2567
3.0415
2.9358
3.0394
2.5233
1.4718
0.8176
1.2335
2.2696
3.0523
3.4245
3.7526
4.1043
4.229
4.0116
3.6337
3.3759
3.2901
3.1836
3.037
3.0677
3.2917
3.4689
3.4589
3.4
3.5171
3.8421
4.2027
4.4232
4.5204
4.6483
4.8995
5.2172
5.4628
5.6035
5.7253
5.9111
6.1689
6.3831
6.4235
6.3168
6.2172
6.1964
6.1828
6.1435
6.1705
6.3614
6.6625
6.8025
6.5474
];
% Create time series object
ts1 = timeseries(x, 1:243);

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
ts2 = timeseries(filtered, 1:243);

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
