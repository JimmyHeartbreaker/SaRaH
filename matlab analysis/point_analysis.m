clc; clear; close all;

distances = readtable('wallDataCorrected.csv').Var1;

figure
plot(distances);
Y = fft(distances);
N = length(Y);
P2 = abs(Y/N);
P1 = P2(1:N/2+1);        % one-sided spectrum, drop other side
P1(2:end-1) = 2*P1(2:end-1);

f = (0:(N/2)) / N;

figure
plot(f, P1)
xlabel('Frequency (Hz or cycles/rev)')
ylabel('|Amplitude|')
title('LiDAR Distance FFT Spectrum')
grid on




% filtered = step(wall_filter2,distances);
% figure
% plot(filtered);
% hold on
% plot(distances)
% hold off


% Fs = 1024;       % sample rate (adjust to your system)
% Fc = 100;        % desired cutoff frequency in Hz
% 
% % Design 2nd-order Butterworth low-pass
% [b,a] = butter(2, Fc/(Fs/2), 'low');  
% 
% % Convert to single precision
% b = single(b);
% a = single(a);
% 
% % Convert to DF2T format for CMSIS
% [sos,g] = tf2sos(b,a);    % second-order section
% b_scaled = sos(:,1:3);    % numerator
% a_scaled = sos(:,3:5);    % denominator a1,a2
% 
% % Multiply numerator by gain factor
% b_scaled = b_scaled * g;
gain = [ 0.079377897083759307861328125  *2.15            ;       
   0.064816884696483612060546875       *2.15        ;
  0.0586096234619617462158203125      *2.15            ];


b1 = [gain(1) gain(1)*2 gain(1)]; a1 = [1 -0.4556631743907928466796875   0.09316779673099517822265625  ];
b2 = [gain(2) gain(2)*2 gain(2)]; a2 = [1  -0.587421238422393798828125    0.1616139113903045654296875];
b3 = [gain(3) gain(3)*2 gain(3)]; a3 = [1  -0.70819938182830810546875         0.23030115664005279541015625];

% Create individual DF2T filters
Hd1 = dfilt.df2t(b1, a1);
Hd2 = dfilt.df2t(b2, a2);
Hd3 = dfilt.df2t(b3, a3);

% Cascade them
Hd = dfilt.cascade(Hd1, Hd2, Hd3);
y = filter(Hd, distances);

figure
plot(y);
hold on
plot(distances)
hold off

distances2 = readtable('wallDataFiltered.csv').Var1;

figure
plot(distances);
hold on
plot(distances2);
hold off


s1 = readtable('sample1.csv').Var1;
s2 = readtable('sample2.csv').Var1;

figure
plot(s1);
hold on
plot(s2);
hold off
