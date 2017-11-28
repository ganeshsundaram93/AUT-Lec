% Glas_data
y1=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_ok_1.wav');
y2=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_ok_2.wav');
y3=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_ok_3.wav');
y4=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_ok_4.wav');
y5=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_ok_5.wav');

y11=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_def_1.wav');
y12=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_def_2.wav');
y13=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_def_3.wav');
y14=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_def_4.wav');
y15=wavread('C:\Users\Sidney\Documents\TU Kaiserslautern\Lectures\Sensig\SENSIG_Lab1_2016_Data\SENSIG_Lab_24112016\Signale\Glas_def_5.wav');

% y2=wavread('C:\Programme\MATLAB71\work\Glass16_ok2.wav');
% y3=wavread('C:\Programme\MATLAB71\work\Glass16_ok3.wav');
% y4=wavread('C:\Programme\MATLAB71\work\Glass16_ok4.wav');
% y5=wavread('C:\Programme\MATLAB71\work\Glass16_ok5.wav');
% y6=wavread('C:\Programme\MATLAB71\work\Glass16_ok6.wav');
% y7=wavread('C:\Programme\MATLAB71\work\Glass16_ok7.wav');
% y8=wavread('C:\Programme\MATLAB71\work\Glass16_ok8.wav');
% y9=wavread('C:\Programme\MATLAB71\work\Glass16_ok9.wav');
% y10=wavread('C:\Programme\MATLAB71\work\Glass16_ok10.wav');
% 
% y11=wavread('C:\Programme\MATLAB71\work\Glass16_defect1.wav');
% y12=wavread('C:\Programme\MATLAB71\work\Glass16_defect2.wav');
% y13=wavread('C:\Programme\MATLAB71\work\Glass16_defect3.wav');
% y14=wavread('C:\Programme\MATLAB71\work\Glass16_defect4.wav');
% y15=wavread('C:\Programme\MATLAB71\work\Glass16_defect5.wav');
% y16=wavread('C:\Programme\MATLAB71\work\Glass16_defect6.wav');
% y17=wavread('C:\Programme\MATLAB71\work\Glass16_defect7.wav');
% y18=wavread('C:\Programme\MATLAB71\work\Glass16_defect8.wav');
% y19=wavread('C:\Programme\MATLAB71\work\Glass16_defect9.wav');
% y20=wavread('C:\Programme\MATLAB71\work\Glass16_defect10.wav');

y1 = y1(1:71669);
y2 = y2(1:71669);
y3 = y3(1:71669);
y4 = y4(1:71669);
y5 = y5(1:71669);
y11 = y11(1:71669);
y12 = y12(1:71669);
y13 = y13(1:71669);
y14 = y14(1:71669);
y15 = y15(1:71669);
%y6 = y6(1:71669);
%y7 = y7(1:71669);
%y8 = y8(1:71669);
%y9 = y9(1:71669);
%y10 = y10(1:71669);

Glass =[y1, y2, y3, y4, y5, y11, y12, y13, y14, y15];
Glass_Class = [1,1,1,1,1,2,2,2,2,2];


figure
subplot(2,2,1)
plot(Glass(:,1))
subplot(2,2,2)
plot(Glass(:,2))
subplot(2,2,3)
plot(Glass(:,6))
subplot(2,2,4)
plot(Glass(:,7))

con2nif(Glass,'Glass_Dat_Sid.nif');
con2nif(Glass_Class,'Glass_Class_Sid.nif');


signal = Glass(:,1);
dim = length(signal);
NFFT = 2^nextpow2(dim);
%NFFT = pow2(nextpow2(dim));

sigFFT = fft (signal, NFFT)/dim;

Mag = abs(sigFFT);
Phase = unwrap(angle(sigFFT));
Re = real(sigFFT);
Im = imag(sigFFT);
FS = 11025; % Sampling rate (hopefully)
f=FS/2*linspace(0,1,NFFT/2);

% Plot single-sided Magnitude spectrum of glass time data

figure;
plot(f,2*Mag(1:NFFT/2))
title('Single-Sided Magnitude Spectrum of Glass Time Data')
xlabel('Frequency in [Hz]')
ylabel('|Y(f)|')

for i=1:1:10 
    signal= Glass(:,i);
    dim = length(signal);
%NFFT = 2^nextpow2(dim);
NFFT = pow2(nextpow2(dim));
sigFFT = fft (signal, NFFT)/dim;
Mag = abs(sigFFT);
Phase = unwrap(angle(sigFFT));
if (i==1)GlassF = Mag; else GlassF = [GlassF, Mag];  end
end

%connif(GlassF,'Glass_FFT_Dat.nif');

%Lab 4

figure;
sig=y3;
subplot(2,1,1)
plot([0:1/11025:(length(sig)-1)/11025], sig)
%plot(y1)
subplot(2,1,2)
%spectrogram(y1,128,20,256,11025)
% specgram(a,nfft,fs,window,numoverlap)
specgram(sig,128,11025, hamming(128),20)
