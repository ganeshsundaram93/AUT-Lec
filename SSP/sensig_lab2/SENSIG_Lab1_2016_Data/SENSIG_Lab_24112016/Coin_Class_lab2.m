% Coin_data
y1=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro1.wav');
y2=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro2.wav');
y3=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro3.wav');
y4=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro4.wav');
y5=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro5.wav');
y6=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro6.wav');
y7=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro7.wav');
y8=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro8.wav');
y9=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro9.wav');
y10=wavread('C:\Programme\MATLAB71\work\Coin16_2Euro10.wav');

y11=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro1.wav');
y12=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro2.wav');
y13=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro3.wav');
y14=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro4.wav');
y15=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro5.wav');
y16=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro6.wav');
y17=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro7.wav');
y18=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro8.wav');
y19=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro9.wav');
y20=wavread('C:\Programme\MATLAB71\work\Coin16_1Euro10.wav');


y21=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent1.wav');
y22=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent2.wav');
y23=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent3.wav');
y24=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent4.wav');
y25=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent5.wav');
y26=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent6.wav');
y27=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent7.wav');
y28=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent8.wav');
y29=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent9.wav');
y30=wavread('C:\Programme\MATLAB71\work\Coin16_2Cent10.wav');

y31=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent1.wav');
y32=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent2.wav');
y33=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent3.wav');
y34=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent4.wav');
y35=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent5.wav');
y36=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent6.wav');
y37=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent7.wav');
y38=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent8.wav');
y39=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent9.wav');
y40=wavread('C:\Programme\MATLAB71\work\Coin16_1Cent10.wav');

%y1 = y1(1:71669);
%y2 = y2(1:71669);
%y3 = y3(1:71669);
%y4 = y4(1:71669);
%y5 = y5(1:71669);
%y6 = y6(1:71669);
%y7 = y7(1:71669);
%y8 = y8(1:71669);
%y9 = y9(1:71669);
%y10 = y10(1:71669);

Coin =[y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12, y13, y14, y15, y16, y17, y18, y19, y20, y21, y22, y23, y24, y25, y26, y27, y28, y29, y30,y31, y32, y33, y34, y35, y36, y37, y38, y39, y40];
Coin_Class = [1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4];


figure
subplot(2,2,1)
plot(Coin(:,1))
subplot(2,2,2)
plot(Coin(:,11))
subplot(2,2,3)
plot(Coin(:,21))
subplot(2,2,4)
plot(Coin(:,31))


con2nif(Coin,'Coin_Lab2_Dat.nif');
con2nif(Coin_Class,'Coin_Lab2_Class.nif');


signal = Coin(:,1);
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
title('Single-Sided Magnitude Spectrum of Coin Time Data')
xlabel('Frequency in [Hz]')
ylabel('|Y(f)|')

for i=1:1:10 
    signal= Coin(:,i);
    dim = length(signal);
%NFFT = 2^nextpow2(dim);
NFFT = pow2(nextpow2(dim));
sigFFT = fft (signal, NFFT)/dim;
Mag = abs(sigFFT);
Phase = unwrap(angle(sigFFT));
if (i==1)CoinF = Mag; else CoinF = [CoinF, Mag];  end
end

%connif(CoinF,'Coin_FFT_Dat.nif');

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
