#!/usr/bin/octave -q
# vi: syntax=matlab

fd = fopen('/tmp/pm-mldata');
raw = fread(fd, Inf, 'float', 0, 'ieee-le');
fclose(fd);

raw = reshape(raw, 10, [])';

T = 35.e-6;
N_hid = 100;

X = raw(1:end-1, 1:7);
Z = (raw(2:end, 1:2) - raw(1:end-1, 1:2)) / T;

rstd = std(X) * 10;
ravg = mean(X);

X = (X - ravg) ./ rstd;
W_tanh = randn(8, N_hid) / sqrt(7);

H = [ones(size(X,1),1) X] * W_tanh;
H = H ./ (abs(H) + 1);
%H = tanh(H);

W_lin = pinv(H'*H + eye()*5.e+1) * H'*Z;
Z_est = H*W_lin;

err = std(Z-Z_est) * T
len = size(reshape(W_tanh, [], 1), 1) ...
	+ size(reshape(W_lin, [], 1), 1)

p = [Z Z_est];

save -ascii '/tmp/zest.txt' p

