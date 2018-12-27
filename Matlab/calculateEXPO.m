
clear all;
clear;

rcData = [1000:50:2000];
% length(rcData)        % 21

rcCommand = zeros(1,21);
rcCommandf1 = zeros(1,21);
rcCommandf2 = zeros(1,21);
rcCommandAbs = zeros(1,21);

for i = 1:length(rcData)
    rcCommand(i) = rcData(i) - 1500;
end

for j = 1:length(rcData)
    rcCommandf1(j) = rcCommand(j) / 500.0;
end

figure;
plot(rcCommand(:), rcCommandf1(:));

for k = 1:length(rcData)
    rcCommandAbs(k) = abs(rcCommandf1(k));
end

expo = 20;
expof = expo / 100.0;

for l = 1:length(rcData)
   rcCommandf2(l) = rcCommandf1(l) * rcCommandAbs(l) * rcCommandAbs(l) * rcCommandAbs(l) * expof + rcCommand(l) * (1 - expof);
end

hold on;
plot(rcCommand(:), rcCommandf2(:));
