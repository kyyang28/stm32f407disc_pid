
a0 = 9.996278957172137756013002477405568253404e-1;
a1 = 9.979387291070364307450237392251445288594e-1;
a2 = 5.028986508540491482566165849167001609232e-1;
a3 = 1.764862321902469630550896534647276819692e-1;
a4 = 3.996291422520886755278730528311966596433e-2;

x = (-1:0.01:1);
len = length(x);
fastexp = zeros(1, len);
for i = 1:len
    fastexp(i) = a0 + x(i) * (a1 + x(i) * (a2 + x(i) * (a3 + x(i) * a4)));
end

plot(x, fastexp, 'LineWidth', 8, 'Color', 'b');
grid on;

hold on;
plot(x, exp(x), 'LineWidth', 2, 'Color', 'r');

xlabel("interval x=[-1,1]");
ylabel("exp(x) and fastexp(x)");
title("fastexp algo VS exp algo");
