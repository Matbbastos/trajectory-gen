xi = linspace(0, 10, 6);
xnew = linspace(0, 10, 100);

yi = [0 2 1 4 2 0];

knots = optknt(xi, order);
ref_spline = spapi(knots, xi, yi);

spline = fnval(xnew, ref_spline);
der_spline = gradient(spline);

%% Plots
figure
plot(xnew, spline, '-r', 'LineWidth', 1.5)
hold on
grid on
plot(xi, yi, '-r^', 'LineWidth', 1.5, 'LineStyle', 'none')
legend({'Spline'; 'Ref points'})
title('Spline Interpolation')

figure
plot(xnew, der_spline, '-b', 'LineWidth', 1.5)
grid on
legend({'1st derivative'})
title('Derivatives of Spline')
