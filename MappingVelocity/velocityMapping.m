t = [1 2 2 2 2 3 3 3 3 4 Inf];

d = [24 40.125 35.5 30.5 25.75 31 24 16.5 9.3750 2.8750 0] * 0.0254;


speed = d./t;
power = 1:-.1:0;

% Reverse
speed = speed(end:-1:1);
power = power(end:-1:1);

plot(power, speed, 'r*');

% Find equation
order = 2;
coeffs = polyfit(power, speed, 2)
powerNew = linspace(0,1);
speedNew = polyval(coeffs, powerNew);
hold on
plot(powerNew, speedNew);

hold off

