x0 = [0; .01; -.01; 0; 0; 0];

[tout, xout] = ode45(@(t, y) dynamics(y, 0), [0 10], x0);

for i = 1:length(tout)
    render_frame(xout(i,:));
    drawnow;
end

function [] = render_frame(state)
l = 1;
cart_dims = [.6 .4];
x_left = state(1) - cart_dims(1)/2;
x_right = state(1) + cart_dims(1)/2;

plot([x_left, x_right, x_right, x_left, x_left], [0, 0, cart_dims(2), cart_dims(2), 0],'k')
hold on
plot(state(1)+[0, -l*sin(state(2)), -l*(sin(state(2))+sin(state(3)))], ...
    cart_dims(2)/2 + [0, l*cos(state(2)), l*(cos(state(2))+cos(state(3)))], ...
    '.k-', 'MarkerSize',12);
hold off
xlim([-3 3])
ylim([-2 4])
end