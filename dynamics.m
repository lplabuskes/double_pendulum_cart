function [state_dot] = dynamics(state, u)
%Compute the time evolution of a double pendulum on a cart with force u on
%the cart
%% Constants
m_c = 1;
m_p = 1;
l = 1;
g = 9.81;
c_c = .1; %Damping coefficient for the cart
c_p = .05; %Damping coefficient for the pendulums

%% State Equations
state_dot = [state(4:end);0;0;0];
theta1 = state(2);
theta2 = state(3);
theta1dot = state(5);
theta2dot = state(6);

%equations derived from Lagrangian mechanics
mass_matrix = [m_c+2*m_p, -1.5*m_p*l*cos(theta1), -.5*m_p*l*cos(theta2);
               -1.5*m_p*l*cos(theta1), 1.25*m_p*l^2, .5*m*l^2*cos(theta1-theta2);
               -.5*m_p*l*cos(theta2), .5*m*l^2*cos(theta1-theta2), .25*m_p*l^2];

rhs = [u - 1.5*m_p*l*theta1dot^2*sin(theta1) - .5*m_p*l*theta2dot^2*sin(theta2);
       1.5*m_p*g*l*sin(theta1) - .5*m_p*l^2*theta2dot^2*sin(theta1-theta2);
       .5*m_p*g*l*sin(theta2) + .5*m_p*l^2*theta1dot^2*sin(theta1-theta2)];
   
state_dot(4:end) = mass_matrix\rhs;

%include damping
state_dot(4:end) = state_dot(4:end) - [c_c;c_p;c_p].*state(4:end);
end

