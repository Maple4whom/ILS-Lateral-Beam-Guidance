% Joshua Yandoc 
% State derivative function for Lateral Beam Guidance System Dynamics

function xdot = Lateral_Beam_Guidance_System_State_Deriv_JoshuaYandoc(x)

% transfer global parameters from main program
global B_SM g G_c J_M K_A K_D K_E K_P K_R K_T K_V L_A R_A T_A V_T R K_I

 % array x contains our state variables - initialize them to their proper
 % corresponding variable
 i = x(1); % current [A]
 delta_a = x(2); % aileron deflection angle [rad]
 delta_a_dot = x(3); % aileron deflection angle rate [rad/s]
 phi = x(4); % roll angle converted to [rad]
 p = x(5); % roll rate - dphi/dt converted to [rad/s]
 psi = x(6); % heading angle converted to [rad]
 Y_R = x(7); % lateral displacement [m]


 % Calculations of Guidance system controls and its subsystems

 lambda = Y_R/R; % angular error between aircraft and centre line; using small angle approx.

 % coupler of the overall Lateral Beam Guidance System; psi_c is outputted
 % from coupler into aircraft and lateral autopilot controller to output an
 % appropriate heading angle psi, which will influence the lateral speed,
 % dY_R/dt (xdot(7))
 %psi_c = -G_c  * lambda; % commanded heading angle [rad]
 % for Q8
 psi_c = -G_c  * K_I * lambda;

 % aircraft and lateral autopilot subsystem dynamic equations
 % outer loop compares the commanded heading, psi_c, and the actual
 % heading, psi; this comparison passes through the directional gyro, which
 % is represented by gain K_D. The resulting signal is the required (commanded) roll
 % angle, phi_c
 phi_c = K_D * (psi_c - psi); % [rad]
 % once the commanded roll angle is found, it is compared with the actual
 % roll angle, phi, through the second loop, where it passes through a
 % vertical gyro, represented by gain K_V. This is the commanded roll rate,
 % dphi_c/dt = p_c (recall that p = roll rate)
 p_c = K_V * (phi_c - phi); % [rad/s]
 % The final loop uses the outputted commanded roll rate to measure an error signal e which uses the
 % aircraft's roll rate through the roll rate gyro, represented by gain
 % K_R, and compares it to the commanded roll rate (see fig. 6)
 e = p_c - (p * K_R); % error measures offset of measured roll rate from commanded roll rate [rad/s]
 % For the aileron servo-motor subsystem, the aileron angular deflection is
 % regulated so that it follows and is compared to the reference signal e,
 % and then amplified by gain K_P to produce the input voltage for the
 % motor, V_A
 V_A = K_P * (e - delta_a); % [V]

 % output the state derivative equations
 % x(1) dot = di/dt
 xdot(1) = (-(R_A/L_A) * i) - ((K_E/L_A)*delta_a_dot) + (V_A/L_A);
 % x(2) dot = ddelta_a/dt
 xdot(2) = delta_a_dot;
 % x(3) dot = d^2delta_a/dt^2
 xdot(3) = ( -(B_SM/J_M)*delta_a_dot) + ((K_T/J_M)*i);
 % x(4) dot = dphi/dt
 xdot(4) = p; % convert to [rad/s]
 % x(5) dot = d^2phi/dt^2
 xdot(5) = (-(1/T_A)*p) +  ((K_A/T_A)*delta_a);
 % x(6) dot = dpsi/dt
 xdot(6) = (g/V_T)*phi;
 % x(7) dot = dY_R/dt
 xdot(7) = V_T*sin(psi);


end

