clear all

%format shorteng
format longeng

				%
				% Setup
				%
g = 9.80665;  % Gravitational acceleration (m/s^2)
e_in_max = 36.5;
quantization_theta_1 = 2 * pi / (4 * 500);  % Quadrature encoder with 500 counts steps per turn per A and B channel
quantization_e_in = e_in_max / 255;  % Motor controlled by 8-bit PWM

sim_duration = 0.5;  % Length of simulation (s)
initial_angle = (1/1000) * pi / 180;  % (rad)

is_underdamped = false
is_deadbeat = false  % Can design gains for deadbeat control

is_messing_with_sim_model = true;  % Whether or not sim model matches obs and con models
mess_factor = 0.125;  % Parameters (alpha etc.) changed by 1+rand*mess_factor
is_quantizing_theta_1 = true;  % Whether or not to simulate quantization error
is_quantizing_e_in = true;  % Whether or not to simulate quantization of input
is_using_observer = true;  % Whether or not controller uses estimated state vector or not
is_limiting_e_in = true;
fprintf("Create scope for current.  Get e_a - e_b and divide by R_a.\n");

% Flywheel geometry, mass, and inertia
num_flywheels = 0;
flywheel_mass = num_flywheels * 90.9 / 1000  % (kg)
Jc_flywheel = num_flywheels * 335807.02 * (1/100)^2 * (1/1000)  % (kg*m^2)

% Motor parameters
Ra = 2.07;  % (Ohm)
Kt = .0525;  % (N*m/A)
Kb = 60 / (182 * 2 * pi);  % (V*s)
La = .62 / 1000;  % (H)
b2 = .0062 / 1000;  % (N*m*s)
Jc_armature = 69.6 * (1/100)^2 * (1/1000);  % (kg*m^2)
motor_mass = .34;

% Combined motor and flywheel
m2 = flywheel_mass + motor_mass;
Jc_2 = Jc_flywheel + Jc_armature;
l1 = 0.04;  % Distance from bearing axis to motor axis --- I think this val. is correct!!! (m)
fprintf("Using made-up l_1 value!!\n")
J2 = Jc_2 + m2 * l1^2;

% Mass and inertia values of rod and plate via SolidWorks model
% Printed rod
l_rod = .01916;  % Distance from bearing axis to rod COM (m)
rod_mass = 22.35 / 1000;  % (kg)
Jc_rod = 7932.43 * (1 / 1000) * (1 / 1000)^2;  % kg * mm^2
fprintf("b1 is a made-up value.\n");
b1 = b2 * 10;  % This can be determined experimentally.  Right now it's a fabrication!!!!!!

% Combined rod and motor plate
m1 = rod_mass;
lc1 = l_rod;
J1 = Jc_rod + rod_mass * l_rod^2;


			  %
			  % Continuous-time state-space representation
			  %
rho = Kb * Kt / Ra + b2;
beta = Kt / Ra;
gamma = (m1 * lc1 + m2 * l1) * g;
phi = 1 / (J1 + m2 * l1^2);

A = [0, 1, 0;
     phi * gamma, -phi * b1, phi * rho;
     -phi * gamma, phi * b1, -(1 / Jc_2 + phi) * rho];
B = [0;
     -phi * beta;
     (1 / Jc_2 + phi) * beta];
C = [1, 0, 0];
D = 0;
p = ss(A, B, C, D);

			    %
			    % Verify controllability and observability
			    %
C_M = [B, A*B, A*A*B];
if 3 ~= rank(C_M)
  fprintf("System is not controllable.\n")
  return
  
end

O_M = [C;
       C*A;
       C*A*A];
if 3 ~= rank(O_M)
  fprintf("System is not observable.\n")
  return
  
end

	   %
	   % Discretize model for simulation, observer, and controller
	   %
% Time steps
h_sim = 1E-5;
obs_factor = 100;  % Observer calcs run this factor slower than simulation
h_obs = h_sim * obs_factor;
con_factor = obs_factor;  % Controller calcs run this factor slower than simulation
h_con = h_sim * con_factor;

% Discretized SS representations
pd_sim = c2d(p, h_sim);
pd_obs = c2d(p, h_obs);
pd_con = c2d(p, h_con);

if is_messing_with_sim_model
  rho = rho * (1 + rand * mess_factor);
  beta = beta  * (1 + rand * mess_factor);
  gamma = gamma * (1 + rand * mess_factor);
  phi = phi * (1 + rand * mess_factor);

  A_mess = [0, 1, 0;
       phi * gamma, -phi * b1, phi * rho;
       -phi * gamma, phi * b1, -(1 / Jc_2 + phi) * rho];
  B_mess = [0;
       -phi * beta;
       (1 / Jc_2 + phi) * beta];
  C = [1, 0, 0];
  D = 0;
  p_mess = ss(A_mess, B_mess, C, D);
  pd_sim = c2d(p_mess, h_sim);

end


			 %
			 % Calculate gains for observer and controller
			 %
% Desired closed-loop poles
if is_underdamped  % Desired CL poles, underdamped
  overshoot = .2;
  Ts = .1;  % Settling time
  zeta = -log(overshoot) / sqrt(pi^2 + (log(overshoot))^2);
  omega_n = 4 / (zeta * Ts);
  
  s1 = -zeta * omega_n + omega_n * sqrt(zeta^2 - 1);
  s2 = -zeta * omega_n - omega_n * sqrt(zeta^2 - 1);
  s3 = -5 * zeta * omega_n;

else  % Desired CL poles, overdamped
  Ts = .3;  % Settling time
  a = 4 / Ts;
  
  s1 = -a;
  s2 = 5 * s1;
  s3 = 2 * s2;
  
end

% Desired poles in z-plane
z1 = exp(s1 * h_con);
z2 = exp(s2 * h_con);
z3 = exp(s3 * h_con);

if is_deadbeat
  z1 = 0;
  z2 = 0;
  z3 = 0;
end

% Regulator design via Ackermann's formula
alpha_d = poly([z1, z2, z3]);
phi = pd_con.A * pd_con.A * pd_con.A + alpha_d(2) * pd_con.A * pd_con.A + alpha_d(3) * pd_con.A + alpha_d(4) * eye(3);
K_d = [0, 0, 1] * inv([pd_con.B, pd_con.A * pd_con.B, pd_con.A * pd_con.A * pd_con.B]) * phi

% Observer design
observer_scale = 4;  % Multiply CL system gains
s1 = s1 * observer_scale;
s2 = s2 * observer_scale;
s3 = s3 * observer_scale;
z1 = exp(s1 * h_obs);
z2 = exp(s2 * h_obs);
z3 = exp(s3 * h_obs);
alpha_d = poly([z1, z2, z3]);
phi = pd_obs.A * pd_obs.A * pd_obs.A + alpha_d(2) * pd_obs.A * pd_obs.A + alpha_d(3) * pd_obs.A + alpha_d(4) * eye(3);
K_e = phi * inv([C; C * pd_obs.A; C * pd_obs.A * pd_obs.A]) * [0; 0; 1]  % Ackermann's formula




% State vectors and output
x_sim = zeros(3, round(sim_duration / h_sim));
x_sim(:, 1) = [initial_angle; 0; 0];
x_obs = [initial_angle; 0; 0];
x_con = zeros(3, sim_duration / h_con);
y(1) = C * x_sim(:, 1);
fprintf("\nInitialize initial observer estimate in Arduino!!!!!!!!!!\n\n")



				%
				% Simulation loop
				%
k_obs = 1;  % Index for observer
k_con = 1;  % Index for controller

u(1) = 0;  % Input

for k_sim = 2:1:round(sim_duration / h_sim)
  % Update simulated state vector
  x_sim(:, k_sim) = pd_sim.A * x_sim(:, k_sim - 1) + pd_sim.B * u(k_con);

  % Calculate output and apply possibly quantization effects
  y(k_sim) = C * x_sim(:, k_sim);
  %true_count = fix(y / quantization_theta_1);  % Assume perfect counting of encoder pulses
  
  if is_quantizing_theta_1
    y(k_sim) = y(k_sim) - mod(y(k_sim), quantization_theta_1);
    %y(k_sim) = fix(y(k_sim) / quantization_theta_1) * quantization_theta_1;
    
  end

  
  % Update control action
  if ~mod(k_sim, con_factor)  % Simulation loops con_factor times before control action updates
    k_con = k_con + 1;

    if is_using_observer
      u(k_con) = -K_d * x_obs(:, k_obs);  % Calculate control action based on estimated state vector

    else
      u(k_con) = -K_d * x_sim(:, k_obs);  % Calculate control action based on true (simulated) state vector

    end

    if is_quantizing_e_in
      u(k_con) = fix(u(k_con) / quantization_e_in) * quantization_e_in;  % Input from PWM signal

    end

    if is_limiting_e_in && (e_in_max < abs(u(k_con)))  % Need to limit input to battery voltage
      u(k_con) = sign(u(k_con)) * e_in_max;

    end

  end  % End controller branch

  
  % Update observer estimate via predictive observer
  if ~mod(k_sim, obs_factor)  % Simulation loops obs_factor times before observer updates
    k_obs = k_obs + 1;
    
    x_obs(:, k_obs) = pd_obs.A * x_obs(:, k_obs - 1) + pd_obs.B * u(k_con) + K_e * (y(k_sim) - C * x_obs(:, k_obs - 1));

  end  % End observer branch

end  % End simulation loop


				%
				% Plots
				%
t_sim = [0:h_sim:sim_duration-h_sim];
t_obs = [0:h_obs:sim_duration-h_obs];
t_con = [0:h_con:sim_duration-h_con];

%figure('units','normalized','outerposition',[0 0 1 1])

subplot(2, 2, 1)
stairs(t_sim, x_sim(1, :))
hold on
stairs(t_sim, y)
stairs(t_obs, x_obs(1, 1:length(x_obs)-1))
legend('$x_1$', '$y_q$', '$\hat{x_1}$', 'Interpreter', 'latex')
title('$\theta_1$', 'Interpreter', 'latex')

subplot(2, 2, 2)
stairs(t_sim, x_sim(2, :))
hold on
stairs(t_obs, x_obs(2, 1:length(x_obs)-1))
legend('$x_2$', '$\hat{x_2}$', 'Interpreter', 'latex')
title('$\omega_1$', 'Interpreter', 'latex')

subplot(2, 2, 3)
stairs(t_sim, x_sim(3, :))
hold on
stairs(t_obs, x_obs(3, 1:length(x_obs)-1))
legend('$x_3$', '$\hat{x_3}$', 'Interpreter', 'latex')
title('$\omega_2$', 'Interpreter', 'latex')

subplot(2, 2, 4)
stairs(t_con, u(1, 1:length(u)-1))
title('$u$', 'Interpreter', 'latex')
