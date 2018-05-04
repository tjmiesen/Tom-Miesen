function qsim = init_qsim( )
% RAINDrop Quadrotor Simulator: Simulation Init File
%
% Thomas Miesen -- RAIN Lab

qsim = struct;

%Simulation Parameters
qsim.g = 9810;  %mm/s^2
qsim.tmax = 10;
qsim.dt = 0.001; %simulation dt in s (NOTE: 10ms is ONLY for to speed up the SETUP process)
qsim.N = qsim.tmax/qsim.dt;
qsim.k1 = 0.01;    %translational damping terms (drag forces) (Ns/m) = kg/s 
qsim.k2 = 0.01;
qsim.k3 = 0.01;
qsim.k4 = 0.04;   %rotational damping terms
qsim.k5 = 0.04;
qsim.k6 = 0.04;
qsim.t_loop = 0;
qsim.t_loop_pos = 0;
qsim.t_loop_rot = 0;

end

