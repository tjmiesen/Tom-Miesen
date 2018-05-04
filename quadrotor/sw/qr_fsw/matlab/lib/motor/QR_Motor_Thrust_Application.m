%% RAINDrop Quadrotor Simulator: Motor Thrust Application
%Thomas Miesen

for i = 1:4
   %Shift each control thrust input to hover thrust range (mg/4 per motor)
   u(i,t) = u(i,t) + quad.m*qsim.g/4;
   
   %Convert motor speeds to applied thrusts
   %(note: this is done for LQR as well to account for rounding error in the
   %motor speed value)
   %(note: the conversion is actually for the force measured from all four
   %motors running at that uS pulse. So the individual forces are
   %multiplied by 4, then converted to N)
   motor_cmd(i,t) = -193.51*(u(i,t)*4/1000)^2 + 640.42*(u(i,t)*4/1000) + 1011.5;
   
   %Round to simulate integer conversion on QR
   %mot(i,t) = round(mot(i,t));
   motor_cmd(i,t) = round(10*motor_cmd(i,t))/10;    %Simulate single floating point
   
   %ROT ONLY REMOVE OTHERWISE
   %mot(i,t) = mot(i,t) + 100; %shift up by a small value to allow for actuation

   %Apply PID control
   motor_cmd(i,t) = motor_cmd(i,t) + cont_PID(i,1);
   
   %Since the max operating range of the command pulse is 1000-1900, the
   %linearized QR dynamics has the command pulse set to ~460. So we shift
   %down the max and min values to account for this.
   if motor_cmd(i,t) > quad.mmax
       motor_cmd(i,t) = quad.mmax;
   elseif motor_cmd(i,t) < quad.mmin
       motor_cmd(i,t) = quad.mmin;
   end
   
   %Simulate uS pulse legnth noise from interrupt subroutines
   %Currently modeled as simple variable offset
   motor_cmd(i,t) = motor_cmd(i,t) + noise.motor_pulse_noise;
   %mot(i,t) = mot(i,t) + motor_noise + sound;
   
   %Convert motor command back to a force (kg*mm/s^2
   u(i,t) = 1000*((2.3159e-6)*motor_cmd(i,t)^2 - (3.5209e-3)*motor_cmd(i,t) + 1.2079)/4;
end
