function [PID] = QR_Rotational_PID(quad_pid, quad, e_r, err_sum)

%Clear control terms
PID = zeros(4,1);

% Define gain matrixes
K_p = [ quad_pid.kp -quad_pid.kp -quad_pid.kp_yaw;
       -quad_pid.kp -quad_pid.kp  quad_pid.kp_yaw;
        quad_pid.kp  quad_pid.kp  quad_pid.kp_yaw;
       -quad_pid.kp  quad_pid.kp -quad_pid.kp_yaw];

K_d = [ quad_pid.kd -quad_pid.kd -quad_pid.kd_yaw;
       -quad_pid.kd -quad_pid.kd  quad_pid.kd_yaw;
        quad_pid.kd  quad_pid.kd  quad_pid.kd_yaw;
       -quad_pid.kd  quad_pid.kd -quad_pid.kd_yaw];

K_i = [ quad_pid.ki -quad_pid.ki -quad_pid.ki_yaw;
       -quad_pid.ki -quad_pid.ki  quad_pid.ki_yaw;
        quad_pid.ki  quad_pid.ki  quad_pid.ki_yaw;
       -quad_pid.ki  quad_pid.ki -quad_pid.ki_yaw];
   
%Calculate control output needed
%Note this is tuned to output a motor pulse command, not a force!
for i = 1:4
    sum_pd = 0;
    sum_i = 0;
    for j = 1:3
        sum_pd = -[K_p(i,j), K_d(i,j)]*[e_r(j,1); e_r(j+3,1)];
        sum_i = -K_i(i,j)*err_sum(j,1);
        
        %Saturate all integral terms 
        if sum_i > 10
            sum_i = 10;
        elseif sum_i < -10
            sum_i = -10;
        end

        sum_pd = sum_pd + sum_i;
        
        %Saturate R and P control terms
        if(j < 3)   
           if sum_pd > 20
               sum_pd = 20;
           elseif sum_pd < -20
               sum_pd = -20;
           end
        end
        PID(i,1) = PID(i,1) + sum_pd;
    end 
end

