function [u] = QR_Position_LQR(quad_lqr, measurement_buffer, quad, e, u)

K = [ quad_lqr.K_lqr_full(1,1)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(1,2)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1)))     ,quad_lqr.K_lqr_full(1,3:6),      quad_lqr.K_lqr_full(1,7)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(1,8)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))),        quad_lqr.K_lqr_full(1,9:12);
      quad_lqr.K_lqr_full(2,1)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(2,2)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1)))     ,quad_lqr.K_lqr_full(2,3:6),      quad_lqr.K_lqr_full(2,7)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(2,8)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))),        quad_lqr.K_lqr_full(2,9:12);
      quad_lqr.K_lqr_full(3,1)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(3,2)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1)))     ,quad_lqr.K_lqr_full(3,3:6),      quad_lqr.K_lqr_full(3,7)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(3,8)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))),        quad_lqr.K_lqr_full(3,9:12);
      quad_lqr.K_lqr_full(4,1)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(4,2)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1)))     ,quad_lqr.K_lqr_full(4,3:6),      quad_lqr.K_lqr_full(4,7)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), quad_lqr.K_lqr_full(4,8)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))),        quad_lqr.K_lqr_full(4,9:12)];

%Calculate control output needed
for i = 1:4
    row_sum = 0;
    for j = 1:12
        col_sum = -K(i,j)*(e(j,1));
        %Saturate XY Position Controls
        if (j == 1 || j == 2 || j == 7 || j == 8)
            if col_sum > quad.p_cont_max
                col_sum = quad.p_cont_max;
            elseif col_sum < -quad.p_cont_max
                col_sum = -quad.p_cont_max;
            end
        %Saturate RP Attitude Controls
        elseif (j == 4 || j == 5 || j == 10 || j == 11)
            if col_sum > quad.r_cont_max
                col_sum = quad.r_cont_max;
            elseif col_sum < -quad.r_cont_max
                col_sum = -quad.r_cont_max;
            end
        end
        row_sum = row_sum + col_sum;
    end
    u(i,1) = u(i,1) + row_sum;
end

