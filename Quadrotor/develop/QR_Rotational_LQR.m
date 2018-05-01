function [u] = QR_Rotational_LQR(quad_lqr, quad, e_r, u)

sum = zeros(4,1);

%Calculate control output needed
for i = 1:4
    for j = 1:6
        row_sum = -quad_lqr.K_lqr_rot(i,j)*(e_r(j,1));
        
        %Saturate RP rotational controls
        if (j == 1 || j == 2 || j == 4 || j == 5)
            if row_sum > quad.r_cont_max
                row_sum = quad.r_cont_max;
            elseif row_sum < -quad.r_cont_max
                row_sum = -quad.r_cont_max;
            end
        end
        sum(i,1) = sum(i,1) + row_sum;
    end
    u(i,1) = u(i,1) + sum(i,1);
end

