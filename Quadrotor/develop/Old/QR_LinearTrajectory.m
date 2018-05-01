function des = QR_LinearTrajectory(q, des_pos, prev_des, vmax, dt)

    %Take in desired endpoint, and plan trajectory accordingly
    %Goal:
    %1) Take the difference between the current position (distance remaining)
    %2) Calculate maximum step in that direction
    %3) If target is beyond that maximum step, then set as trajectory point
    %4) Else set to difference between current pos and final.
    
    %First check the difference between the current position and the desired
    des(:,1) = (des_pos(1:3,1)-q(1:3));
    
    %Second check if the quad is within a certain range of the deisired
    %position, this will avoid oscillations due to discrete time sampling
    %by basing the next trajectory point off the previous trajectory
    %restination, rather than the position directly.
    if (abs(norm(des(:,1),2)) < 10)
        des(:,1) = (des_pos(1:3,1)-prev_des(1:3));
    else

        %Calculate maximum step based off max desired velocity:
        %(max step) = (direction of step) * (max velocity[dx]) * (dt)
        max_step(:,1) = (des(:,1)/norm(des(:,1),2))*vmax*dt;

        %This is done for each vector element to ensure some selectivity
        %between the axis, in the event of axis-specific disturbances.
        for i = 1:3
            %If difference is too great, then limit step to max desired
            if (abs(des(i,1)) > abs(max_step(i,1)))
                des(i,1) = prev_des(i,1) + max_step(i,1);
            end
            %Otherwise it will simple retain the difference between the current
            %point and next point as the destination
        end
    end
    
end
