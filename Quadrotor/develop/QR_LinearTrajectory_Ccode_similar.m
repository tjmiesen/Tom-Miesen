function des = QR_LinearTrajectory_Ccode_similar(q, des_pos, prev_des, vmax, dt)
    %This is intendend to be as close to the possible to the C++ code for the QR 


    %Take in desired endpoint, and plan trajectory accordingly
    %Goal:
    %1) Take the difference between the current position (distance remaining)
    %2) Calculate maximum step in that direction
    %3) If target is beyond that maximum step, then set as trajectory point
    %4) Else set to difference between current pos and final.
    
    %Get difference between current position and final target desitnation
    for i = 1:3
        des(i,1) = des_pos(i,1) - q(i,1);
    end
    
    dest_norm = sqrt(des(1,1)*des(1,1)+des(2,1)*des(2,1)+des(3,1)*des(3,1));

    for i = 1:3
        %checks 100mm ball
       if abs(dest_norm) > 50
         max_step = (des(i,1)/dest_norm)*vmax*dt; 
           if (abs(des(i,1)) > abs(max_step))
                des(i,1) = prev_des(i,1) + max_step;
                %des(i,1) = q(i,1) + max_step;
                %des(i,1) = (0.25*q(i,1)+0.75*prev_des(i,1)) + max_step;
           end
       else
           des(i,1) = des_pos(i,1);
       end
    end
end
