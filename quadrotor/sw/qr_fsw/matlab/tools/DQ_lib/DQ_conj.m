function [ dq_star ] = DQ_conj( dq )
% ----------------------------------------------------------------------- %
% Conjugates a dual quaternion according to
%   q^*     = q_1^* + \eps q_2^*
% and the usual quaternion conjugation
%   q_1^*   = [-q_{1,v} ; q_0]
%
% T. Reynolds -- RAIN Lab 8.1.17
% ----------------------------------------------------------------------- %

if length(dq)~=8
    if length(dq) == 6
        dq = [dq(1:3); 0; dq(4:6); 0];
    else
        error('Input is not a dual quaternion')
    end
end

dq  = reshape(dq,8,1);

q1  = dq(1:4);
q2  = dq(5:8);

q1_st   = [-q1(1:3); q1(4)];
q2_st   = [-q2(1:3); q2(4)];

dq_star  = [q1_st; q2_st];
dq_star  = reshape(dq_star,8,1);

end

