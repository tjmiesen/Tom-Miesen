function [ q_unit ] = DQ_normalize( q )

% Outputs a normalized dual quaternion

if length(q) ~= 8
    error('input not a dual quaterion')
end

q_conj  = DQ_conj( q );

q_unit  = DQ_mult( q_conj, q );

end

