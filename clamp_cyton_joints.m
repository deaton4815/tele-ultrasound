function q = clamp_cyton_joints(q, qMin, qMax)
% Clamp joint values to safe limits

q = min(q, qMax);
q = max(q, qMin);
end
