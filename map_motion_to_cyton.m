function q = map_motion_to_cyton(q, motion, activeJoint, dq)
% Map classified motion to one Cyton joint

switch lower(motion)
    case 'flex'
        q(activeJoint) = q(activeJoint) + dq;

    case 'extend'
        q(activeJoint) = q(activeJoint) - dq;

    case 'rest'
        % hold current position
end
end
