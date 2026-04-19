function jac = numeric_jacobian(q, robotStruct)

% Kinematics for the Elbow Robot
[~, T] = get_kinematics(q, robotStruct);

% Define components for Jacobian:
% z-axis is just the 3rd column of the transforms
z = cell(robotStruct.numJoints, 1);
z{1} = [0 0 1]';
for loop = 2 : length(z)
    z{loop} = T{loop - 1}(1:3,3);
end

% The joint centers are just the 4th column from each transformation
% matrix to the base
o = cell(robotStruct.numJoints + 1, 1);
o{1} = [0 0 0]';
for loop = 2 : length(o)
    o{loop} = T{loop - 1}(1:3,4);
end

% Per Eq. 4.64, J is the Geometric Jacobian
oc = o{end};
J = cell(robotStruct.numJoints, 1);
for loop = 1 : length(J)
    J{loop} = cross(z{loop}, (oc-o{loop}));
end

% construct the top half of the Jacobian, that corresponds to endpoint
J_endpoint = J{1};
J_orientation = z{1};
for loop = 2:length(J)
    J_endpoint = [J_endpoint, J{loop}];
    J_orientation = [J_orientation, z{loop}];
end

% stack the two jacobian components
jac = [J_endpoint; J_orientation]; 

