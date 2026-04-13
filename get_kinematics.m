function [A, T] = get_kinematics(theta, robotStruct)
% Get kinematics for the Elbow Manipulator
% theta is a list of joint angles
% robot_struct has four fields, which comprise the DH convention for
% describing a robot: (1) numJoints, (2) d, (3) a, (4) alpha
% 
% Returns: A as a cell array of transformation matrices between joints
% Returns: T as a cell array of transformation matrices in global
%       coordinates

% grab info from the struct
numJoints = robotStruct.numJoints;
d = robotStruct.d;
a = robotStruct.a;
alpha = robotStruct.alpha;

% initialize each transformation matrix
A = repmat({eye(4)},[1 numJoints]);

% Ref pg 77 Spong for DH convention
% Ai = DH_transformation(a_i,alpha_i,d_i,theta_i)
% Ai = DH_transformation(linkLength,linkTwist,linkOffset,jointAngle)
% Ai = DH_transformation(Tx,Rx,Tz,Rz)
% Ai = Rot_z * Trans_z * Trans_x * Rot_x

for i = 1:numJoints
    A{i} = DH(a(i),alpha(i),d(i),theta(i));
end

% compute the global transformation matrices if output is size 2
if nargout > 1
    % initialize T and then initialize T{1}
    T = cell(size(A));
    T{1} = A{1};
    
    % post-multiply each transformation matrix
    for i = 2:numJoints
        T{i} = T{i-1}*A{i};
    end
end 

function A = DH(linkLength,linkTwist,linkOffset,jointAngle)
% Compute homogeneous transformation A given Denavit-Hartenberg parameters:
% linkLength    = a_i
% linkTwist     = alpha_i
% linkOffset    = d_i
% jointAngle    = theta_i

% pre-computing the cosine and sine is more efficient
c_theta = cos(jointAngle);
s_theta = sin(jointAngle);

% pre-computing the cosine and sine is more efficient
c_alpha = cos(linkTwist);
s_alpha = sin(linkTwist);

A = [
    c_theta -s_theta*c_alpha  s_theta*s_alpha   linkLength*c_theta;
    s_theta  c_theta*c_alpha -c_theta*s_alpha   linkLength*s_theta;
    0        s_alpha          c_alpha           linkOffset;
    0        0                0                 1;];
