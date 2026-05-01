function robotStruct = link_constants_cyton()
% Link length constants for Cyton
%     % Ref pg 106 Table 3.6 Spong
%     A(:,:,1) = DH(0,pi/2,d1,theta(1));
%     A(:,:,2) = DH(a2,0,0,theta(2));
%     A(:,:,3) = DH(a3,0,0,theta(3));
%     % Ref pg 87 Table 3.3 Spong
%     A(:,:,4) = DH(0,-pi/2,0,theta(4));
%     A(:,:,5) = DH(0,pi/2,0,theta(5));
%     A(:,:,6) = DH(0,0,d6,theta(6));
d(1) = 0.2;
d(2) = 0;
d(3) = 0;
d(4) = 0;
d(5) = 0;
d(6) = 0;
d(7) = 0.09;

a(1) = 0;
a(2) = 0.135;
a(3) = 0.125;
a(4) = 0.125;
a(5) = 0.135;
a(6) = 0;
a(7) = 0;

% MSF: corrected to match the Cyton axes, 03/11/2019; negated 1 & 6
alpha(1) = -pi/2; 
alpha(2) = -pi/2;
alpha(3) = pi/2; 
alpha(4) = -pi/2;
alpha(5) = pi/2;
alpha(6) = -pi/2; 
alpha(7) = 0;

% package this up into a struct for returning
robotStruct = struct();
robotStruct.numJoints = 7;
robotStruct.d = d;
robotStruct.a = a;
robotStruct.alpha = alpha;


