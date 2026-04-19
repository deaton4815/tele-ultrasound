function T_0_n = update_cyton_robot(handles, A)
% this gives access to the free parameters in our robot, all other
% parameters are constant (e.g. lengths)
%
% returns the transformation to the last link
h = handles.tforms;

% assign the updated matrices
set(h(2),'Matrix',A{1})
set(h(3),'Matrix',A{2})
set(h(4),'Matrix',A{3})
set(h(5),'Matrix',A{4})
set(h(6),'Matrix',A{5})
set(h(7),'Matrix',A{6})
set(h(8),'Matrix',A{7})

% compute forward kinematics to get end effector pose
T_0_n = A{1}*A{2}*A{3}*A{4}*A{5}*A{6}*A{7};

if isempty(handles.trail)
    return
end

% use pose to drop breadcrumbs from end effector
d = get(handles.trail,{'XData','YData','ZData'});

% shift out old data points and add new ones
x = circshift(d{1},[0 1]);
y = circshift(d{2},[0 1]);
z = circshift(d{3},[0 1]);
x(1) = T_0_n(1,4);
y(1) = T_0_n(2,4);
z(1) = T_0_n(3,4);

% update the trail points
set(handles.trail,'XData',x);
set(handles.trail,'YData',y);
set(handles.trail,'ZData',z);
