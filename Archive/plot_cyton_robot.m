function handles = plot_cyton_robot()

% create a set of points for a 1x1x2 cylinder
scale = [0.02; 0.02; 0.07;];
numSides = 20;
[xData,yData,zData] = cylinder(1,numSides);
fvc = surf2patch(xData,yData,zData);
fvc.vertices = [fvc.vertices; 0 0 0; 0 0 1];
for i = 1:2:2*numSides
    fvc.faces = [fvc.faces; i i+2 2*numSides+3 2*numSides+3;];
    fvc.faces = [fvc.faces; i+1 i+3 2*numSides+4 2*numSides+4;];
end

fvc.vertices(:,1) = fvc.vertices(:,1).*scale(1);
fvc.vertices(:,2) = fvc.vertices(:,2).*scale(2);
fvc.vertices(:,3) = fvc.vertices(:,3).*scale(3) -scale(3)/2;

% Create all the plot objects we'll need (cylinders, coordinate frames,
% text labels)

% first draw the cylinders
p(1) = patch(fvc);
p(2) = patch(fvc);
p(3) = patch(fvc);
p(4) = patch(fvc);
p(5) = patch(fvc);
p(6) = patch(fvc);
p(7) = patch(fvc);

set(p,'EdgeColor','None');
% set(p,'EdgeColor','k');

% next draw the coordinate frames
scaleTriad = 0.08;
t(1) = PlotUtils.triad(eye(4),scaleTriad);  % GCS
t(2) = PlotUtils.triad(eye(4),scaleTriad);  % x0y0z0
t(3) = PlotUtils.triad(eye(4),scaleTriad);  % x1y1z1
t(4) = PlotUtils.triad(eye(4),scaleTriad);  % x2y2z2
t(5) = PlotUtils.triad(eye(4),scaleTriad);  % x3y3z3
t(6) = PlotUtils.triad(eye(4),scaleTriad);  % x4y4z4
t(7) = PlotUtils.triad(eye(4),scaleTriad);  % x5y5z5
t(8) = PlotUtils.triad(eye(4),scaleTriad);  % x6y6z6
t(9) = PlotUtils.triad(eye(4),scaleTriad);  % x7y7z7

% next draw the text
tX = 0.03;
tY = 0.02;
cs_text(1) = text(tX,tY,0,'A0');
cs_text(2) = text(tX,tY,0,'A1');
cs_text(3) = text(tX,tY,0,'A2');
cs_text(4) = text(tX,tY,0,'A3');
cs_text(5) = text(tX,tY,0,'A4');
cs_text(6) = text(tX,tY,0,'A5');
cs_text(7) = text(tX,tY,0,'A6');
cs_text(8) = text(tX,tY,0,'A7');

% create MATLAB transformation containers and assign hierarchy
% add a nominal offset to show all the joints
% (this is for temporary vizualization only
base_offset = [.1 0 0];
h(1) = hgtransform('Parent',gca);
h(2) = hgtransform('Parent',h(1), 'Matrix',makehgtform('translate',base_offset));
h(3) = hgtransform('Parent',h(2), 'Matrix',makehgtform('translate',base_offset));
h(4) = hgtransform('Parent',h(3), 'Matrix',makehgtform('translate',base_offset));
h(5) = hgtransform('Parent',h(4), 'Matrix',makehgtform('translate',base_offset));
h(6) = hgtransform('Parent',h(5), 'Matrix',makehgtform('translate',base_offset));
h(7) = hgtransform('Parent',h(6), 'Matrix',makehgtform('translate',base_offset));
h(8) = hgtransform('Parent',h(7), 'Matrix',makehgtform('translate',base_offset));

% assign all the patches to the transformation containers
set(p(1),'Parent',h(1));
set(p(2),'Parent',h(2));
set(p(3),'Parent',h(3));
set(p(4),'Parent',h(4));
set(p(5),'Parent',h(5));
set(p(6),'Parent',h(6));
set(p(7),'Parent',h(7));

% assign all the triads to the transformation containers
set(t(1),'Parent',h(1));
set(t(2),'Parent',h(2));
set(t(3),'Parent',h(3));
set(t(4),'Parent',h(4));
set(t(5),'Parent',h(5));
set(t(6),'Parent',h(6));
set(t(7),'Parent',h(7));
set(t(8),'Parent',h(8));

% assign all the labels to the transformation containers
set(cs_text(1),'Parent',h(1));
set(cs_text(2),'Parent',h(2));
set(cs_text(3),'Parent',h(3));
set(cs_text(4),'Parent',h(4));
set(cs_text(5),'Parent',h(5));
set(cs_text(6),'Parent',h(6));
set(cs_text(7),'Parent',h(7));
set(cs_text(8),'Parent',h(8));

% assign some different colors
set(p(1),'FaceColor','cyan')
set(p(2),'FaceColor','green')
set(p(3),'FaceColor','red')
set(p(4),'FaceColor','magenta')
set(p(5),'FaceColor','yellow')
set(p(6),'FaceColor','cyan')
set(p(7),'FaceColor','green')

%set(p,'Visible','off')

% setup the figure how we need it
daspect([1 1 1])
axis([-.5 .5 -.5 .5 -.2 1])

% create a placeholder for a plot 'trail' that will show our end effector
tailLength = 20;
hTrail = line(nan(1,tailLength),nan(1,tailLength),nan(1,tailLength),'Color','k','Marker','.');

camlight
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')

handles.patch = p;
handles.text = cs_text;
handles.tforms = h;
handles.triad = t;
handles.trail = hTrail;
% handles.trail = []; %disable trail

