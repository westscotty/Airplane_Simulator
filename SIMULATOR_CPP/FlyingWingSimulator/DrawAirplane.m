function DrawAirplane(pn, pe, pd, phi, theta, psi)

%zoom factor for the view window
zoom = 2; %times 3 zoom out

%get the airplane vertices, faces, and colors
[Vertices, Faces, Colors] = AirplaneGraphics;

%Convert from the body reference frame to the inertial frame
%first rotate, then translate
Vertices = BodyToInertialRotation(Vertices, phi, theta, psi);
Vertices = BodyToInertialTranslation(Vertices, pn, pe, pd);

% transform vertices from North-East-Down to East-North-Up 
% (for matlab rendering)
R_ENU = [...
    0, 1, 0;...
    1, 0, 0;...
    0, 0, -1;...
    ];
Vertices = (R_ENU*Vertices')';

%convert "down" pd to "height" h
h = -pd;

%draw the airplane using the "patch" command
%To learn more about the patch command, type "help patch" in the command
% window
hold on
patch('Vertices', Vertices, 'Faces', Faces,...
     'FaceVertexCData',Colors,...
     'FaceColor','flat');
title('Spacecraft')
xlabel('East (y)')
ylabel('North (x)')
zlabel('Elevation (-z)')
view(32,47)  % set the view angle for the figure
axis([pe-10*zoom,pe+10*zoom,pn-10*zoom,pn+10*zoom,h-10*zoom,h+10*zoom]); %set the axis limits
pbaspect([1 1 1]) %Set the aspect ratio
grid on
hold off

end


function [V, F, colors] = AirplaneGraphics
% [V, F, colors] = AirplaneGraphics
%
% V is a matrix of verticies (3D location of vertices)
% F is a matrix of faces that use the vertices to define the corners
% colors is a matrix of colors corresponding to the faces
%
%Define the vertices of the airplane, its faces, and its colors

% Define the vertices (physical location of vertices) (y,x,z)
V = [...
    0.9    0.9    0;... % point 1
    0.9   -0.9    0;... % point 2
    -1.   -1.2    0;... % point 3
    -1.    1.2    0;... % point 4
    1.1    1.1   -0.7;... % point 5
    1.1   -1.1   -0.7;... % point 6
    -1   -1.1   -1.;... % point 7
    -1    1.1   -1.;... % point 8
    -0.3  0.3  -1.5;... % point 9
    -0.3 -0.3  -1.5;... % point 10
    -1. -0.3   -1.5;... % point 11
    -1.  0.3   -1.5;... % point 12
    -1   0.5    0;... % point 13
    -1   -0.5   0;... % point 14
    -4   -0.5   0;... % point 15
    -4   0.5    0;... % point 16
    -1   0.5  -0.7;... % point 17
    -1   -0.5 -0.7;... % point 18
    -4   -0.5 -0.7;... % point 19
    -4   0.5  -0.7;... % point 20
    -4    2   -0.7;... % point 21
    -3   2   -0.7;... % point 22
    -3  0.5  -0.7;... % point 23
    -4   -2   -0.7;... % point 24
    -3  -2   -0.7;... % point 25
    -3  -0.5 -0.7;... % point 26
    -3  3.2  0;... % point 27
    -3  2  0;... % point 28
    -7  2  0;... % point 29
    -7  3.2  0;... % point 30
    -3  3.2  -1.5;... % point 31
    -3  2  -1.5;... % point 32
    -7  2  -1.5;... % point 33
    -7  3.2  -1.5;... % point 34
    -3  -2  0;... % point 35
    -3  -3.2  0;... % point 36
    -7  -3.2  0;... % point 37
    -7  -2  0;... % point 38
    -3  -2  -1.5;... % point 39
    -3  -3.2  -1.5;... % point 40
    -7  -3.2  -1.5;... % point 41
    -7  -2  -1.5;... % point 42
];

% define faces as a list of vertices numbered above
F = [...
        1, 2,  6,  5;...  % front
        4, 3,  7,  8;...  % back
        1, 5,  8,  4;...  % right 
        2, 6,  7,  3;...  % left
        9,10, 11, 12;...  % top
        5, 9, 12,  8;...  % slant right
        5, 6, 10,  9;...  % slant front
        6, 7, 11, 10;...  % slant left
        8,12, 11,  7;...  % rear view window
        1, 2,  3,  4;...  % bottom
        13,14,15, 16;...  % body bottom
        17,18,19, 20;...  % body top
        13,17,20, 16;...  % body right
        14,18,19, 15;...  % body left
        20,21,22, 23;...  % right wing
        19,24,25, 26;...  % left wing
        27,28,29, 30;...  % right E. bottom
        31,32,33, 34;...  % right E. top
        27,31,34, 30;...  % right E. right
        28,32,33, 29;...  % right E. left
        30,34,33, 29;...  % right E. back
        27,31,32, 28;...  % right E. front
        35,36,37, 38;...  % left E. bottom
        39,40,41, 42;...  % left E. top
        35,39,42, 38;...  % left E. right
        36,40,41, 37;...  % left E. left
        38,42,41, 37;...  % left E. back
        35,39,40, 36;...  % left E. front
        ];

% define colors for each face    
myred = [1, 0, 0];
mygrey = [.5, .5, .5];
mygreen = [0, 1, 0];
myblue = [0, 0, 1];
myyellow = [1, 1, 0];
mycyan = [0, 1, 1];

colors = [...
    myyellow;... % front
    mygrey;...   % back
    mygrey;...   % right
    mygrey;...   % left
    myblue;...   % top
    mycyan;...   % slant right
    mycyan;...   % slant front
    mycyan;...   % slant left
    myblue;...   % rear view window
    mygrey;...   % bottom
    mygrey;...   % body bottom
    mygrey;...   % body top
    mygrey;...   % body right
    mygrey;...   % body left
    mygrey;...    % right wing
    mygrey;...    % left wing
    mygrey;...   % right E. bottom
    mygrey;...   % right E. top
    mygrey;...   % right E. right
    mygrey;...   % right E. left
    mygrey;...   % right E. back
    mygrey;...   % right E. front
    mygrey;...   % left E. bottom
    mygrey;...   % left E. top
    mygrey;...   % left E. right
    mygrey;...   % left E. left
    mygrey;...   % left E. back
    mygrey;...   % left E. front
    ];
end

function RotatedPoints = BodyToInertialRotation(points, phi, theta, psi)
%RotatedPoints = BodyToInertialRotation(points, phi, theta, psi)
%
% points: location of points in the body frame that are to be
%  
    %yaw rotation matrix (about the z-axis)
    R_yaw = [cos(psi),sin(psi),0; ...
            -sin(psi),cos(psi),0; ...
               0     ,  0     ,1];
    %pitch rotation matrix (about the y-axis)
    R_pitch = [cos(theta), 0, -sin(theta); ...
                 0       , 1,    0       ; ...
               sin(theta), 0,  cos(theta)];
    %roll rotation matrix (about the x-axis)
    R_roll = [1,   0,        0     ; ...
              0, cos(phi), sin(phi); ...
              0,-sin(phi), cos(phi)];
          
    %Get the overall rotation matrix from the Inertial Frame to the body
    % frame
    R_Inertia_to_Body = R_roll*R_pitch*R_yaw;
    %Get the overall rotation matrix from the body frame to the inertial
    % frame
    R = R_Inertia_to_Body';
    
    %Rotate the points
    RotatedPoints = (R*points')';
end

function translatedPoints = BodyToInertialTranslation(points, pn, pe, pd)
translatedPoints = zeros(size(points));
translatedPoints(:,1) = points(:,1)+pn;
translatedPoints(:,2) = points(:,2)+pe;
translatedPoints(:,3) = points(:,3)+pd;
end



