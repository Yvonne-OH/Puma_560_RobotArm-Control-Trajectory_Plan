function plot_data(robot,joint_q,joint_qd,joint_qdd,Tc,DH_points)
    % Tc is the trajectory planning in Cartesian coordinate system, 
    % a list filled with SE3, assuming n points, size=(1xn)
    % robot_q is the angle solved by inverse kinematics under the Cartesian coordinate system planning, 
    % joint_q is the trajectory planning by angle
    s1=size(joint_q,1);
    s2=size(joint_qd,1);
    s3=size(joint_qdd,1);

    s4=size(Tc,3);

    if s1 ~= s2 || s2 ~= s3  || s3 ~= s4 || s1==0
        disp('size err');
    end
    

    % The plane is divided into a total of 2*4=8 subdrawing intervals, two rows, four in each row
    % The position of the first subdrawing in the first row.
        subplot(2,4,1);
        i = 1:4;
        plot(joint_q(:,i)); 
        
        

        
        legend('j1','j2','j3','j4');
        grid on;
        title('Position');
        

    % Draw speed information in the first row, 2nd subplot.
        subplot(2,4,2);
        i = 1:4;
        plot(joint_qd(:,i));grid on;
        title('Velocity');
        legend('j1','j2','j3','j4');
        

    % Draw acceleration information in the first subplot of the second row.
        subplot(2,4,5);
        i = 1:4;
        plot(joint_qdd(:,i));grid on;
        title('Acceleration');
        legend('j1','j2','j3','j4');
        
            
    % Extracts the shift variable in the flush rotation matrix, which corresponds to the 
    % position of the point in the Cartesian coordinate system.
    Tjtraj=zeros(size(Tc,3),3);
    for i =1:size(Tc,3)
        Tc_one=Tc(:,:,i);
        Tjtraj(i,1)=Tc_one(1,4);
        Tjtraj(i,2)=Tc_one(2,4);
        Tjtraj(i,3)=Tc_one(3,4);
    end
    
    %Tjtraj is a matrix with n rows and 3 columns (x y z)
    
    
    %Draw p1 to p2 linear trajectory in the 2nd subgraph of the 2nd row.
    subplot(2,4,6);
    plot2(Tjtraj,'r');grid on;
    title('Trajectory from T0 to Tf');
    
    % in the first row of three or four sub diagrams and the second row of three or four sub diagrams, 
    % which corresponds to the right half of the whole drawing
     subplot(2,4,[3,4,7,8]);
     
     % Draw the trajectory line
     plot2(Tjtraj,'b');
     hold on;
     
     
     % Draw the points that must be passed
     via_points=zeros(size(DH_points,3),3);
     
     for i = 1:size(DH_points,3)
        via_points(i,1)=DH_points(1,4,i);
        via_points(i,2)=DH_points(2,4,i);
        via_points(i,3)=DH_points(3,4,i);
        
     end
     plot2(via_points,'r>');
     
     hold on;
     % Draw the trajectory line
     plot2(Tjtraj,'b');
     hold on;
     %plotcube([0.5 1 0.4],[ -0.8 -0.6 -1],.8,[1 0 0]);
     %plotcube([0.5 1 0.7],[ 0.4 -0.6 -1],.8,[0 1 0]);
     hold on;
     zlim([-1 0.5])
     robot.plot(joint_q,'trail','.r','tile1color',[0 255 255],'tile2color',[255 251 0])
     
 
end


function plotcube(varargin)
% PLOTCUBE - Display a 3D-cube in the current axes
%
%   PLOTCUBE(EDGES,ORIGIN,ALPHA,COLOR) displays a 3D-cube in the current axes
%   with the following properties:
%   * EDGES : 3-elements vector that defines the length of cube edges
%   * ORIGIN: 3-elements vector that defines the start point of the cube
%   * ALPHA : scalar that defines the transparency of the cube faces (from 0
%             to 1)
%   * COLOR : 3-elements vector that defines the faces color of the cube
%
% Example:
%   >> plotcube([5 5 5],[ 2  2  2],.8,[1 0 0]);
%   >> plotcube([5 5 5],[10 10 10],.8,[0 1 0]);
%   >> plotcube([5 5 5],[20 20 20],.8,[0 0 1]);

% Default input arguments
inArgs = { ...
  [10 56 100] , ... % Default edge sizes (x,y and z)
  [10 10  10] , ... % Default coordinates of the origin point of the cube
  .7          , ... % Default alpha value for the cube's faces
  [1 0 0]       ... % Default Color for the cube
  };

% Replace default input arguments by input values
inArgs(1:nargin) = varargin;

% Create all variables
[edges,origin,alpha,clr] = deal(inArgs{:});

XYZ = { ...
  [0 0 0 0]  [0 0 1 1]  [0 1 1 0] ; ...
  [1 1 1 1]  [0 0 1 1]  [0 1 1 0] ; ...
  [0 1 1 0]  [0 0 0 0]  [0 0 1 1] ; ...
  [0 1 1 0]  [1 1 1 1]  [0 0 1 1] ; ...
  [0 1 1 0]  [0 0 1 1]  [0 0 0 0] ; ...
  [0 1 1 0]  [0 0 1 1]  [1 1 1 1]   ...
  };

XYZ = mat2cell(...
  cellfun( @(x,y,z) x*y+z , ...
    XYZ , ...
    repmat(mat2cell(edges,1,[1 1 1]),6,1) , ...
    repmat(mat2cell(origin,1,[1 1 1]),6,1) , ...
    'UniformOutput',false), ...
  6,[1 1 1]);


cellfun(@patch,XYZ{1},XYZ{2},XYZ{3},...
  repmat({clr},6,1),...
  repmat({'FaceAlpha'},6,1),...
  repmat({alpha},6,1)...
  );

view(3);
end
