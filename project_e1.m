l1 = 0;
l2 = 1.5;
l3 = 0.5;
l4 = 0.5; 
l5 = 0;
l6 = 0;
dh = [0, l1, 0, -pi/2; ...
      0, 0, l2, 0;...
      0, 0, l3, 0;...
      pi/2,0,0,pi/2;...
      0,l4,0,pi/2;...
      pi/2,0,l5,-pi/2;...
      pi/2,0,0,pi/2;...
      0,l6,0,0];

angle = [-pi/4,-pi/4,pi/4,0,0,0];

figure(1);
[pose,~] = plot(dh,angle)

angle = [-pi/4,-pi/4,pi/4,pi/3,0,0];
[pose2,~] = plot(dh,angle)
% scatter3(pose2(1,:),pose2(2,:), pose2(3,:), 'filled', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'g');
% plot3(pose2(1,:),pose2(2,:), pose2(3,:), 'r-');
hold off; % Release the hold on the current plot

% Customize the plot
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
xlim([-2.5,2.5]);
ylim([-2.5,2.5]);
zlim([-2.5,2.5]);
grid on;


%% 

%%Q2
pose(:,7)
params = [l1,0,l2,l3+l4];
angles = IK(pose(:,7),params)
ee = zeros(4,4);
figure(2)
angles(6,:) = 0;

[pose,~]= plot(dh,angles(:,1)');
ee(:,1) = pose(:,7);

[pose,~]= plot(dh,angles(:,2)');
ee(:,2) = pose(:,7);

[pose,~]= plot(dh,angles(:,3)');
ee(:,3) = pose(:,7);

[pose,~]= plot(dh,angles(:,4)');
ee(:,4) = pose(:,7);
plot_sphere([0,0,0],.5,'y',0.1)
plot_sphere([0,0,0],2.5,'y',0.1)
hold off

disp(ee);

%----------------------------------------------------------------------------------------
%%
% Ball Trajectory
t = 0:0.01:1.5;
g = -9.8;
bx = 5 - 8*sin(pi/4)*t;
bz = 8*sin(pi/4)*(rem(t,1.154)) + g*(rem(t,1.154)).^2*1/2; %1.154 is the value of t at which the ball bounces
by = 0*t;

pose = [bx; by; bz];

figure(3)

prot = zeros(1,6);

for i = 1:length(t)
    if(norm(pose(:,i)) < (params(3)+params(4)))
    angles = IK(pose(:, i), params);
    angles(6, :) = 0;
    
    rot = angles(:,1);
    sec = angles(:,2);
        if i>1
            dev1 = 0;
            dev2 = 0;
            for j = 1:length(rot)
                dev1 = dev1 + abs(rot(j) - prot(j));
                dev2 = dev2 + abs(rot(j) - sec(j));
            end
            if dev1 > 3
                rot = sec;
            end
        end
    pose;rotation = plot(dh,rot');
    plot3(bx,by,bz,'k-')
    xlim([-4,6]);
    zlim([-1,4]);
    ylim([-1,1]);
    hold off
    pause(0.01)

    prot = rot;

    end
end

%%
%Velocity Kinematics
% disp("first column is angular velocity and second is linear velocity corresponding to each frame")

theta = [0,pi/4,pi/4,0,0,0];

% figure(4);
% [pose,~] = plot(dh,theta);

re_vel = [3,2,2];

theta_diff =  solver(theta(1),theta(2),theta(3),re_vel)

theta_dot = [theta_diff(1),theta_diff(2),theta_diff(3),1,1,1]; 
[R,velocities] = frame_velocity(dh,theta_dot,theta);

disp(double(R'*velocities(:,2,9))); % clearly linear velocity is independent of angles 4-6

%TODO: find theta(4)-(6) such that they generate R'R_per where R is the
%rotation currently possesed by 3R and Rperp is the angle made by the ball
%in trajectory , this will make a constant velocity of 8

% TODO: from the difference in angles of joints 4-6 from the above and the
% time step find angular velocities of 4-6.

%TODO: run solver for every point in the ball trajectory(make it a
%function and simply call it.


% Functions
%----------------------------------------------------------------------------------------
function [pose,rotation] = plot(dh,angle)

[pose,rotation] = FK(dh,angle);

% pose, double(rotation)
% R1 = SerialLink(dh,'name', 'TutorialBot')

scatter3(pose(1,:),pose(2,:), pose(3,:), 'filled', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'g');
title('3D Scatter Plot');

% Create a line connecting the points
hold on; % Keep the current plot active
plot3(pose(1,:),pose(2,:), pose(3,:), 'b-');

frame_length = 0.5; % Length of the frame axes
x_ = double(rotation*[frame_length;0;0]);
y_ = double(rotation*[0;frame_length;0]);
z_ = double(rotation*[0;0;frame_length]);
quiver3(pose(1,7),pose(2,7), pose(3,7),x_(1),x_(2), x_(3), 'r', 'LineWidth', 2); % X-axis (red)
quiver3(pose(1,7),pose(2,7), pose(3,7),y_(1),y_(2), y_(3), 'g', 'LineWidth', 2); % Y-axis (green)
quiver3(pose(1,7),pose(2,7), pose(3,7),z_(1),z_(2), z_(3), 'b', 'LineWidth', 2); % Z-axis (blue)
end

%----------------------------------------------------------------------------------------

function T_mat = transformation(DH)

len = size(DH);
len = len(1);
T_mat = sym(zeros(4,4,len));

for i = 1:len

    theta =DH(i,1);
    d     =DH(i,2);
    a     =DH(i,3);
    alpha =DH(i,4);
% cannot do double here as that causes 
    T_mat(:,:,i) =[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
                    sin(theta),cos(theta)*cos(alpha), -cos(theta)*sin(alpha),a*sin(theta);
                    0 ,sin(alpha),    cos(alpha),    d;  
                    0 ,0 ,0 ,1 ;];
end
end
%----------------------------------------------------------------------------------------
function [poses,rotation] = FK(dh,angles)

    len = size(dh,1);
    j=1;
    for i =1:len
        if(i ~= 4 && i~=6 && i ~= 7)
            dh(i,1) = angles(j);
            j = j+1;
        elseif (i == 6)
            dh(i,1) = angles(j) + pi/2;
            j = j+1;
        else
            dh(i,1) = pi/2;
        end
    end

 T_mat = transformation(dh);

 poses = zeros(4,6);
 net_transform = transformation([0,0,0,0]);
 end_pose = [0;0;0;1;];
 
 j=1;
 for i=1:len
     if(i ~= 4 && i ~= 7)
        poses(:,j) = double(net_transform*end_pose);
        j =j+1;
     end
     net_transform = net_transform*T_mat(:,:,i);
 end
 poses(:,j) = double(net_transform*end_pose);
 rotation = net_transform(1:3,1:3);
end

%-------------------------------------------------------------------------------------------

function angle_set = IK(pose,params)
% every column ofoutput is a possible solution 
l1 = params(1);
l2 = params(2);
l3 = params(3);
l4 = params(4);

x = pose(1);
y = pose(2);
z = pose(3);

costht1 = (l2 * y + sqrt(-x^2 * (l2^2 - (x^2 + y^2)))) / (x^2 + y^2);
if abs(x - 0) > 1e-9
    theta_1_1 = atan2((y * costht1 - l2) /x ,costht1);
else
    theta_1_1 =acos(costht1);
end

% Calculate the second solution
costht1 = (l2 * y - sqrt(-x^2 * (l2^2 - (x^2 + y^2)))) / (x^2 + y^2);
if abs(x - 0) > 1e-9
    theta_1_2 = atan2((y * costht1 - l2) /x ,costht1);
else
    theta_1_2 = -acos(costht1);
end


theta_3_1 = acos(((x^2 +y^2) + (z - l1)^2 - (l2^2+l3^2 + l4^2)) / (2 * l3 * l4));
theta_3_2 = -theta_3_1 ;%both may have solutions

c = z-l1;
d_1 = -l3-l4*cos(theta_3_1); 
e_1 = -l4*sin(theta_3_1); 
e_2 = -l4*sin(theta_3_2);
%d_2 = d_1

costht2 = (c * e_1 + sqrt(-d_1^2 * (c^2 - (e_1^2 + d_1^2))))/(e_1^2 + d_1^2);
theta_2_1 = atan2((c-e_1*costht2)/d_1,costht2);

costht2 = (c * e_1 - sqrt(-d_1^2 * (c^2 - (e_1^2 + d_1^2))))/(e_1^2 + d_1^2);
theta_2_2 = atan2((c-e_1*costht2)/d_1,costht2);

costht2 =(c * e_2 + sqrt(-d_1^2 * (c^2 - (e_2^2 + d_1^2))))/(e_2^2 + d_1^2);
theta_2_3 = atan2((c-e_2*costht2)/d_1,costht2);

costht2 =(c * e_2 - sqrt(-d_1^2 * (c^2 - (e_2^2 + d_1^2))))/(e_2^2 + d_1^2);
theta_2_4 = atan2((c-e_2*costht2)/d_1,costht2);


angle_set = zeros(3,4);
% every block of two has a solution between them
% order so that no two solutions which constrict each other do not occur 
%e.g. [theta_1_1, theta_2_1 ,theta_3_1],[theta_1_1, theta_2_2 ,theta_3_1] cannot both be possible unless theta_2_1 = theta theta_2_2

statement1 = (abs(x - (-l2*sin(theta_1_1) +l3*cos(theta_1_1)*cos(theta_2_1) + l4*cos(theta_1_1)*cos(theta_2_1 + theta_3_1))) < 1e-9);
statement2 = (abs(y - (l2*cos(theta_1_1) +l3*sin(theta_1_1)*cos(theta_2_1) + l4*sin(theta_1_1)*cos(theta_2_1 + theta_3_1))) < 1e-9);
if statement1 && statement2
    angle_set(:,1) = [theta_1_1, theta_2_1 ,theta_3_1];
else
    angle_set(:,1) = [theta_1_2, theta_2_1 ,theta_3_1];
end

statement1 =(abs(x - (-l2*sin(theta_1_2) +l3*cos(theta_1_2)*cos(theta_2_2) + l4*cos(theta_1_2)*cos(theta_2_2 + theta_3_1))) < 1e-9);
statement2 =(abs(y - (l2*cos(theta_1_2) +l3*sin(theta_1_2)*cos(theta_2_2) + l4*sin(theta_1_2)*cos(theta_2_2 + theta_3_1)))< 1e-9);
if statement1&& statement2
    angle_set(:,2) = [theta_1_2, theta_2_2 ,theta_3_1];
else
    angle_set(:,2) = [theta_1_1, theta_2_2 ,theta_3_1];
end

statement1 =(abs(x - (-l2*sin(theta_1_1) +l3*cos(theta_1_1)*cos(theta_2_3) + l4*cos(theta_1_1)*cos(theta_2_3 + theta_3_2))) < 1e-9);
statement2 =(abs(y - (l2*cos(theta_1_1) +l3*sin(theta_1_1)*cos(theta_2_3) + l4*sin(theta_1_1)*cos(theta_2_3 + theta_3_2)))< 1e-9);
if statement1&& statement2
    angle_set(:,3) = [theta_1_1, theta_2_3 ,theta_3_2];
else
    angle_set(:,3) = [theta_1_2, theta_2_3 ,theta_3_2];
end

statement1 =(abs(x - (-l2*sin(theta_1_2) +l3*cos(theta_1_2)*cos(theta_2_4) + l4*cos(theta_1_2)*cos(theta_2_4 + theta_3_2))) < 1e-9);
statement2 =(abs(y - (l2*cos(theta_1_2) +l3*sin(theta_1_2)*cos(theta_2_4) + l4*sin(theta_1_2)*cos(theta_2_4 + theta_3_2)))< 1e-9);
if statement1 && statement2
    angle_set(:,4) = [theta_1_2, theta_2_4 ,theta_3_2];
else
    angle_set(:,4) = [theta_1_1, theta_2_4 ,theta_3_2];
end

end

%--------------------------------------------------------------------------------------------
function [R_out,velocities] = frame_velocity(dh,theta_dot,theta_vals)

    DH = sym(dh);
    theta_dot = [theta_dot(1),theta_dot(2),theta_dot(3),0,theta_dot(4),theta_dot(5),0,theta_dot(6)];
    theta_vals = [theta_vals(1),theta_vals(2),theta_vals(3),pi/2,theta_vals(4),theta_vals(5)+pi/2,pi/2,theta_vals(6)];
    
    syms dtheta [1,8] real
    syms theta [1,8] real
    len = size(dh,1);

    for i =1:len
            DH(i,1) = theta(i);
    end
    
    T_mat = transformation(DH);
    eqns = sym(zeros(3,2,len+1));
% 3=> x,y,z ,2 => v,w , len+1 => for each joint

% assigning first row to angular velocities and second row to linear velocities
    R_out = eye;
    for i=1:len+1
        if i > 1
            R = T_mat(1:3,1:3,i-1);
            params = T_mat(1:3,4,i-1);
            v_prev = eqns(:,2,i-1);
            w_prev = eqns(:,1,i-1);
        else
            R = eye(3);
            params = zeros(3,1);
            v_prev = [0;0;0];
            w_prev = [0;0;0]; 
        end
        if(i < len+1)
        eqns(:,1,i) = R'*(w_prev) + dtheta(i)*[0;0;1];
        else
        eqns(:,1,i) = R'*(w_prev);
        end
        eqns(:,2,i) = R'*(v_prev + cross(w_prev,params));
        R_out = R_out*R;
    end
    R_out = subs(R_out,theta,theta_vals);
    velocities = subs(eqns,[dtheta,theta],[theta_dot,theta_vals]);
end 
%--------------------------------------------------------------------------------------
function Jacobian = der_jac(params)

l1 = params(1);
l2 = params(2);
l3 = params(3);
l4 = params(4);

syms theta1 theta2 theta3;

% Define the equations
x = -l2 * sin(theta1) + l3 * cos(theta1) * cos(theta2) + l4 * cos(theta1) * cos(theta2 + theta3);
y = l2 * cos(theta1) + l3 * sin(theta1) * cos(theta2) + l4 * sin(theta1) * cos(theta2 + theta3);
z = l1 - l3 * sin(theta2) - l4 * sin(theta2 + theta3);

F = [x; y; z];

% Create a vector of the three variables
theta = [theta1; theta2; theta3];

% Compute the Jacobian matrix
Jacobian = jacobian(F, theta);

% Display the Jacobian matrix
disp('Jacobian Matrix:');
disp(Jacobian);

end

function theta_diff = solver(theta1,theta2,theta3,re_vel)
    
    syms d1 d2 d3;

    v_x = re_vel(1);
    v_y = re_vel(2);
    v_z = re_vel(3);
    
    % TODO:
    % jacobian = der_jac(params)
    % replace left side of equations with the jacobian itself symbolically
    eqn1 = v_x == (- (3*cos(theta2)*sin(theta1))/2 - cos(theta2 + theta3)*sin(theta1))*d1 +(- (3*cos(theta1)*sin(theta2))/2 - sin(theta2 + theta3)*cos(theta1))*d2 + (-sin(theta2 + theta3)*cos(theta1))*d3 ;
    eqn2 = v_y == ((3*cos(theta1)*cos(theta2))/2 + cos(theta2 + theta3)*cos(theta1))*d1 + ( - sin(theta2 + theta3)*sin(theta1) - (3*sin(theta1)*sin(theta2))/2)*d2 + ( -sin(theta2 + theta3)*sin(theta1))*d3;
    eqn3 = v_z == (- cos(theta2 + theta3) - (3*cos(theta2))/2)*d2  + (-cos(theta2 + theta3))*d3;
 
    S = solve([eqn1,eqn2,eqn3],[d1 d2 d3]);
    theta_diff = [S.d1,S.d2,S.d3];
end