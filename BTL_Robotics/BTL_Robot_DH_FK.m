% Ve Articulated Arm Robot bang MATLAB
global theta1 theta2 theta3 PX PY PZ T3 det_J
cla %Xoa trang thai truoc
% Thiet lap goc theta
h = rotate3d; 
h.Enable = 'on'% cho phép xoay h? t?a ?? Oxyz
hold on;
grid on;
axis equal;
% Tao hình tru cho khop
[X,Y,Z] = cylinder(1); %Ban kinh truc base 
h = 2; %d(O0,O1)
Z = Z * h;
surf(X, Y, Z); % Tao hinh tru truc base



% tao gia tri bang DH
a1 = 0; alpha1 = pi/2; d1 = h; 
a2 = 7; alpha2 = 0; d2 = 0; 
a3 = 5; alpha3 = 0; d3 = 0; 

% Tính các ma tr?n bi?n ??i thu?n nh?t
A1 = dh_matrix(a1, alpha1, d1, theta1);
A2 = dh_matrix(a2, alpha2, d2, theta2);
A3 = dh_matrix(a3, alpha3, d3, theta3);

% Tính các ma tr?n bi?n ??i tích l?y
T1 = A1;
T2 = T1 * A2;
T3 = T2 * A3;

%ma tr?n JACOBIAN//////////////////////////////////////////////////////
Z0 = [0; 0; 1];
Z1 = T1(1:3, 3);
Z2 = T2(1:3, 3);
J0=[-T3(2,4);
    T3(1,4);
    0];

J1=[T1(2,3)*(T3(3,4)-T1(3,4))-T1(3,3)*(T3(2,4)-T1(2,4));
    T1(3,3)*(T3(1,4)-T1(1,4))-T1(1,3)*(T3(3,4)-T1(3,4));
    T1(1,3)*(T3(2,4)-T1(2,4))-T1(2,3)*(T3(1,4)-T1(1,4))];

J2=[T2(2,3)*(T3(3,4)-T2(3,4))-T2(3,3)*(T3(2,4)-T2(2,4));
    T2(3,3)*(T3(1,4)-T2(1,4))-T2(1,3)*(T3(3,4)-T2(3,4));
    T2(1,3)*(T3(2,4)-T2(2,4))-T2(2,3)*(T3(1,4)-T2(1,4))];

J = [J0, J1, J2;
     Z0, Z1, Z2];
%Vì RRP ch? xét chuy?n ??ng trong 3 không gian x, y, z
%? Ch? c?n quan tâm ?i?u khi?n v?n t?c trên 3 không gian x, y, z ? rút g?n ma tr?n Jacobian thành:
J_new= [J0,J1,J2];
det_J= det(J_new);
%disp(J_new)
%if det_J==0
  %     errordlg('Giá tr? c?a x không ???c âm', 'L?i');
%end

  
%//////////////////////////////////////////////////////////////////////



% V? các link và kh?p v?i ?? dày l?n h?n
plot3([0 T1(1,4)], [0 T1(2,4)], [0 T1(3,4)], 'LineWidth', 10, 'Color', 'r');
plot3([T1(1,4) T2(1,4)], [T1(2,4) T2(2,4)], [T1(3,4) T2(3,4)], 'LineWidth', 20, 'Color', 'g');
plot3([T2(1,4) T3(1,4)], [T2(2,4) T3(2,4)], [T2(3,4) T3(3,4)], 'LineWidth', 20, 'Color', 'b');

% V? các kh?p v?i kích th??c l?n h?n
scatter3(0, 0, 0, 200, 'k', 'filled');
scatter3(T1(1,4), T1(2,4), T1(3,4), 200, 'k', 'filled');
scatter3(T2(1,4), T2(2,4), T2(3,4), 200, 'k', 'filled');
scatter3(T3(1,4), T3(2,4), T3(3,4), 200, 'k', 'filled');

% Thi?t l?p view và labels
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Articulated Arm Robot');
axis([-15 15 -15 15 -20 20]);

% Hàm t?o ma tr?n bi?n ??i thu?n nh?t
function A = dh_matrix(a, alpha, d, theta)
    A = [cosd(theta) -sind(theta)*cos(alpha) sind(theta)*sin(alpha) a*cosd(theta);
         sind(theta) cosd(theta)*cos(alpha) -cosd(theta)*sin(alpha) a*sind(theta);
         0 sin(alpha) cos(alpha) d;
         0 0 0 1];
end