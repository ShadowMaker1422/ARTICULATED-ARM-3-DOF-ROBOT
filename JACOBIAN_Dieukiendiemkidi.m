syms theta1 theta2 theta3
h=2;
a1 = 0; alpha1 = pi/2; d1 = h; 
a2 = 7; alpha2 = 0; d2 = 0; 
a3 = 5; alpha3 = 0; d3 = 0; 

A1 = dh_matrix(a1, alpha1, d1, theta1);
A2 = dh_matrix(a2, alpha2, d2, theta2);
A3 = dh_matrix(a3, alpha3, d3, theta3);
T1 = A1;
T2 = T1 * A2;
T3 = T2 * A3;

Z0 = [0; 0; 1];
Z1 = T1(1:3, 3);
Z2 = T2(1:3, 3);

J0 = [-T3(2,4);
      T3(1,4);
      0];

J1 = [T1(2,3)*(T3(3,4)-T1(3,4))-T1(3,3)*(T3(2,4)-T1(2,4));
      T1(3,3)*(T3(1,4)-T1(1,4))-T1(1,3)*(T3(3,4)-T1(3,4));
      T1(1,3)*(T3(2,4)-T1(2,4))-T1(2,3)*(T3(1,4)-T1(1,4))];

J2 = [T2(2,3)*(T3(3,4)-T2(3,4))-T2(3,3)*(T3(2,4)-T2(2,4));
      T2(3,3)*(T3(1,4)-T2(1,4))-T2(1,3)*(T3(3,4)-T2(3,4));
      T2(1,3)*(T3(2,4)-T2(2,4))-T2(2,3)*(T3(1,4)-T2(1,4))];

%J= [J0,J1,J2;
 %   Z0,Z1,Z2]
%J_new = [J0, J1, J2]
det_J = det(J_new);
det_J_simplified = simplify(det_J);
det_J_simplified = vpa(det_J_simplified,4);
%disp(det_J_simplified)
%disp(J0);disp(J1);disp(J2);
eqn = det_J_simplified == 0;
solutions = solve(det_J == 0, [theta1, theta2, theta3], 'Real', true);
%disp(solutions)
solutions_theta1 = vpa(solutions.theta1);
solutions_theta2 = vpa(solutions.theta2);
solutions_theta3 = vpa(solutions.theta3);


function A = dh_matrix(a, alpha, d, theta)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
         sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0 sin(alpha) cos(alpha) d;
         0 0 0 1];
end