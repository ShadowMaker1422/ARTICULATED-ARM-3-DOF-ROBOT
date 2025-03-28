global amax vmax
t1 = vmax/amax;
global size_q_theta t3_theta t_theta % theta có m?ng l?n nh?t, th?i gian có m?ng l?n nh?t và t3 c?a m?mng l?n nh?t
%Link1
global q0_theta1 q3_theta1 q_theta1 t3_theta1 t_theta1 v_theta1 a_theta1
t3_theta1=(2*((q0_theta1+q3_theta1)/2-(amax*t1/2+q0_theta1-vmax*t1))/vmax);
t_theta1 = 0:0.1:t3_theta1;%l?y b??c h?i l?n nên m?ng ko v? 0 ???c 
lengthT_theta1 = length(t_theta1);
q_theta1 =zeros(lengthT_theta1, 1);
v_theta1 = zeros(lengthT_theta1, 1);
a_theta1 = zeros(lengthT_theta1, 1);
for i = 1:1:lengthT_theta1
    if t_theta1(i) <= t1
        q_theta1(i) = (amax / 2) * t_theta1(i)^2 + q0_theta1;
        v_theta1(i) = amax * t_theta1(i);
        a_theta1(i) = amax;
    elseif t_theta1(i) <= t3_theta1 - t1 && t_theta1(i) > t1
        q_theta1(i) = vmax * t_theta1(i) + (amax / 2) * t1^2 + q0_theta1 - vmax * t1;
        v_theta1(i) = vmax;
        a_theta1(i) = 0;
    elseif t_theta1(i) > t3_theta1 - t1 && t_theta1(i) <= t3_theta1
        q_theta1(i) = (-amax / 2) * t_theta1(i)^2 + amax * t3_theta1 * t_theta1(i) +(q3_theta1-(amax/2)*t3_theta1^2);
        v_theta1(i) = -amax * t_theta1(i) + amax * t3_theta1;
        a_theta1(i) = -amax;
    end
end
%plot(t,q_theta1);
%xlabel('Time (s)');
%ylabel('Position (q)');
%title('Position vs Time');
%grid on;

%Link2
global q0_theta2 q3_theta2 q_theta2 v_theta2 a_theta2 t_theta2 t3_theta2
t3_theta2=(2*((q0_theta2+q3_theta2)/2-(amax*t1/2+q0_theta2-vmax*t1))/vmax);
t_theta2 = 0:0.1:t3_theta2;
lengthT_theta2 = length(t_theta2);
q_theta2 =zeros(lengthT_theta2, 1);
v_theta2 = zeros(lengthT_theta2, 1);
a_theta2 = zeros(lengthT_theta2, 1);
for i = 1:1:lengthT_theta2
    if t_theta2(i) <= t1
        q_theta2(i) = (amax / 2) * t_theta2(i)^2 + q0_theta2;
        v_theta2(i) = amax * t_theta2(i);
        a_theta2(i) = amax;
    elseif t_theta2(i) <= t3_theta2 - t1 && t_theta2(i) > t1
        q_theta2(i) = vmax * t_theta2(i) + (amax / 2) * t1^2 + q0_theta2 - vmax * t1;
        v_theta2(i) = vmax;
        a_theta2(i) = 0;
    elseif t_theta2(i) > t3_theta2 - t1 &&t_theta2(i) <= t3_theta2
        q_theta2(i) = (-amax / 2) *t_theta2(i)^2 + amax * t3_theta2 *t_theta2(i) +(q3_theta2-(amax/2)*t3_theta2^2);
        v_theta2(i) = -amax * t_theta2(i) + amax * t3_theta2;
        a_theta2(i) = -amax;
    end
end
%plot(t_theta2,q_theta2);
%xlabel('Time (s)');
%ylabel('Position (q)');
%title('Position vs Time');
%grid on;

%Link3
global q0_theta3 q3_theta3 q_theta3 v_theta3 a_theta3 t_theta3 t3_theta3
t3_theta3=(2*((q0_theta3+q3_theta3)/2-(amax*t1/2+q0_theta3-vmax*t1))/vmax);
t_theta3 = 0:0.1:t3_theta3;
lengthT_theta3 = length(t_theta3);
q_theta3 =zeros(lengthT_theta3, 1);
v_theta3 = zeros(lengthT_theta3, 1);
a_theta3 = zeros(lengthT_theta3, 1);
for i = 1:1:lengthT_theta3
    if t_theta3(i) <= t1
        q_theta3(i) = (amax / 2) * t_theta3(i)^2 + q0_theta3;
        v_theta3(i) = amax * t_theta3(i);
        a_theta3(i) = amax;
    elseif t_theta3(i) <= t3_theta3 - t1 && t_theta3(i) > t1
        q_theta3(i) = vmax * t_theta3(i) + (amax / 2) * t1^2 + q0_theta3 - vmax * t1;
        v_theta3(i) = vmax;
        a_theta3(i) = 0;
    elseif t_theta3(i) > t3_theta3 - t1 &&t_theta3(i) <= t3_theta3
        q_theta3(i) = (-amax / 2) *t_theta3(i)^2 + amax * t3_theta3 *t_theta3(i) +(q3_theta3-(amax/2)*t3_theta3^2);
        v_theta3(i) = -amax * t_theta3(i) + amax * t3_theta3;
        a_theta3(i) = -amax;
    end
end
%plot(t_theta3,q_theta3);
%xlabel('Time (s)');
%ylabel('Position (q)');
%title('Position vs Time');
%grid on;

% x? lý các m?ng ?? cân b?ng nhau
% So sánh kích th??c c?a q_theta1 và q_theta2
size_q_theta1 = size(q_theta1, 1);
size_q_theta2 = size(q_theta2, 1);
size_q_theta3 = size(q_theta3, 1);
max_size = max([size_q_theta1, size_q_theta2, size_q_theta3]);
% N?u q_theta1 l?n h?n q_theta2, thêm các ph?n t? vào q_theta2
if max_size == size_q_theta1
    % S? ph?n t? c?n thêm
    num_elements_to_add_to_theta2 = size_q_theta1 - size_q_theta2;
    num_elements_to_add_to_theta3 = size_q_theta1 - size_q_theta3;
    % Ph?n t? cu?i c?a q_theta
    last_element_q_theta2 = q_theta2(end);
    last_element_v_theta2 = v_theta2(end);
    last_element_a_theta2 = a_theta2(end);
    last_element_q_theta3 = q_theta3(end);
    last_element_v_theta3 = v_theta3(end);
    last_element_a_theta3 = a_theta3(end);
    % Thêm các ph?n t? vào q_theta
    q_theta2 = [q_theta2; repmat(last_element_q_theta2, num_elements_to_add_to_theta2, 1)];
    q_theta3 = [q_theta3; repmat(last_element_q_theta3, num_elements_to_add_to_theta3, 1)];
    v_theta2 = [v_theta2; repmat(last_element_v_theta2, num_elements_to_add_to_theta2, 1)];
    v_theta3 = [v_theta3; repmat(last_element_v_theta3, num_elements_to_add_to_theta3, 1)];
    a_theta2 = [a_theta2; repmat(last_element_a_theta2, num_elements_to_add_to_theta2, 1)];
    a_theta3 = [a_theta3; repmat(last_element_a_theta3, num_elements_to_add_to_theta3, 1)];
    t_theta = t_theta1;
    t3_theta = t3_theta1;
elseif  max_size == size_q_theta2
     % S? ph?n t? c?n thêm
    num_elements_to_add_to_theta1 = size_q_theta2 - size_q_theta1;
    num_elements_to_add_to_theta3 = size_q_theta2 - size_q_theta3;
    % Ph?n t? cu?i c?a q_theta
    last_element_q_theta1 = q_theta1(end);
    last_element_q_theta3 = q_theta3(end);
    last_element_v_theta1 = v_theta1(end);
    last_element_v_theta3 = v_theta3(end);
    last_element_a_theta1 = a_theta1(end);
    last_element_a_theta3 = a_theta3(end);
    % Thêm các ph?n t? vào q_theta
    q_theta1 = [q_theta1; repmat(last_element_q_theta1, num_elements_to_add_to_theta1, 1)];
    q_theta3 = [q_theta3; repmat(last_element_q_theta3, num_elements_to_add_to_theta3, 1)];
    v_theta1 = [v_theta1; repmat(last_element_v_theta1, num_elements_to_add_to_theta1, 1)];
    v_theta3 = [v_theta3; repmat(last_element_v_theta3, num_elements_to_add_to_theta3, 1)];
    a_theta1 = [a_theta1; repmat(last_element_a_theta1, num_elements_to_add_to_theta1, 1)];
    a_theta3 = [a_theta3; repmat(last_element_a_theta3, num_elements_to_add_to_theta3, 1)];
    t_theta = t_theta2;
    t3_theta = t3_theta2;
else
    num_elements_to_add_to_theta1 = size_q_theta3 - size_q_theta1;
    num_elements_to_add_to_theta2 = size_q_theta3 - size_q_theta2;
    % Ph?n t? cu?i c?a q_theta
    last_element_q_theta1 = q_theta1(end);
    last_element_q_theta2 = q_theta2(end);
    last_element_v_theta1 = v_theta1(end);
    last_element_v_theta2 = v_theta2(end);
    last_element_a_theta1 = a_theta1(end);
    last_element_a_theta2 = a_theta2(end);
    % Thêm các ph?n t? vào q_theta
    q_theta1 = [q_theta1; repmat(last_element_q_theta1, num_elements_to_add_to_theta1, 1)];
    q_theta2 = [q_theta2; repmat(last_element_q_theta2, num_elements_to_add_to_theta2, 1)];
    v_theta1 = [v_theta1; repmat(last_element_v_theta1, num_elements_to_add_to_theta1, 1)];
    v_theta2 = [v_theta2; repmat(last_element_v_theta2, num_elements_to_add_to_theta2, 1)];
    a_theta1 = [a_theta1; repmat(last_element_a_theta1, num_elements_to_add_to_theta1, 1)];
    a_theta2 = [a_theta2; repmat(last_element_a_theta2, num_elements_to_add_to_theta2, 1)];
    t_theta = t_theta3;
    t3_theta = t3_theta3;
end
size_q_theta=  max_size ;
%plot(t_theta,a_theta1)
%grid on


