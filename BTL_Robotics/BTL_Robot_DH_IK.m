% Tinh toan dong hoc nghich
% Ve Articulated Arm Robot bang MATLAB
global PX PY PZ theta1 theta2 theta3 distance
h = 2; %d(O0,O1)

% tao gia tri bang DH
a1 = 0; alpha1 = pi/2; d1 = h; 
a2 = 7; alpha2 = 0; d2 = 0; 
a3 = 5; alpha3 = 0; d3 = 0; 

Pwx=PX;
Pwy=PY;
Pwz=PZ;
dis=sqrt(Pwx^2+Pwy^2+(Pwz-h)^2); 

    %theta3 n?m trong 0-->90 ??--> s3>0
    c3= (Pwx^2+Pwy^2+(Pwz-h)^2-a2^2-a3^2)/(2*a2*a3);
    s3=  sqrt(1-c3^2); % ta? sao không ???c ch?n s3 âm ???%%%%%%%%%%
    th3= rad2deg(atan2(s3,c3));
    
    %Tinh goc theta2
    %có 2 cách ch?n theta2: 90-->180 ho?c 0-->-90 , tr??ng h?p này ta ch?n
    %90-->180
    c2 = (-sqrt(Pwx^2+Pwy^2)*(a2 + a3*c3)+(Pwz-h)*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);
    s2 = (sqrt(Pwx^2+Pwy^2)*(a3*s3)+(Pwz-h)*(a2 + a3*c3))/(a2^2+a3^2+2*a2*a3*c3);
    th2= rad2deg(atan2(s2, c2));
    %bu do
    %Tinh goc theta2
    %c2 = (sqrt(PY^2+PZ^2)*(a2 + a3*c3)+PZ*a3*s3)/(a2^2+a3^2*a2*a3*c3);
    %s2 = (sqrt(PY^2+PZ^2)*(a3*s3)+PZ*(a2 + a3*c3))/(a2^2+a3^2*a2*a3*c3);
   % th2 =rad2deg(atan2(s2, c2));
    
    %Tinh goc theta1  
    th1 = rad2deg(atan2(-Pwy,Pwx));
    if th1>0
        th1 = 180-th1;
    elseif th1<0
        th1= -180- th1;
    else
        th1=0;
    end
% ??nh ngh?a các thông s?
%Wx = PX; 
%Wy = PY;
%Wz = PZ+h;
%distance = sqrt(Wx^2 + Wy^2 + Wz^2);
%if (abs(a2 - a3) <= distance) && (distance <= a2 + a3)
%Tinh goc theta1  
%th1 = rad2deg(atan2(PY,PX));
%Tinh goc theta3
%r =sqrt( PX^2 + PY^2 + PZ^2);
%c3 = (r^2 - a2^2 - a3^2)/(2*a2*a3);
%s3 = sqrt((1 - c3^2));
%th3 =rad2deg(atan2(s3, c3));
%Tinh goc theta2
%c2 = (sqrt(PY^2+PZ^2)*(a2 + a3*c3)+PZ*a3*s3)/(a2^2+a3^2*a2*a3*c3);
%s2 = (sqrt(PY^2+PZ^2)*(a3*s3)+PZ*(a2 + a3*c3))/(a2^2+a3^2*a2*a3*c3);
%th2 =rad2deg(atan2(s2, c2));
 % Hi?n th? k?t qu?
 theta1=th1;
 theta2=th2;
 theta3=th3;
 distance= dis;



