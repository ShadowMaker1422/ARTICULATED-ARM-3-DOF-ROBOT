function varargout = Monitor_1(varargin)
% MONITOR_1 MATLAB code for Monitor_1.fig
%      MONITOR_1, by itself, creates a new MONITOR_1 or raises the existing
%      singleton*.
%
%      H = MONITOR_1 returns the handle to a new MONITOR_1 or the handle to
%      the existing singleton*.
%
%      MONITOR_1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MONITOR_1.M with the given input arguments.
%
%      MONITOR_1('Property','Value',...) creates a new MONITOR_1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Monitor_1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Monitor_1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Monitor_1

% Last Modified by GUIDE v2.5 07-Dec-2024 18:35:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Monitor_1_OpeningFcn, ...
                   'gui_OutputFcn',  @Monitor_1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before Monitor_1 is made visible.
function Monitor_1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Monitor_1 (see VARARGIN)
global theta1 theta2 theta3 det_J
theta1 = 0.1; % giá tr? m?c ??nh
theta2 = 0;
theta3 = 0;
BTL_Robot_DH_FK;
set(handles.edit36, 'String', num2str(det_J));
% Choose default command line output for Monitor_1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = Monitor_1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
axes(handles.axes1);
cla
h = rotate3d;
h.Enable ='on';
hold on;
grid on;
axis equal;

% Nhap gia tri
global theta1 theta2 theta3 T3 q0_theta1 q0_theta2 q0_theta3 det_J
new_theta1 = get(handles.Slider_Theta_1,'Value');
new_theta2 = get(handles.Slider_Theta_2,'Value');
new_theta3 = get(handles.Slider_Theta_3,'Value');
old_theta1 = theta1;
old_theta2 = theta2;
old_theta3 = theta3;


steps = 1;
 
for k = 1:steps
    % Calculate intermediate theta values
    
    theta1 = old_theta1 + (new_theta1 - old_theta1) * k / steps;
    theta2 = old_theta2 + (new_theta2 - old_theta2) * k / steps;
    theta3 = old_theta3 + (new_theta3 - old_theta3) * k / steps;
    % Update display values
        %set(handles.Theta_1, 'String', num2str(theta1));
       % set(handles.Theta_2, 'String', num2str(theta2));
        %set(handles.Theta_3, 'String', num2str(theta3));
    try
        BTL_Robot_DH_FK;
        set(handles.edit36, 'String', num2str(det_J));
       
    catch ME
        errordlg(['loi in TEST2.m: ' ME.message], 'Loi');
        return;
    end
      pause(0.01);
end
    theta1 = new_theta1;
    theta2 = new_theta2;
    theta3 = new_theta3;
    set(handles.edit21, 'String', num2str(T3(1,4)));
    set(handles.edit22, 'String', num2str(T3(2,4)));
    set(handles.edit23, 'String', num2str(T3(3,4)));
    

  

% --- Executes on slider movement.
function Slider_Theta_1_Callback(hObject, eventdata, handles)
value = get(hObject, 'Value');   
% Cap nhat gia tri hien thi 
set(handles.Theta_1, 'String', num2str(value));


% --- Executes during object creation, after setting all properties.
function Slider_Theta_1_CreateFcn(hObject, eventdata, handles)
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function Slider_Theta_2_Callback(hObject, eventdata, handles)
value = get(hObject, 'Value');   
% Cap nhat gia tri hien thi 
set(handles.Theta_2, 'String', num2str(value));

% --- Executes during object creation, after setting all properties.
function Slider_Theta_2_CreateFcn(hObject, eventdata, handles)
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function Slider_Theta_3_Callback(hObject, eventdata, handles)
value = get(hObject, 'Value');   
% Cap nhat gia tri hien thi 
set(handles.Theta_3, 'String', num2str(value));

% --- Executes during object creation, after setting all properties.
function Slider_Theta_3_CreateFcn(hObject, eventdata, handles)
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
global PX PY PZ theta1 theta2 theta3 distance q3_theta2 q3_theta1 q3_theta3 T3 det_J
axes(handles.axes1);
PX = handles.PX;
PY = handles.PY;
PZ = handles.PZ;

% Tính các ?i?m c?a qu? ??o
%PVAx;
%PVAy;

%distance
%set(handles.edit20, 'String', num2str(distance));
b = rotate3d;
b.Enable ='on';
axes(handles.axes1);
cla

    
old_theta1 = get(handles.Slider_Theta_1,'Value');
old_theta2 = get(handles.Slider_Theta_2,'Value');
old_theta3 = get(handles.Slider_Theta_3,'Value');
BTL_Robot_DH_IK;
% disp(theta1)
 %disp(theta2)
% disp(theta3)
if  90<=theta2 && theta2<=180 && 0<=theta3 && theta3<=90 && -180<=theta1 && theta1<=180
    set(handles.edit20, 'String', num2str(distance));
    set(handles.Slider_Theta_1, 'Value', theta1);
    set(handles.Slider_Theta_2, 'Value', theta2);
    set(handles.Slider_Theta_3, 'Value', theta3);
    new_theta1 = get(handles.Slider_Theta_1,'Value');
    new_theta2 = get(handles.Slider_Theta_2,'Value');
    new_theta3 = get(handles.Slider_Theta_3,'Value');
else
    error('OUT OF WORKSPACE OF ROBOT!');
    return;
end
%code ? ?ây
steps = 1;
try 
for k = 1:steps
    % Calculate intermediate theta values
    theta1 = old_theta1 + (new_theta1 - old_theta1) * k / steps;
    theta2 = old_theta2 + (new_theta2 - old_theta2) * k / steps;
    theta3 = old_theta3 + (new_theta3 - old_theta3) * k / steps;
    % Update display values
        set(handles.Theta_1, 'String', num2str(theta1));
        set(handles.Theta_2, 'String', num2str(theta2));
        set(handles.Theta_3, 'String', num2str(theta3));
    try
        BTL_Robot_DH_FK;
        set(handles.edit36, 'String', num2str(det_J));
    catch ME
        errordlg(['loi in TEST2.m: ' ME.message], 'Loi');
        return;
    end
      pause(0.01);
end

    
catch ME
    errordlg(['loi in TEST2.m: ' ME.message], 'Loi');
        return;
end
set(handles.edit24, 'String', num2str(theta1));
set(handles.edit25, 'String', num2str(theta2));
set(handles.edit26, 'String', num2str(theta3));
set(handles.edit21, 'String', num2str(T3(1,4)));
set(handles.edit22, 'String', num2str(T3(2,4)));
set(handles.edit23, 'String', num2str(T3(3,4)));

q3_theta1= theta1;
q3_theta2=theta2;
q3_theta3=theta3;
%PVA;
    theta1 = new_theta1;
    theta2 = new_theta2;
    theta3 = new_theta3;





% --- Executes when edit15 is updated.
function edit15_Callback(hObject, eventdata, handles)
px = str2double(get(hObject,'String'));
handles.PX = px;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when edit17 is updated.
function edit17_Callback(hObject, eventdata, handles)
py = str2double(get(hObject,'String'));
handles.PY = py;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when edit18 is updated.
function edit18_Callback(hObject, eventdata, handles)
pz = str2double(get(hObject,'String'));
handles.PZ = pz;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit18_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when edit20 is updated.
function edit20_Callback(hObject, eventdata, handles)
% Placeholder function

% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double


% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit23_Callback(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit23 as text
%        str2double(get(hObject,'String')) returns contents of edit23 as a double


% --- Executes during object creation, after setting all properties.
function edit23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit24_Callback(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit24 as text
%        str2double(get(hObject,'String')) returns contents of edit24 as a double


% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit25_Callback(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit25 as text
%        str2double(get(hObject,'String')) returns contents of edit25 as a double


% --- Executes during object creation, after setting all properties.
function edit25_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit26_Callback(hObject, eventdata, handles)
% hObject    handle to edit26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit26 as text
%        str2double(get(hObject,'String')) returns contents of edit26 as a double


% --- Executes during object creation, after setting all properties.
function edit26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
global theta1 q_theta1 t3 theta2 theta3 
PVA;
for i= 1:size(q_theta1,1)
    theta1=q_theta1(i);
    theta2=0;
    theta3=0;
    BTL_Robot_DH_FK;
    pause(t3/(size(q_theta1,1)-1));
end
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit27_Callback(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit27 as text
%        str2double(get(hObject,'String')) returns contents of edit27 as a double


% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
global amax vmax
global theta1  q_theta1 v_theta1 a_theta1 t3_theta1 t_theta1 q0_theta1 q3_theta1
global theta2  q_theta2 v_theta2 a_theta2 t3_theta2 t_theta2 q0_theta2 q3_theta2
global theta3  q_theta3 v_theta3 a_theta3 t3_theta3 t_theta3 q0_theta3 q3_theta3
global size_q_theta t_theta t3_theta 
global T3
global det_J
global x_positions y_positions z_positions
amax= get(handles.edit28,'String');
amax= str2double(amax);
vmax= get(handles.edit29,'String');
vmax= str2double(vmax);
    q0_theta1= get(handles.edit30,'String');
    q0_theta1= str2double(q0_theta1);
    q0_theta2= get(handles.edit31,'String');
    q0_theta2= str2double(q0_theta2);
    q0_theta3= get(handles.edit32,'String');
    q0_theta3= str2double(q0_theta3);
    
    q3_theta1= get(handles.edit33,'String');
    q3_theta1= str2double(q3_theta1);
    q3_theta2= get(handles.edit34,'String');
    q3_theta2= str2double(q3_theta2);
    q3_theta3= get(handles.edit35,'String');
    q3_theta3= str2double(q3_theta3);
PVA;

% v? ???ng ?i robot
    x_positions = [];
    y_positions = [];
    z_positions = [];
for i= 1: size_q_theta
    theta1=q_theta1(i);
    theta2=q_theta2(i);
    theta3=q_theta3(i);

    BTL_Robot_DH_FK;
    %t?a ?? x,y,z hi?n t?i 
    x=T3(1,4);
    y=T3(2,4);
    z=T3(3,4);
   x_positions=[x_positions,x];
   y_positions=[y_positions,y];
   z_positions=[z_positions,z];
   set(handles.edit36, 'String', num2str(det_J));
    pause(t3_theta/(size_q_theta-1));
end


axes(handles.axes3);
plot(t_theta, q_theta1);
title('Plot of q\_theta1 vs Time');
grid on;
axes(handles.axes4);
plot(t_theta, q_theta2);
title('Plot of q\_theta2 vs Time');
grid on;
axes(handles.axes5);
plot(t_theta, q_theta3);
xlabel('Time (t)');
title('Plot of q\_theta3 vs Time');
grid on;
axes(handles.axes6);
plot(t_theta, v_theta1);
title('Plot of v\_theta3 vs Time');
grid on;
axes(handles.axes7);
plot(t_theta, v_theta2);
title('Plot of v\_theta2 vs Time');
grid on;
axes(handles.axes8);
plot(t_theta, v_theta3);
xlabel('Time (t)');
title('Plot of v\_theta3 vs Time');
grid on;
axes(handles.axes9);
plot(t_theta, a_theta1);
title('Plot of a\_theta1 vs Time');
grid on;
axes(handles.axes10);
plot(t_theta, a_theta2);
title('Plot of a\_theta2 vs Time');
grid on;
axes(handles.axes11);
plot(t_theta, a_theta3);
xlabel('Time (t)');
title('Plot of a\_theta3 vs Time');
grid on;
axes(handles.axes1);

% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
%v? ???ng ?i
global x_positions y_positions z_positions
axes(handles.axes1)
plot3(x_positions, y_positions, z_positions, 'LineWidth', 2);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1



function edit28_Callback(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit28 as text
%        str2double(get(hObject,'String')) returns contents of edit28 as a double


% --- Executes during object creation, after setting all properties.
function edit28_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit29_Callback(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit29 as text
%        str2double(get(hObject,'String')) returns contents of edit29 as a double


% --- Executes during object creation, after setting all properties.
function edit29_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit30_Callback(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit30 as text
%        str2double(get(hObject,'String')) returns contents of edit30 as a double


% --- Executes during object creation, after setting all properties.
function edit30_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit31_Callback(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit31 as text
%        str2double(get(hObject,'String')) returns contents of edit31 as a double


% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit32_Callback(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit32 as text
%        str2double(get(hObject,'String')) returns contents of edit32 as a double


% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit33_Callback(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit33 as text
%        str2double(get(hObject,'String')) returns contents of edit33 as a double


% --- Executes during object creation, after setting all properties.
function edit33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit34_Callback(hObject, eventdata, handles)
% hObject    handle to edit34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit34 as text
%        str2double(get(hObject,'String')) returns contents of edit34 as a double


% --- Executes during object creation, after setting all properties.
function edit34_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit35_Callback(hObject, eventdata, handles)
% hObject    handle to edit35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit35 as text
%        str2double(get(hObject,'String')) returns contents of edit35 as a double


% --- Executes during object creation, after setting all properties.
function edit35_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit36_Callback(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit36 as text
%        str2double(get(hObject,'String')) returns contents of edit36 as a double


% --- Executes during object creation, after setting all properties.
function edit36_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
