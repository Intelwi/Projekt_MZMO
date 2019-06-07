%Autor: Michał Stolarz


function varargout = gui(varargin)
% GUI MATLAB code for gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui

% Last Modified by GUIDE v2.5 29-Apr-2019 14:33:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
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


% --- Executes just before gui is made visible.
function gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

%Choose default command line output for gui
handles.output = hObject;
% okres z jakim działa regulacja
handles.Ts = 0.01;
% okres z jakim odswiezane sa wykresy i obraz
handles.Tdraw = 0.2;
% wspolczynniki do regulatora PID
handles.r0 = 0;
handles.r1 = 0;
handles.r2 = 0;
% timery
handles.figureTimer = timer('StartDelay', 1, 'Period', handles.Tdraw,'ExecutionMode', 'fixedRate');
handles.PIDTimer = timer('StartDelay', 1, 'Period', handles.Ts,'ExecutionMode', 'fixedRate');
% wektory do rysowania wektorow
handles.Z = zeros(100,1);
handles.X = zeros(100,1);
handles.xzad = 340;
handles.XZad = ones(100,1)*handles.xzad;
% obiekt do odbioru obrazu z robota (inicjalizacja w obsłudze przycisku 'Start')
handles.imsub = 0; 
% obiekt do odbioru danych o predkosci robota (inicjalizacja w obsłudze przycisku 'Start')
handles.odom = 0; 
% obiekt do wysylania predkosci do robota (inicjalizacja w obsłudze przycisku 'Start')
handles.robot = 0; 
% obraz z robota;
handles.image = 0;
% uchyby
handles.E = zeros(3,1);
% limity predkosci katowych
handles.Umax = 0.5;
handles.Umin = -0.5;
% predkosc liniowa
handles.linVel = 0.2;
%przeszle sterowanie
handles.upast = 0;
% srodek wykrytej linii
handles.x = 0;
handles.y = 0;

% Update handles structure
guidata(hObject, handles);
% UIWAIT makes gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function kPid_Callback(hObject, eventdata, handles)
% hObject    handle to kPid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of kPid as text
%        str2double(get(hObject,'String')) returns contents of kPid as a double


% --- Executes during object creation, after setting all properties.
function kPid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kPid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tiPid_Callback(hObject, eventdata, handles)
% hObject    handle to tiPid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tiPid as text
%        str2double(get(hObject,'String')) returns contents of tiPid as a double


% --- Executes during object creation, after setting all properties.
function tiPid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tiPid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tdPid_Callback(hObject, eventdata, handles)
% hObject    handle to tdPid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tdPid as text
%        str2double(get(hObject,'String')) returns contents of tdPid as a double


% --- Executes during object creation, after setting all properties.
function tdPid_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tdPid (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in contoursRadioButton.
function contoursRadioButton_Callback(hObject, eventdata, handles)
% hObject    handle to contoursRadioButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of contoursRadioButton


% --- Executes on button press in pointsRadioButton.
function pointsRadioButton_Callback(hObject, eventdata, handles)
% hObject    handle to pointsRadioButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pointsRadioButton


% --- Executes on button press in startButton.
function startButton_Callback(hObject, eventdata, handles)
% hObject    handle to startButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%inicjalizacja polaczenia
    % nawiazanie polaczenia z robotem
    IP_OF_ROBOT = '192.168.18.90';
    IP_OF_HOST_COMPUTER = '192.168.18.223';
    rosinit(IP_OF_ROBOT,'NodeHost',IP_OF_HOST_COMPUTER);
    
    % inicjalizacja
    K = str2double(get(handles.kPid,'String'));
    Ti = str2double(get(handles.tiPid,'String'));
    if(Ti == 0)
        Ti = 1000000;
    end
    Td = str2double(get(handles.tdPid,'String'));
    handles.r0 = K*(1+handles.Ts /(2*Ti)+Td/handles.Ts );
    handles.r1 = K*(handles.Ts /(2*Ti)-2*Td/handles.Ts -1);
    handles.r2 = K*Td/handles.Ts ;
    % inicjalizacja odbiorcy informacji o położeniu robota
    handles.odom = rossubscriber('/gazebo_odom');
    % inicjalizacja nadawcy informacji o predkosci
    handles.robot = rospublisher('/mux_vel_keyboard/cmd_vel') ;
    % inicjalizacja odbiorcy obrazu
    if ismember('/bottom_kinect/rgb/image_raw', rostopic('list'))
        handles.imsub = rossubscriber('/bottom_kinect/rgb/image_raw');
    end
    
    handles.PIDTimer.TimerFcn = {@pid_callback_fcn,hObject};
    handles.figureTimer.TimerFcn = {@figure_callback_fcn,hObject};
    start(handles.PIDTimer);
    start(handles.figureTimer);
    guidata(hObject,handles);
    
function pid_callback_fcn(obj, event, hObject)
    
    handles = guidata(hObject);
    robot = handles.robot;
    imsub = handles.imsub;
    odom = handles.odom;
    
    velmsg = rosmessage(robot);
    
    % pobranie informacji o sposobie wykrywania linii
    radiobuttonContours = get(handles.contoursRadioButton,'Value');
    radiobuttonPoints = get(handles.pointsRadioButton,'Value');
    
    % pobranie obrazu z robota
    img = receive(imsub);
    img = readImage(img);
    img = im2double(img);
    % uciecie obrazu do interesujacego obszaru
    img = imcrop(img,[0 350 640 480]);
    % konwersja obrazu na czarno biały
    I3=rgb2gray(img);
    % binaryzacja
    I4 = imbinarize(I3);
    handles.image = I4;
    I5 = imcomplement(I4);
    
    % wyznaczenie srodka linii poprzez srodek ciezkosci obszaru
    if(radiobuttonContours)
        stats = regionprops(I5,'Centroid');
        centroids = cat(1, stats.Centroid);
        % wyznaczenie środka konturu
        handles.x = centroids(1);
        handles.y = centroids(2);
        % zapis do rysowania wykresu
        handles.X = [handles.X(2:end);handles.x];
    end
    
    % wyznaczenie srodka linii poprzez usrednienie trzech punktow n
    % linii
    if(radiobuttonPoints)
        % pobranie wielkosci obrazu
        a = size(I5);
        % p_num-1 = liczba usrednianych punktow na linii
        p_num = 4;
        % odleglosc miedzy punktami
        y_step = round(a(1)/p_num);
        % wektor przechowujacy polozenie punktow
        avgs_x = zeros(p_num-1,1);
        
        % liczenie srednich polozen punktow
        for i=1:p_num-1
            avgs_x(i) = nanmean(find(I5(y_step*i,:)));
        end
        
        % liczenie sredniej ze srednich punktow, czyli polozenie x srodka
        % linii
        handles.x = nanmean(avgs_x);
        % polozenie y srodka linii
        handles.y = 2*y_step;
        % zapis do rysowania wykresu
        handles.X = [handles.X(2:end);handles.x];
    end
    
    % pobranie polozenia robota
    odomdata = receive(odom,3);
    
    % wyluskanie predkasci katowej robota
    twist = odomdata.Twist.Twist;
    z = twist.Angular.Z;
    handles.Z = [handles.Z(2:end);z];
    
    % wyliczenie uchybu do PID
    e = handles.xzad - handles.x;
    
    % zapis uchybu
    handles.E = [e;handles.E(1:end-1)];
    
    % wyliczenie sterowania
    u = handles.r2*handles.E(3)+handles.r1*handles.E(2)+handles.r0*handles.E(1)+handles.upast;
    
    if(isnan(u))
        u=0;
    end
    
    % zapis przeszlego sterowania
    handles.upast = u;
    
    if u > handles.Umax
        u = handles.Umax;
    elseif u < handles.Umin
        u = handles.Umin;
    end
    
    % tworzenie pakietu z predkascia linowa robota
    velmsg.Linear.X = handles.linVel;
    velmsg.Angular.Z = u;
    
    % wyslanie predkosci do robota
    send(robot,velmsg);
    
    % zapis danych
    guidata(hObject,handles);
 
    
function figure_callback_fcn(obj, event, hObject)

    handles = guidata(hObject);
    % rysowanie wykresu predkosci katawej
    plot(handles.angularVel,handles.Z);
    % rysowanie wykresu wspolrzednej X srodka linii
    plot(handles.xCoord,handles.X);
    %rysowanie obrazu
    pos = [round(handles.x) round(handles.y)];
    handles.image = 255 * repmat(uint8(handles.image), 1, 1, 3);
    imshow(handles.image,'Parent',handles.cameraImage);
    viscircles(handles.cameraImage,pos,3);
    close(gcf);

% --- Executes during object deletion, before destroying properties.
function cameraImage_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to cameraImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in stopButton.
function stopButton_Callback(hObject, eventdata, handles)
% hObject    handle to stopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 %velocity publisher
    %zatrzymanie timerow
    stop(handles.figureTimer)
    stop(handles.PIDTimer)

    robot = handles.robot;
    velmsg = rosmessage(robot);

    %zatrzymanie robota
    velmsg.Linear.X = 0;
    velmsg.Angular.Z = 0;
    for i=0:1:20
        send(robot,velmsg);
    end

    % zamkniecie polaczenia z robotem
    rosshutdown;
    
