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

% Choose default command line output for gui
handles.output = hObject;

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

    % nawaiznie polaczenia z robotem
    IP_OF_ROBOT = '192.168.18.101';
    IP_OF_HOST_COMPUTER = '192.168.18.223';
    rosinit(IP_OF_ROBOT,'NodeHost',IP_OF_HOST_COMPUTER);
    
    % pobranie obrazu z robota
    if ismember('/bottom_kinect/rgb/image_raw', rostopic('list'))
        imsub = rossubscriber('/bottom_kinect/rgb/image_raw');
    end
    
    % wektory do rysowania wektorow
    Z = zeros(100,1);
    X = zeros(100,1);
    
    % inicjalizacja odbiorcy informacji o położeniu robota
    odom = rossubscriber('/gazebo_odom');
    
    % inicjalizacja nadawcy informacji o predkosci
    robot = rospublisher('/mux_vel_keyboard/cmd_vel') ;
    velmsg = rosmessage(robot);
    
    % nastawy do regulatora PID
    K = str2double(get(handles.kPid,'String'));
    Ti = str2double(get(handles.tiPid,'String'));
    Td = str2double(get(handles.tdPid,'String'));
    
    % pobranie informacji o sposobie wykrywania linii
    radiobuttonContours = get(handles.contoursRadioButton,'Value');
    radiobuttonPoints = get(handles.pointsRadioButton,'Value');
    
    % czas probkowania
    T = 0.01;
    
    % zadana pozycja srodka linii na obrazie
    xzad = 340;
    XZad = ones(100,1)*xzad;
    
    % regulator PID
    r0 = K*(1+T/(2*Ti)+Td/T);
    r1 = K*(T/(2*Ti)-2*Td/T-1);
    r2 = K*Td/T;
%     C = pidstd(K,Ti,Td,'Ts',T,'IFormula','Trapezoidal');
%     C = tf(C);
% 
%     if(Td ~= 0)
%         a1 = C.Numerator{1}(1);
%         b1 = C.Numerator{1}(2);
%         c1 = C.Numerator{1}(3);
%         d1 = C.Denominator{1}(2);
%         e1 = C.Denominator{1}(3);
%     else
%         a1 = C.Numerator{1}(1);
%         b1 = C.Numerator{1}(2);
%         c1 = C.Denominator{1}(1);
%         d1 = C.Denominator{1}(2);
%     end
    
    % uchyby
    E = zeros(3,1);
    
    % predkosc z poprzedniej iteracji
    upast = 0;
    
    % limity predkosci katawych
    Umax = 0.5;
    Umin = -0.5;
    
    % predkosc liniowa
    linVel = 0.2;
    
    % poczatek petli sterujacej
    while 1
        % pobranie obrazu z robota
        img = receive(imsub);
        img = readImage(img);
        
        % uciecie obrazu do interesujacego obszaru
        img = imcrop(img,[0 350 640 480]);
        % konwersja obrazu na czaarno biały
        I3=rgb2gray(img);
        % binaryzacja
        I4 = imbinarize(I3);
        % zmiana białe na czarne a czarne na białe
        I5 = imcomplement(I4);
        
        % wyznaczenie srodka linii poprzez srodek ciezkosci obszru
        if(radiobuttonContours)
            stats = regionprops(I5,'Centroid');
            centroids = cat(1, stats.Centroid);
            % wyznaczenie środka konturu 
            x = centroids(1);
            y = centroids(2);
            % zapis do rysowania wykresu
            X = [X(2:end);x];
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
                %Jak find nic nie znajdzie to zwraca Nan
                %trzeba użyć nanmean()
                avgs_x(i) = nanmean(find(I5(y_step*i,:)));
            end
            
            % liczenie sredniej ze srednich punktow, czyli polozenie x srodka
            % linii
            x = nanmean(avgs_x)
            % polozenie y srodka linii
            y = 2*y_step;
            % zapis do rysowania wykresu
            X = [X(2:end);x];
        end
        
        % pobranie polozenia robota
        odomdata = receive(odom,3);
        
        % wyluskanie predkasci katowej robota
        twist = odomdata.Twist.Twist;
        z = twist.Angular.Z;
        Z = [Z(2:end);z];
        
        % rysowanie wykresu predkosci katawej
        plot(handles.angularVel,Z);
        
        % rysowanie wykresu wspolrzednej X srodka linii
        axes(handles.xCoord)
        plot(X);
        hold on;
        plot(XZad);
        hold off;
        
        % rysowanie obrazu z robota
        axes(handles.cameraImage);
        imshow(I4);
        hold on
        plot(x, y, 'b*')
        hold off
        
        % wyliczenie uchybu do PID
        e = xzad - x;
        
        % zapis uchybu
        E = [e;E(1:end-1)];
       
        % wyliczenie sterowania
        u = r2*E(3)+r1*E(2)+r0*E(1)+upast;
%         if(Td~=0)
%             u = (c1*E(3) + b1*E(2) + a1*E(1) - e1*upast)/d1;
%         else
%             u = (a1*E(1) + b1*E(2) - d1*upast)/c1;
%         end
        
        if(isnan(u))
            u
            u=0;
        end
        
        % zapis przeszlego sterowania
        %upast = u;
        
        if u > Umax
            u = Umax;
        elseif u < Umin
            u = Umin;
        end
        
        % tworzenie pakietu z predkascia linowa robota
        velmsg.Linear.X = linVel;
        velmsg.Angular.Z = u;
        
        % wyslanie predkosci do robota
        send(robot,velmsg);
        
        % czekanie żeby był okres próbkowania
        pause(T);
    end


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
    robot = rospublisher('/mux_vel_keyboard/cmd_vel') ;
    velmsg = rosmessage(robot);
    velmsg.Linear.X = 0;
    velmsg.Angular.Z = 0;
    for i=0:1:20
        send(robot,velmsg);
    end
    rosshutdown;
