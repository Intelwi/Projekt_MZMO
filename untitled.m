% IP_OF_TURTLEBOT = '192.168.18.90'
% IP_OF_HOST_COMPUTER = '192.168.18.223'
% rosinit(IP_OF_TURTLEBOT,'NodeHost',IP_OF_HOST_COMPUTER);
% velocity = 0.1;     % meters per second
% 
% if ismember('/bottom_kinect/rgb/image_raw', rostopic('list'))
%     imsub = rossubscriber('/bottom_kinect/rgb/image_raw');
% end
% 
% robot = rospublisher('/mux_vel_keyboard/cmd_vel') ;
% velmsg = rosmessage(robot);
% se = strel('line',11,90);
% 
% while 1
%     velmsg.Linear.X = velocity;
%     send(robot,velmsg);
%     
%     img = receive(imsub)
%     img_erode = imerode(readImage(img),se);
%     imshow(img_erode)
%     
% end

 % timer and gui data
f = figure;
data.numberOfClicks = 0; 
data.timer_call = 0;
guidata(f,data)
f.ButtonDownFcn = @My_Callback;

t = timer('StartDelay', 4, 'Period', 1,'ExecutionMode', 'fixedRate');
t.TimerFcn = {@my_callback_fcn, f, data};
 start(t)


function My_Callback(src,event)
data = guidata(src);
data.numberOfClicks = data.numberOfClicks + 1;
guidata(src,data)
data
end

 
function my_callback_fcn(obj, event, src, data)
    data = guidata(src);
    disp(data.timer_call)
    data.timer_call = data.timer_call + 1;
    guidata(src,data)   
end