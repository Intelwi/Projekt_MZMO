IP_OF_TURTLEBOT = '192.168.18.90'
IP_OF_HOST_COMPUTER = '192.168.18.223'
rosinit(IP_OF_TURTLEBOT,'NodeHost',IP_OF_HOST_COMPUTER);
velocity = 0.1;     % meters per second

if ismember('/bottom_kinect/rgb/image_raw', rostopic('list'))
    imsub = rossubscriber('/bottom_kinect/rgb/image_raw');
end

robot = rospublisher('/mux_vel_keyboard/cmd_vel') ;
velmsg = rosmessage(robot);
se = strel('line',11,90);

while 1
    velmsg.Linear.X = velocity;
    send(robot,velmsg);
    
    img = receive(imsub)
    img_erode = imerode(readImage(img),se);
    imshow(img_erode)
    
end