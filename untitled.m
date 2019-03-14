%ipaddress = '192.168.18.101';
%rosinit(ipaddress) 

rosinit('127.0.0.1')
velocity = 0.1;     % meters per second

if ismember('/bottom_kinect/rgb/image_raw', rostopic('list'))
    imsub = rossubscriber('/bottom_kinect/rgb/image_raw');
end

robot = rospublisher('/mux_vel_nav/cmd_vel') ;
velmsg = rosmessage(robot);
se = strel('line',11,90);

while 1
    velmsg.Linear.X = velocity;
    %send(robot,velmsg);
    
    img = receive(imsub)
    img_erode = imerode(readImage(img),se);
    imshow(img_erode)
    
end