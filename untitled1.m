C = pidstd(1,0.5,'Ts',0.1,'IFormula','Trapezoidal')
C=tf(C);
a = C.Numerator{1}(1);
 b=C.Numerator{1}(2);
 c=C.Denominator{1}(1);
 d=C.Denominator{1}(2);
K=12;
Ti=50;
Td=10;
 
 Ypp=0.5;
 Upp=0.5;
 x = -pi:0.01:20*pi;
 Y=sin(x)+Ypp;
 Yzad=ones(length(x),1)*Ypp;
 U=cos(x)+Upp;
 
 
 fig = figure(2)
     subplot(2,1,1);
     stairs(Y);
     hold on;
     plot(Yzad);
     hold off;
     title(['Regulator PID K=',sprintf('%g',K'),' Ti=',sprintf('%g',Ti),' Td=',sprintf('%g',Td)]);
     legend('y','yzad')
     subplot(2,1,2);
     stairs(U+Upp);
     legend('u')
     
fig.Position=[680, 558, 560, 420];
%Button
c = uicontrol(fig,'Style','pushbutton');
c.Position = [0 0 70 20];
c.String = 'Plot Data';
c.Callback = @plotButtonPushed;

%     function plotButtonPushed(src,event)
%         bar(randn(1,5));
%     end

etf = uicontrol(fig,'Style','edit');
etf.Position = [1070 0 70 20];
