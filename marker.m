I = imread('ex1.png');
%I = imresize(I,0.3);
RGB = insertShape(I,'circle',[150 280 35],'LineWidth',5);
%RGB = insertMarker(I,[147 279]);