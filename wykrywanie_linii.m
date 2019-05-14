% I= imread('ex1.png');
% size(I);
% I2 = imcrop(I,[0 120 640 270]);
% I3=rgb2gray(I2);
% I4 = imbinarize(I3);
% %[C,h] = imcontour(I4);
% %m = moment(C,2);
% I5 = imcomplement(I4);
% stats = regionprops(I5,'Centroid');
% centroids = cat(1, stats.Centroid);
% figure(1)
% imshow(I5)
% hold on
% plot(centroids(1), centroids(2), 'b*')
% hold off

I= imread('ex1.png');
I = im2double(I);
I = imcrop(I,[0 120 640 270]);
a = size(I);
I = rgb2gray(I);
I = imbinarize(I);
I = imcomplement(I);
p_num = 4;
y_step = round(a(1)/p_num);
avgs_x = zeros(p_num-1,1);

for i=1:p_num-1
    avgs_x(i) = mean(find(I(y_step*i,:)));
end

avg_x = mean(avgs_x);
y = 2*y_step;

figure(1);
imshow(I);
hold on;
plot(avg_x,y,'b*');
hold off;