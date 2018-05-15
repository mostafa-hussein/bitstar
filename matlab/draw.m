close all;
clear;
clc;
% 
% x1=load('sol.txt1');
% figure 
% plot(x1(:,1),x1(:,2))
% set(gca,'fontsize',20)
% set(findall(gca, 'Type', 'Line'),'LineWidth',3);
% axis([0 3500 0 6000]);
% 
% x2=load('sol.txt2');
% figure 
% plot(x2(:,1),x2(:,2))
% % axis([500 2000 2000 4000]);
% 
% set(gca,'fontsize',20)
% set(findall(gca, 'Type', 'Line'),'LineWidth',3);
% 
% x3=load('sol.txt3');
% figure 
% plot(x3(:,1),x3(:,2))
% set(gca,'fontsize',20)
% set(findall(gca, 'Type', 'Line'),'LineWidth',6);
% axis([500 2000 2000 4000]);
% 
% x4=load('sol.txt4');
% figure 
% plot(x4(:,1),x4(:,2))
% set(gca,'fontsize',20)
% set(findall(gca, 'Type', 'Line'),'LineWidth',3);
% axis([500 2000 2000 4000]);


data= load('total3');
count=1;
for i=1:2:size(data)
   cost(count,1)=data(i); 
   time(count,1)=data(i+1);
   count = count+1;
end
clear data
data(:,1)=time/1000;
data(:,2)=cost;

[~,idx] = sort(data(:,1));
newdata = data(idx,:);


x=floor(newdata(1,1));
cost=0;
time=0;
count=1;
c=0;
clear data
for i=1:size(newdata)
   if( floor(newdata(i,1))==x)
      cost= cost +newdata(i,2);
      time= time +newdata(i,1);
      c=c+1;
   else
       c
       data(count,1)=time/c;
       data(count,2)=cost;
       count = count +1;
       plot(time/c,cost,'*')
       hold on;
       cost=newdata(i,2);
       time=newdata(i,1);
       c=1;
       x=floor(newdata(i,1));
   end
end
clear sum;
plot(data(:,1),data(:,2))

% 
% t1=0;t2=0;t3=0;
% c1=0;c2=0;c3=0;
% for i=1:3:size(time)
%     t1=t1+time(i);
%     t2=t2+time(i+1);
%     t3=t3+time(i+2);
%     
%     c1=c1+cost(i);
%     c2=c2+cost(i+1);
%     c3=c3+cost(i+2);
% end
% 
% p(1,1)=t1/1000 /(size(time,1)/3);
% p(2,1)=t2/1000 /(size(time,1)/3);
% p(3,1)=t3/1000 /(size(time,1)/3);
% 
% p(1,2)=c1 /(size(cost,1)/3);
% p(2,2)=c2 /(size(cost,1)/3);
% p(3,2)=c3 /(size(cost,1)/3);
% 
% disp(p)
% 
% plot(p(:,1), p(:,2))
% hold on 
% plot(44,833,'*','r')



