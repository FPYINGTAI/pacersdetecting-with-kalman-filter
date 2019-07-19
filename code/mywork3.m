clear all;clc;
v = VideoReader('new1.mp4');
He=v.Height;                   %从v对象中引用高度、宽度
W=v.Width;x
e3=75;                      %差分的二值化阈值
e1=0.2;                       %初始更新速率
e2=0.01;
m=5;                       %建立初始背景帧数
i=1;

% 初始化Kalman滤波器
R=[[0.2845,0.0045]',[0.0045,0.0455]'];      %观察协方差矩阵
H=[[1,0]',[0,1]',[0,0]',[0,0]'];            %测量矩阵
Q=0.01*eye(4);                              %系统协方差矩阵, eye生成单位阵，例 s=eye(n) 返回n*n单位矩阵
P = 100*eye(4);                             %上一时刻的预测估计协方差矩阵
dt=10;
A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];  %从时刻k到k+1的状态xx(k-1,:)的跃迁矩阵
g = 1;                                              % pixels^2/time step
Bu = [0,0,0,g]';           %外界高斯白噪声，由前一帧预测当前帧的预测值时，Bu影响预测值的幅度.
kfinit=0;
x=zeros(400,4);

while hasFrame(v)
    video(:,:,:,i)=readFrame(v);
    i=i+1;
end
length=i-1;
for i=1:length
grayvideo(:,:,i)=rgb2gray(video(:,:,:,i));
end

back(:,:,1)=grayvideo(:,:,1);
for i=2:m
I=grayvideo(:,:,i)-grayvideo(:,:,i-1);
DB=(I>15);
back(:,:,i)=myupdate(back(:,:,i-1),grayvideo(:,:,i),DB,e1,He,W);
end
for i=m+1:length
I=grayvideo(:,:,i)-grayvideo(:,:,i-1);
DB=(I>15);
back(:,:,i)=myupdate(back(:,:,i-1),grayvideo(:,:,i),DB,e2,He,W);
end
%差分并筛选
for i=m+1:length
    DG(:,:,i)=uint8(abs(int16(grayvideo(:,:,i))-int16(back(:,:,i))));
    DGB0(:,:,i)=(DG(:,:,i)>e3);
    B=[0 1 0
       1 1 1
       0 1 0];
    DGB=imdilate(DGB0,B);%图像DGB0被结构元素B膨胀
    BW(:,:,i)=bwareafilt(DGB(:,:,i),1);
       area=bwarea(BW(:,:,i));

if area>10
      q1= regionprops(BW(:,:,i),'Centroid');
      q2=q1.Centroid;
      a1=q2(1);
      a2=q2(2);
      videomarked(:,:,:,i)=insertShape(video(:,:,:,i),'circle',[a1,a2,50],'LineWidth',5);
      
      % 更新kalman滤波器
        if kfinit==0
            xp = [a1,a2,0,0]' ;      %初始的预测状态
        else
            xp=A*x(i-1,:)'   ;       %预测未来状态
        end
        kfinit=1;
        PP = A*P*A' + Q   ;  %预测估计协方差矩阵
        K = PP*H'*inv(H*PP*H'+R) ; %增益, Inv 矩阵求逆
        x(i,:) = (xp + K*([a1,a2]' - H*xp))';  %修正状态,即更新后的状态。 x(n,:); 获取矩阵的第n行；x(:,n); 获取矩阵的第n列
        P = (eye(4)-K*H)*PP  ; %修正误差,即更新后的协方差
        
        % kalman预测的区域
        
        
        videomarked(:,:,:,i)=insertShape(videomarked(:,:,:,i),'circle',[x(i,1),x(i,2),50],'LineWidth',5,'Color','red');
       
        
end
end
implay(videomarked)
