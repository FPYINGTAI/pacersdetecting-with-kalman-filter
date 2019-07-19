clear all;clc;
v = VideoReader('new1.mp4');
He=v.Height;                   %��v���������ø߶ȡ����
W=v.Width;x
e3=75;                      %��ֵĶ�ֵ����ֵ
e1=0.2;                       %��ʼ��������
e2=0.01;
m=5;                       %������ʼ����֡��
i=1;

% ��ʼ��Kalman�˲���
R=[[0.2845,0.0045]',[0.0045,0.0455]'];      %�۲�Э�������
H=[[1,0]',[0,1]',[0,0]',[0,0]'];            %��������
Q=0.01*eye(4);                              %ϵͳЭ�������, eye���ɵ�λ���� s=eye(n) ����n*n��λ����
P = 100*eye(4);                             %��һʱ�̵�Ԥ�����Э�������
dt=10;
A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];  %��ʱ��k��k+1��״̬xx(k-1,:)��ԾǨ����
g = 1;                                              % pixels^2/time step
Bu = [0,0,0,g]';           %����˹����������ǰһ֡Ԥ�⵱ǰ֡��Ԥ��ֵʱ��BuӰ��Ԥ��ֵ�ķ���.
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
%��ֲ�ɸѡ
for i=m+1:length
    DG(:,:,i)=uint8(abs(int16(grayvideo(:,:,i))-int16(back(:,:,i))));
    DGB0(:,:,i)=(DG(:,:,i)>e3);
    B=[0 1 0
       1 1 1
       0 1 0];
    DGB=imdilate(DGB0,B);%ͼ��DGB0���ṹԪ��B����
    BW(:,:,i)=bwareafilt(DGB(:,:,i),1);
       area=bwarea(BW(:,:,i));

if area>10
      q1= regionprops(BW(:,:,i),'Centroid');
      q2=q1.Centroid;
      a1=q2(1);
      a2=q2(2);
      videomarked(:,:,:,i)=insertShape(video(:,:,:,i),'circle',[a1,a2,50],'LineWidth',5);
      
      % ����kalman�˲���
        if kfinit==0
            xp = [a1,a2,0,0]' ;      %��ʼ��Ԥ��״̬
        else
            xp=A*x(i-1,:)'   ;       %Ԥ��δ��״̬
        end
        kfinit=1;
        PP = A*P*A' + Q   ;  %Ԥ�����Э�������
        K = PP*H'*inv(H*PP*H'+R) ; %����, Inv ��������
        x(i,:) = (xp + K*([a1,a2]' - H*xp))';  %����״̬,�����º��״̬�� x(n,:); ��ȡ����ĵ�n�У�x(:,n); ��ȡ����ĵ�n��
        P = (eye(4)-K*H)*PP  ; %�������,�����º��Э����
        
        % kalmanԤ�������
        
        
        videomarked(:,:,:,i)=insertShape(videomarked(:,:,:,i),'circle',[x(i,1),x(i,2),50],'LineWidth',5,'Color','red');
       
        
end
end
implay(videomarked)
