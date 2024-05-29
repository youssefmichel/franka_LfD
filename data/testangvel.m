
clear all
data=load('rob_pose_quat_demo.txt')

 fs=1000 ; dt=1/fs ;
[B,A]=butter(1,2/(fs/2));

data=filtfilt(B,A,data) ;

quat=data(1:end,1:4) ;
quat_vec=quaternion(quat) ;

omeg1=angvel(quat_vec,1/fs,'frame') ;

plot(omeg1(100:end,:))
hold on
plot(data(100:end, 5:7),'--')
dt=1/fs ;
q_curr=quat(1,:)
Q=[] 

for i=1:length(data)
    
    omega= data(i, 5:7) ;
   
   
    q_curr_arr=quat_mult(quat_exp(0.5*dt*omega'),array2quat(q_curr')) ;
    q_curr=quat2array(q_curr_arr)' ;
    Q=[Q;q_curr] ;


  
end

figure
plot(Q) 
hold on
plot(quat,'--')


