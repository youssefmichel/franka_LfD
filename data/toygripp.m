width=0.03 ;
force=0 ;
gripp_arr=[]

for i=1:12156
   gripp_arr(i,:)=[0.03,0] ;
   if(i>8000)
      gripp_arr(i,:)=[0.07,0] ;
   end

   if(i>9000)
       gripp_arr(i,:)=[0.03,2] ;
   end
   if(i>11000)
       gripp_arr(i,:)=[0.07,0] ;
   end


end


fileID = fopen('gripper_state_demo2.txt','w');
fprintf(fileID,'%12.8f %12.8f \n',[gripp_arr(:,1)'; gripp_arr(:,2)']);
fclose(fileID);