function T = forward_kinematics(i,joint_rads)
% i is the joint, if you want the end joint to be 6
    

%DH_ This shows a summary of this function
% This shows the detailed description
    T = fk_single_transform(1,joint_rads);

    if i >1
        for j=2:1:i
            T=T*fk_single_transform(j,joint_rads);
        end
    end
    T = util_close_zero_to_zero_T(T,4,4,5);
end

function T = fk_single_transform(i,joint_rads)


    %% DH  data
    [alpha_data,a_data,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh();
    
    % Convert alpha and a subscripts from 1 to 0
    alpha_0=alpha_data(1);      a_0=a_data(1);           th(1) =joint_rads(1);
    alpha(1)=alpha_data(2);     a(1)=a_data(2);          th(2) =joint_rads(2);
    alpha(2)=alpha_data(3);     a(2)=a_data(3);          th(3) =joint_rads(3);
    alpha(3)=alpha_data(4);     a(3)=a_data(4);          th(4) =joint_rads(4);
    alpha(4)=alpha_data(5);     a(4)=a_data(5);          th(5) =joint_rads(5);
    alpha(5)=alpha_data(6);     a(5)=a_data(6);          th(6) =joint_rads(6);

    %% general 
    if i==1
            T =[cos(th(i))              -sin(th(i))                 0               a_0; 
            cos(alpha_0)*sin(th(i))     cos(alpha_0)*cos(th(i))     -sin(alpha_0)   -sin(alpha_0)*d(i);
            sin(alpha_0)*sin(th(i))     sin(alpha_0)*cos(th(i))     cos(alpha_0)    cos(alpha_0)*d(1);
            0                           0                           0               1];
    else
            T =[cos(th(i))                      -sin(th(i))                 0                   a(i-1); 
                cos(alpha(i-1))*sin(th(i))      cos(alpha(i-1))*cos(th(i))  -sin(alpha(i-1))    -sin(alpha(i-1))*d(i);
                sin(alpha(i-1))*sin(th(i))      sin(alpha(i-1))*cos(th(i))  cos(alpha(i-1))     cos(alpha(i-1))*d(i);
                0                               0                           0                   1];
    end
        
end
