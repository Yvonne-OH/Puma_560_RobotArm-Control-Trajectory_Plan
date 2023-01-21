%startup_rvc
clear;clc;close all
addpath(genpath('demo'),genpath('my_trajectory_plan'))

%%
demo_ik_fk=0; %Positive and negative solution test
demo=1;  %Calling matlab's method for trajectory planning

if demo
    Object_Lifting();
end
