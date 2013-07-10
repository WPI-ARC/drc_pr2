%% Initialize
clear;
clc;
close all;

%% Setup Stuff Goes Here, clear the terminal, etc.

Condition_Latin_Squares = [...
1, 2, 6, 3, 5, 4;
2, 3, 1, 4, 6, 5;
3, 4, 2, 5, 1, 6;
4, 5, 3, 6, 2, 1;
5, 6, 4, 1, 3, 2;
6, 1, 5, 2, 4, 3];

Trial_Latin_Squares = [...
1	2	8	3	7	4	6	5;
2	3	1	4	8	5	7	6;
3	4	2	5	1	6	8	7;
4	5	3	6	2	7	1	8;
5	6	4	7	3	8	2	1;
6	7	5	8	4	1	3	2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Slip_Q 0 = No, It Didn't Slip   %
% Slip_Q 1 = Yes, Slipped         %
%                                 %
% Hand_Q 0 = Neither Hand Slipped %
% Hand_Q 1 = Right Hand Slipped   %
% Hand_Q 2 = Left Hand Slipped    %
% Hand_Q 3 = Both Hands Slipped   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup for the Current Run

%Get the participant number from the person running the study
Participant_Number = input('Please Enter The Participant Number:');
disp(' ');

% Open a file to save all the data
A = clock;
datestring = strcat(num2str(A(1)), '-', num2str(A(2)), '-', num2str(A(3)), '-', num2str(A(4)), '-', num2str(A(5)), '-', num2str(floor(A(6))));
str = strcat('Participant_', num2str(Participant_Number), '-', datestring, '.csv');
fid = fopen(str, 'w');

% Find the Condition Number
Cond_Number = mod(Participant_Number, 6); 

% Make sure to check the bounds since MATLAB indexes from 1 not 0
if Cond_Number == 0, Cond_Number = 6; end;

%% Explaination

disp('====> Show them the intro sheet and wait for them to be done');
disp(' ');

pause();

%% The actual Running of the Conditions

disp('The Conditions will be Run in the Order:');
disp(Condition_Latin_Squares(Cond_Number, :));
disp(' ');

% Run through each condition
for c = 1:6,
    
    disp('======== Starting New Condition ======== ');
    disp(' ');
    
    disp('Please Run: ');
    Current_Condition = Condition_Latin_Squares(Cond_Number, c);
    str = strcat('$ roslaunch feedback_devices Condition_', num2str(Current_Condition),'.launch');
    disp(str);
    disp(' ');
    pause();
    
    disp('Please run the No-Slip demo now');
    disp(' ');
    pause();
    
    disp('Please run the Slipping demo now');
    disp(' ');
    pause();
    
    disp(' ');
    
    
    % Run through each trial
    for t = 1:8,
        
%         disp('    ===> Run the correct feedback node.');
%         pause();
        
        %Have the person running the study start playing the correct trial
        Current_Trial = Trial_Latin_Squares(c, t);
        str = strcat('    $ rosbag play trial_', num2str(Current_Trial), '.bag');
        disp(str);
        pause();
       
        
        
        
        
        
%         % Ask if the user thought the robot slipped
%         loop = true;
%         while(loop),
%             % Ask the User the appropriate questions
%             Slip_Q = input('        Do you think the robot slipped? (Y / N) :', 's');
%             
%             if Slip_Q == 'Y', loop = false; Slip_Val = 1;
%             elseif Slip_Q == 'N', loop = false; Slip_Val = 0;
%             elseif Slip_Q == 'y', loop = false; Slip_Val = 1;
%             elseif Slip_Q == 'n', loop = false; Slip_Val = 0;
%             else
%                 disp('        Please Enter a Valid Response either "Y" or "N"');
%             end
%             
%         end
%         
%         
%         % Ask which hand they think slipped
%         if Slip_Val == 1,
%             loop = true;
%             while(loop),
%                 % Ask the User the appropriate questions
%                 Hand_Q = input('        Which hand do you think slipped? (B / R / L) :', 's');
% 
%                 if Hand_Q == 'B', loop = false; Hand_Val = 3;
%                 elseif Hand_Q == 'R', loop = false; Hand_Val = 1;
%                 elseif Hand_Q == 'L', loop = false; Hand_Val = 2;
%                 elseif Hand_Q == 'b', loop = false; Hand_Val = 3;
%                 elseif Hand_Q == 'r', loop = false; Hand_Val = 1;
%                 elseif Hand_Q == 'l', loop = false; Hand_Val = 2;
%                 else
%                     disp('        Please Enter a Valid Response either "B", "R" or "L"');
%                 end
%             end
%         else 
%             Hand_Val = 0;
%         end
%         
%         
%         % Ask how confident the user is in their decision
%         loop = true;
%         while(loop),
%             % Ask the User the appropriate questions
%             Conf_Q = input('        How condident are you in your decision? (1 - 7) : ', 's');
% 
%             if Conf_Q == '1', loop = false; Conf_Val = 1;
%             elseif Conf_Q == '2', loop = false; Conf_Val = 2;
%             elseif Conf_Q == '3', loop = false; Conf_Val = 3;
%             elseif Conf_Q == '4', loop = false; Conf_Val = 4;
%             elseif Conf_Q == '5', loop = false; Conf_Val = 5;
%             elseif Conf_Q == '6', loop = false; Conf_Val = 6;
%             elseif Conf_Q == '7', loop = false; Conf_Val = 7;
%             else
%                 disp('        Please Enter a Valid Number Between 1 and 7');
%             end
%         end
%         
%         % Store the data
%         fprintf(fid, '%i,%i,%i,%i,%i,%i\n', Participant_Number, Current_Condition, Current_Trial, Slip_Val, Hand_Val, Conf_Val);
%         
%         disp(' ');
        
    end
    
    disp('If you would like, we can take a short break now.');
    disp(' ');
    pause();
    
end

% fclose(fid);