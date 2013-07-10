%Get the participant number from the person running the study
Participant_Num = input('Please Enter The Participant Number:');
disp(' ');

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


% Open a file to save all the data
str = strcat('Participant_', num2str(Participant_Num), '.csv');
fid = fopen(str, 'w');

% Find the Condition Number
Cond_Number = mod(Participant_Num, 6); 

% Make sure to check the bounds since MATLAB indexes from 1 not 0
if Cond_Number == 0, Cond_Number = 6; end;

% Run through each condition
for c = 1:6,
    
    disp('======== Starting New Condition ======== ');
    Current_Condition = Condition_Latin_Squares(Cond_Number, c);
    str = strcat('Condition_', num2str(Current_Condition));
    disp(str);
    
    
    % Run through each trial
    for t = 1:8,
        
        Current_Trial = Trial_Latin_Squares(c, t);
        str = strcat('    Trial_', num2str(Current_Trial));
        disp(str);
        pause();
       
        
        % Ask if the user thought the robot slipped
        loop = true;
        while(loop),
            % Ask the User the appropriate questions
            Slip_R = input('        Did the RIGHT hand slip? (Y / N) :', 's');
            
            if Slip_R == 'Y', loop = false; Slip_R_Val = 1;
            elseif Slip_R == 'N', loop = false; Slip_R_Val = 0;
                
            else
                disp('        Please Enter a Valid Response either "Y" or "N"');
            end
            
        end        
        
        
        % Ask how confident the user is in their decision
        loop = true;
        while(loop),
            % Ask the User the appropriate questions
            Conf_R = input('        How condident are you in your decision? (1 - 5) : ', 's');

            if Conf_R == '1', loop = false; Conf_R_Val = 1;
            elseif Conf_R == '2', loop = false; Conf_R_Val = 2;
            elseif Conf_R == '3', loop = false; Conf_R_Val = 3;
            elseif Conf_R == '4', loop = false; Conf_R_Val = 4;
            elseif Conf_R == '5', loop = false; Conf_R_Val = 5;
            else
                disp('        Please Enter a Valid Number Between 1 and 7');
            end
        end
        
        
        % Ask if the user thought the robot slipped
        loop = true;
        while(loop),
            % Ask the User the appropriate questions
            Slip_L = input('        Did the LEFT hand slip? (Y / N) :', 's');
            
            if Slip_L == 'Y', loop = false; Slip_L_Val = 1;
            elseif Slip_L == 'N', loop = false; Slip_L_Val = 0;
                
            else
                disp('        Please Enter a Valid Response either "Y" or "N"');
            end
            
        end     
        
        % Ask how confident the user is in their decision
        loop = true;
        while(loop),
            % Ask the User the appropriate questions
            Conf_L = input('        How condident are you in your decision? (1 - 5) : ', 's');

            if Conf_L == '1', loop = false; Conf_L_Val = 1;
            elseif Conf_L == '2', loop = false; Conf_L_Val = 2;
            elseif Conf_L == '3', loop = false; Conf_L_Val = 3;
            elseif Conf_L == '4', loop = false; Conf_L_Val = 4;
            elseif Conf_L == '5', loop = false; Conf_L_Val = 5;
            else
                disp('        Please Enter a Valid Number Between 1 and 7');
            end
        end
        
        
        % Store the data
        fprintf(fid, '%i,%i,%i,%i,%i,%i,%i\n', Participant_Num, Current_Condition, Current_Trial, Slip_R_Val, Conf_R_Val, Slip_L_Val, Conf_L_Val);
        
        disp(' ');
        
    end
    
    disp('If you would like, we can take a short break now.');
    disp(' ');
    pause();
    
end