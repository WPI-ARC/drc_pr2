%% Ask Which File to Analyize

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Slip_Q 0 = No, It Didn't Slip   %
% Slip_Q 1 = Yes, Slipped         %
%                                 %
% Hand_Q 0 = Neither Hand Slipped %
% Hand_Q 1 = Right Hand Slipped   %
% Hand_Q 2 = Left Hand Slipped    %
% Hand_Q 3 = Both Hands Slipped   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%
% Trial Answers:
% 1 = No Hand Slip
% 2 = No Hand Slip
% 3 = Right Hand Slip
% 4 = Right Hand Slip
% 5 = Left Hand Slip
% 6 = Left Hand Slip
% 7 = Two Hand Slip
% 8 = Two Hand Slip
%%%%%%%%%%%%%%%%%%%%%%%

%Ask the user for a filename
filename = input('Analize Which File: ', 's');

%Read the data from that file
data = csvread(filename);

%Setup the results struct
Conf_Results = zeros(6, 6);
Trial_Results = zeros(8, 6);

% DID IT SLIP, WHICH HAND
Answers = [...
0, 0;
0, 0;
1, 1;
1, 1;
1, 2;
1, 2;
1, 3;
1, 3];

%Loop through the data
for i = 1:size(data,1),

    C = data(i, 2); % Condition
    T = data(i, 3); % Trial
    R_S = data(i, 4); % Right Slip?
    R_C = data(i, 5); % Right Confidence?
    L_S = data(i, 6); % Left Slip?
    L_C = data(i, 7); % Left Confidence?
    
    %First check to see if they got it correct at all
    if Answers(T, 1) == R_S || Answers(T, 1) == L_S,
        Conf_Results(C, 1) = Conf_Results(C, 1) + 1;
        Conf_Results(C, 3) = Conf_Results(C, 3) + ((R_C + L_C) / 2);
%         Trial_Results(T, 1) = Trial_Results(T, 1) + 1;
%         Trial_Results(T, 3) = Trial_Results(T, 3) + W;
    else
        Conf_Results(C, 4) = Conf_Results(C, 4) + ((R_C + L_C) / 2);
%         Trial_Results(T, 4) = Trial_Results(T, 4) + W;
    end
    
    
     %Check to see if they were right about which hand
    if Answers(T, 2) == 0,
        
        if R_S == 0 && L_S == 0,
            Conf_Results(C, 2) = Conf_Results(C, 2) + 1;
            Conf_Results(C, 5) = Conf_Results(C, 5) + ((R_C + L_C) / 2);
        else
            Conf_Results(C, 6) = Conf_Results(C, 6) + ((R_C + L_C) / 2);
        end
        
    elseif Answers(T, 2) == 1,
        
        if R_S == 1 && L_S == 0,
            Conf_Results(C, 2) = Conf_Results(C, 2) + 1;
            Conf_Results(C, 5) = Conf_Results(C, 5) + ((R_C + L_C) / 2);
        else
            Conf_Results(C, 6) = Conf_Results(C, 6) + ((R_C + L_C) / 2);
        end
        
    elseif Answers(T, 2) == 2,
        
        if R_S == 0 && L_S == 1,
            Conf_Results(C, 2) = Conf_Results(C, 2) + 1;
            Conf_Results(C, 5) = Conf_Results(C, 5) + ((R_C + L_C) / 2);
        else
            Conf_Results(C, 6) = Conf_Results(C, 6) + ((R_C + L_C) / 2);
        end
        
    elseif Answers(T, 2) == 3,
        
        if R_S == 1 && L_S == 1,
            Conf_Results(C, 2) = Conf_Results(C, 2) + 1;
            Conf_Results(C, 5) = Conf_Results(C, 5) + ((R_C + L_C) / 2);
        else
            Conf_Results(C, 6) = Conf_Results(C, 6) + ((R_C + L_C) / 2);
        end
        
    end
        
        
        
        
%         Conf_Results(C, 2) = Conf_Results(C, 2) + 1;
%         Conf_Results(C, 5) = Conf_Results(C, 5) + W;
%     else
%         Conf_Results(C, 6) = Conf_Results(C, 6) + W;
%     end
    
    
%     %Check to see if they were right about which hand
%     if Answers(T, 2) == H,
%         Conf_Results(C, 2) = Conf_Results(C, 2) + 1;
%         Conf_Results(C, 5) = Conf_Results(C, 5) + W;
% %         Trial_Results(T, 2) = Trial_Results(T, 2) + 1;
% %         Trial_Results(T, 5) = Trial_Results(T, 5) + W;
%     else
%         Conf_Results(C, 6) = Conf_Results(C, 6) + W;
% %         Trial_Results(T, 6) = Trial_Results(T, 6) + W;
%     end
    
end

% Average and Scale the Results


%Average the Confidence Data
Conf_Results = Conf_Results / 8;
Conf_Results(:,1:2) = Conf_Results(:, 1:2) * 100;
% Trial_Results = Trial_Results / 8;
% Trial_Results(:,1:2) = Trial_Results(:, 1:2) * 100;

%% Condition Plotting

% Plot the Results
figure(1);

subplot(3,2,1);
hold all;
title('Correct ID of Any Slip (%)');
xlabel('Condition');
ylabel('Percent Correct');
axis([.5 6.5 0 100]);
bar(1, Conf_Results(1,1), 'r');
bar(2, Conf_Results(2,1), 'y');
bar(3, Conf_Results(3,1), 'g');
bar(4, Conf_Results(4,1), 'c');
bar(5, Conf_Results(5,1), 'b');
bar(6, Conf_Results(6,1), 'm');

subplot(3,2,2);
hold all;
title('Correct ID of Which Hand Slipped (%)');
xlabel('Condition');
ylabel('Percent Correct');
axis([.5 6.5 0 100]);
bar(1, Conf_Results(1,2), 'r');
bar(2, Conf_Results(2,2), 'y');
bar(3, Conf_Results(3,2), 'g');
bar(4, Conf_Results(4,2), 'c');
bar(5, Conf_Results(5,2), 'b');
bar(6, Conf_Results(6,2), 'm');

subplot(3,2,3);
hold all;
title('Confidence When Correctly Guessing Any Slip');
xlabel('Condition');
ylabel('Percent Correct');
axis([.5 6.5 0 5]);
bar(1, Conf_Results(1,3), 'r');
bar(2, Conf_Results(2,3), 'y');
bar(3, Conf_Results(3,3), 'g');
bar(4, Conf_Results(4,3), 'c');
bar(5, Conf_Results(5,3), 'b');
bar(6, Conf_Results(6,3), 'm');

subplot(3,2,5);
hold all;
title('Confidence When Incorrectly Guessing Any Slip');
xlabel('Condition');
ylabel('Percent Correct');
axis([.5 6.5 0 5]);
bar(1, Conf_Results(1,4), 'r');
bar(2, Conf_Results(2,4), 'y');
bar(3, Conf_Results(3,4), 'g');
bar(4, Conf_Results(4,4), 'c');
bar(5, Conf_Results(5,4), 'b');
bar(6, Conf_Results(6,4), 'm');

subplot(3,2,4);
hold all;
title('Confidence When Correctly Guessing Hand Slip');
xlabel('Condition');
ylabel('Percent Correct');
axis([.5 6.5 0 5]);
bar(1, Conf_Results(1,5), 'r');
bar(2, Conf_Results(2,5), 'y');
bar(3, Conf_Results(3,5), 'g');
bar(4, Conf_Results(4,5), 'c');
bar(5, Conf_Results(5,5), 'b');
bar(6, Conf_Results(6,5), 'm');

subplot(3,2,6);
hold all;
title('Confidence When Incorrectly Guessing Hand Slip');
xlabel('Condition');
ylabel('Percent Correct');
axis([.5 6.5 0 5]);
bar(1, Conf_Results(1,6), 'r');
bar(2, Conf_Results(2,6), 'y');
bar(3, Conf_Results(3,6), 'g');
bar(4, Conf_Results(4,6), 'c');
bar(5, Conf_Results(5,6), 'b');
bar(6, Conf_Results(6,6), 'm');

% %% Trial Plotting
% 
% % Plot the Results
% figure(2);
% 
% subplot(3,2,1);
% hold all;
% title('Correct ID of Any Slip (%)');
% xlabel('Condition');
% ylabel('Percent Correct');
% axis([.5 8.5 0 100]);
% bar(1, Trial_Results(1,1), 'r');
% bar(2, Trial_Results(2,1), 'y');
% bar(3, Trial_Results(3,1), 'g');
% bar(4, Trial_Results(4,1), 'c');
% bar(5, Trial_Results(5,1), 'b');
% bar(6, Trial_Results(6,1), 'm');
% bar(7, Trial_Results(5,1), 'w');
% bar(8, Trial_Results(6,1), 'k');
% 
% subplot(3,2,2);
% hold all;
% title('Correct ID of Which Hand Slipped (%)');
% xlabel('Condition');
% ylabel('Percent Correct');
% axis([.5 8.5 0 100]);
% bar(1, Trial_Results(1,2), 'r');
% bar(2, Trial_Results(2,2), 'y');
% bar(3, Trial_Results(3,2), 'g');
% bar(4, Trial_Results(4,2), 'c');
% bar(5, Trial_Results(5,2), 'b');
% bar(6, Trial_Results(6,2), 'm');
% bar(7, Trial_Results(5,2), 'w');
% bar(8, Trial_Results(6,2), 'k');
% 
% subplot(3,2,3);
% hold all;
% title('Confidence When Correctly Guessing Any Slip');
% xlabel('Condition');
% ylabel('Percent Correct');
% axis([.5 8.5 0 7]);
% bar(1, Trial_Results(1,3), 'r');
% bar(2, Trial_Results(2,3), 'y');
% bar(3, Trial_Results(3,3), 'g');
% bar(4, Trial_Results(4,3), 'c');
% bar(5, Trial_Results(5,3), 'b');
% bar(6, Trial_Results(6,3), 'm');
% bar(7, Trial_Results(5,3), 'w');
% bar(8, Trial_Results(6,3), 'k');
% 
% subplot(3,2,5);
% hold all;
% title('Confidence When Incorrectly Guessing Any Slip');
% xlabel('Condition');
% ylabel('Percent Correct');
% axis([.5 8.5 0 7]);
% bar(1, Trial_Results(1,4), 'r');
% bar(2, Trial_Results(2,4), 'y');
% bar(3, Trial_Results(3,4), 'g');
% bar(4, Trial_Results(4,4), 'c');
% bar(5, Trial_Results(5,4), 'b');
% bar(6, Trial_Results(6,4), 'm');
% bar(7, Trial_Results(5,4), 'w');
% bar(8, Trial_Results(6,4), 'k');
% 
% subplot(3,2,4);
% hold all;
% title('Confidence When Correctly Guessing Hand Slip');
% xlabel('Condition');
% ylabel('Percent Correct');
% axis([.5 8.5 0 7]);
% bar(1, Trial_Results(1,5), 'r');
% bar(2, Trial_Results(2,5), 'y');
% bar(3, Trial_Results(3,5), 'g');
% bar(4, Trial_Results(4,5), 'c');
% bar(5, Trial_Results(5,5), 'b');
% bar(6, Trial_Results(6,5), 'm');
% bar(7, Trial_Results(5,5), 'w');
% bar(8, Trial_Results(6,5), 'k');
% 
% subplot(3,2,6);
% hold all;
% title('Confidence When Incorrectly Guessing Hand Slip');
% xlabel('Condition');
% ylabel('Percent Correct');
% axis([.5 8.5 0 7]);
% bar(1, Trial_Results(1,6), 'r');
% bar(2, Trial_Results(2,6), 'y');
% bar(3, Trial_Results(3,6), 'g');
% bar(4, Trial_Results(4,6), 'c');
% bar(5, Trial_Results(5,6), 'b');
% bar(6, Trial_Results(6,6), 'm');
% bar(7, Trial_Results(5,6), 'w');
% bar(8, Trial_Results(6,6), 'k');