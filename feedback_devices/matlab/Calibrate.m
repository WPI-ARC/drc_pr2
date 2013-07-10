function [ data_calib ] = Calibrate( data_raw, start, last )
%CALIBRATE Summary of this function goes here
%   Detailed explanation goes here

data_calib = zeros(size(data_raw));

for i = 1:size(data_raw,2),
    
    data_calib(:,i) = data_raw(:,i) - mean(data_raw(start:last, i));
    
end

end

