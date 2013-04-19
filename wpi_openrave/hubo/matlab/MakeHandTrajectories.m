function MakeHandTrajectories(DOFValues,reps,fname,strength)

jointNames = {'RHY', 'RHR', 'RHP', 'RKP', 'RAP', 'RAR', 'LHY', 'LHR', 'LHP', 'LKP', 'LAP', 'LAR', 'RSP', 'RSR', 'RSY', 'REP', 'RWY', 'RWR', 'RWP', 'LSP', 'LSR', 'LSY', 'LEP', 'LWY', 'LWR', 'LWP', 'NKY', 'NK1', 'NK2', 'HPY', 'rightThumbKnuckle1', 'rightIndexKnuckle1', 'rightMiddleKnuckle1', 'rightRingKnuckle1', 'rightPinkyKnuckle1', 'leftThumbKnuckle1', 'leftIndexKnuckle1', 'leftMiddleKnuckle1', 'leftRingKnuckle1', 'leftPinkyKnuckle1'};

jointIndices = [1, 3, 5, 7, 9, 11, 2, 4, 6, 8, 10, 12, 13, 15, 17, 19, 21, -1, 23, 14, 16, 18, 20, 22, -1, 24, -1, -1, -1, 0, 41, 29, 32, 35, 38, 56, 44, 47, 50, 53];

fid = fopen(fname,'w');

for l = 1:reps
  thisLine = '';
  for j = 1:size(jointIndices,2)
    if jointIndices(j) ~= -1
      if j > 30
	if DOFValues(jointIndices(j)+1) > 0.2
	  fwrite(fid,[num2str(strength) ' ']);
	else
	  fwrite(fid,['-' num2str(strength) ' ']);
	end
      else
	fwrite(fid,[num2str(DOFValues(jointIndices(j)+1)) ' ']);
      end
    else
      fwrite(fid,'0.0 ');
    end
  end
  fwrite(fid,sprintf(['\n']));
end
fclose(fid);