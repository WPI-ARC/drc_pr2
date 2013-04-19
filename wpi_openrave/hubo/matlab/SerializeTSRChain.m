%function outstring = SerializeTSRChain(bSampleFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints)
%
%Input:
%bSampleStartFromChain (0/1): 1: Use this chain for sampling start configurations 0:Ignore for sampling starts
%bSampleGoalFromChain (0/1): 1: Use this chain for sampling goal configurations   0:Ignore for sampling goals
%bConstrainToChain (0/1): 1: Use this chain for constraining configurations   0:Ignore for constraining
%numTSRs (int): Number of TSRs in this chain (must be > 0)
%allTSRstring (str): string of concetenated TSRs generated using SerializeTSR. Should be like [TSRstring 1 ' ' TSRstring2 ...]
%mimicbodyname (str): name of associated mimicbody for this chain (NULL if none associated)
%mimicbodyjoints (int [1xn]): 0-indexed indices of the mimicbody's joints that are mimiced (MUST BE INCREASING AND CONSECUTIVE [FOR NOW])
%
%Output:
%outstring (str): string to include in call to cbirrt planner

function outstring = SerializeTSRChain(bSampleStartFromChain,bSampleGoalFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints)

outstring = [' TSRChain ' num2str(bSampleStartFromChain) ' ' num2str(bSampleGoalFromChain) ' ' num2str(bConstrainToChain) ' ' num2str(numTSRs) ' ' allTSRstring ' ' mimicbodyname];

if(numel(mimicbodyjoints) ~= 0)
    outstring = [outstring ' ' num2str(numel(mimicbodyjoints)) ' ' num2str(mimicbodyjoints)];
end


