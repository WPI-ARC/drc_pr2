function outstring = SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)

outstring = [num2str(manipindex) ' ' bodyandlink ' ' num2str([GetRot(T0_w),GetTrans(T0_w)]) ' ' num2str([GetRot(Tw_e),GetTrans(Tw_e)])];

if isequal(size(Bw),[6 2])
    Bw = Bw';
end

outstring = [outstring, ' ' num2str(Bw(:)')];