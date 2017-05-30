function [StateLattice_endStart] = StartEndSwap(StateLattice_startEnd)
%[StateLattice_endStart] = StartEndSwap(StateLattice_startEnd)
StateLattice_endStart=StateLattice_startEnd;
n=length(StateLattice_startEnd);
offsetID=StateLattice_startEnd(end).ID;
for ii=1:n
    ID_ii=  offsetID + ii;
    StateLattice_endStart(ii).x0     = StateLattice_startEnd(ii).x1;
    StateLattice_endStart(ii).y0     = StateLattice_startEnd(ii).y1;
    StateLattice_endStart(ii).th0    = StateLattice_startEnd(ii).th1;
    StateLattice_endStart(ii).x1     = StateLattice_startEnd(ii).x0;
    StateLattice_endStart(ii).y1     = StateLattice_startEnd(ii).y0;
    StateLattice_endStart(ii).th1    = StateLattice_startEnd(ii).th0;
    StateLattice_endStart(ii).X      = flip(StateLattice_startEnd(ii).X);
    StateLattice_endStart(ii).Y      = flip(StateLattice_startEnd(ii).Y);
    StateLattice_endStart(ii).TH     = flip(StateLattice_startEnd(ii).TH);
    StateLattice_endStart(ii).K      = flip(StateLattice_startEnd(ii).K);
    StateLattice_endStart(ii).ID     = ID_ii;
end
end

