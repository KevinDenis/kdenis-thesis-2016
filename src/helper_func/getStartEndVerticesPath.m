function [vertices] = getStartEndVerticesPath(Path)
%[vertices] = getStartEndVerticesPath(Path)
vertices=[[Path.x0].', [Path.y0].', [Path.th0].', [Path.x1].', [Path.y1].', [Path.th1].'];
end

