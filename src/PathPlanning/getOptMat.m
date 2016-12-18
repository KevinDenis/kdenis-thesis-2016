function [optMat,matLength,matWidth] = getOptMat(varargin)
%[optMat,matLength,matWidth] = getOptMat(varargin)
%   This function will return all the possible compination of the input
%   vectors. Used to transform a nested forloop to a single for loop

% maybe better implementation :
%{
   iterations = [n na tpts];
T = prod(iterations);
(par)for ix = 1:T
      [h i t] = ind2sub(iterations,ix);
        ...;
end
%}

matWidth=nargin;

matLength=1;

for i=1:matWidth
    matLength=matLength*length(cell2mat(varargin(i)));
end

optMat=zeros(matLength,matWidth);

for i=1:matWidth
    
    range_i=cell2mat(varargin(i));
    
    repeat_val=1;
    if i~=matWidth
        for j=i+1:matWidth
            range_j=cell2mat(varargin(j));
            repeat_val=repeat_val*length(range_j);
        end
    end
    
    repeat_range=matLength/(repeat_val*length(range_i));
    
    counter=1;
    for k=1:repeat_range
        for value=range_i
            for m=1:repeat_val
                if  optMat(counter,i)~=0
                    disp('error')
                end
                optMat(counter,i)=value;
                counter=counter+1;
            end
        end
    end
end

end

