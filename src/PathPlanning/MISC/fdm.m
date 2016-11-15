function [y_diff] = fdm(y,x,m,n)
%FDM Summary of this function goes here
%   Detailed explanation goes here
%   n max used points for fdm



n=round(n);

y_diff=zeros(size(x));
x_used=zeros(n,1);

for ii=1:length(x)
    if ii<n/2+1
        x_used=x(1:n);
        y_used=y(1:n);
        w = fdweights(x(ii),x_used,m);
        y_diff(ii) = w'*y_used;
    elseif ii>length(x)-n
        x_used=x(end-n:end);
        y_used=y(end-n:end);
        w = fdweights(x(ii),x_used,m);
        y_diff(ii) = w'*y_used;
    elseif rem(n,2) == 0
        x_used=x(ii-n/2:ii+n/2);
        y_used=y(ii-n/2:ii+n/2);
        w = fdweights(x(ii),x_used,m);
        y_diff(ii) = w'*y_used;    
    else     
        x_used=x(ii-floor(n/2):ii+floor(n/2));
        y_used=y(ii-floor(n/2):ii+floor(n/2));
        w = fdweights(x(ii),x_used,m);
        y_diff(ii) = w'*y_used;    
        
    end
end


    
end