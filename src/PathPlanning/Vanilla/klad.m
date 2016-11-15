clc


temp=zeros(size(t));
for ii=1:length(t)
    temp(ii)=a(ii,:)*obs_k(ii,:)';
end
