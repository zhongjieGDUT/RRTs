function sorted_Ls = sortList(Ls)
%SORTLIST

[~,~,n]=size(Ls);
sorted_Ls = cell(1,3,n);
cost=zeros(n,1);
for i=1:n
    cost(i)=Ls{1,2,i};
end
[~,index]=sort(cost);
for i=1:n
    for j=1:3
        sorted_Ls{1,j,i}=Ls{1,j,index(i)}; 
    end
end
end

