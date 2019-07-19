function [B]=myupdate(I,J,D,e,H,W)

for i=1:H
    for j=1:W
        if D(i,j)>0
            B(i,j)=I(i,j);
        else
            B(i,j)=(1-e)*I(i,j)+e*J(i,j);
        end
    end
end