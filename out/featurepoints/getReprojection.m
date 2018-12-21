function xleft_projected = getReprojection(Pl,x3b)

xleft_projected = Pl*x3b';
xleft_projected = xleft_projected(1:2,:)./xleft_projected(3,:);
xleft_projected = xleft_projected';

end
