function volume = TXYGray(gray)

[m,n,l] = size(gray);
V = zeros(n,m,l);
for i = 1:l
    V(:,:,i) = gray(:,:,i)';
end
volume = V;
end