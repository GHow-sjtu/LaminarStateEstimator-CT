function p = ForceMeanFilter(points, win)

[m,len] = size(points);

count = round(len/win);
if count*win > len
    count = count - 1;
end

p = size(m,count);

for i = 1: count

    p(i) = mean(points((i-1)*win+1:i*win));
end

end