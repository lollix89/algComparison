function Entropy = entropy (input)

info_sz = size (input);
n = info_sz (1, 2);
Entropy = 0;
for i=1:n
    if isequal(input(1,i),0)
        tmp = 0;
    else
        tmp = (-input(1,i)*log2(input(1,i)));
    end
    Entropy = Entropy + tmp;
end


end
