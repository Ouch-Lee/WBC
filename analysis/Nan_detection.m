clear M
M = tauA_opt;
[col, row] = size(M);
flag = 0;
for i = 1:row
    for j = 1:col
        if (isnan(M(j,i)) == 1 || abs(M(j,i)) > 1e3) 
            flag = 1;
        end
        if (flag == 1)
            break;
        end
    end
    if (flag == 1)
        break;
    end
end

disp('over!');