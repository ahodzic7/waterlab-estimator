X_ref_t1_values = [2,2,3,4,5,6,5,4,3,2,4,6,5,4,3,2,3,5,4];
X_ref_t2_values = [2,2,3,3,4,5,5,6,6,5,4,3,2,3,5,6,4,2,3];

ref_switch = [76,211,352,497,594,707,797,939,1090,1303,1453,1607,1719,1899,2046,2188,2348,2497,2750];

for i = 1:size(ref_switch,2)
    if i == 1
        ref_length(i) = ref_switch(i);
    else
        ref_length(i) = ref_switch(i)-ref_switch(i-1);
    end
end

X_ref_t1 = [];
X_ref_t2 = [];
for i = 1:size(ref_length,2)

    X_ref_t1 = [X_ref_t1, X_ref_t1_values(i)*ones(1,t_resample*ref_length(i))];
    X_ref_t2 = [X_ref_t2, X_ref_t2_values(i)*ones(1,t_resample*ref_length(i))];
    
end

X_ref_sim = resample([X_ref_t1; X_ref_t2]',4,1)';%[X_ref_t1; X_ref_t2];


