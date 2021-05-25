function [Z_train_new, Y_train_new ]= reduce_M(Z_pred,Z_train,Y_train,Hp,M)

%Z_pred = [X_pred; U_pred; D_pred];

%dist_X_pred = pdist([Z_pred, Z_train]','cityblock');
dist_X_pred = pdist([Z_pred, Z_train]');

dist_X_pred_sq = squareform(dist_X_pred);
dist_X_pred_sq = dist_X_pred_sq(1:Hp,Hp+1:end);
indices_min = [];
M_collected = 0;

while M_collected<M
    for i = 1:Hp
        M_collected = M_collected+1;
        tmp = sortrows([dist_X_pred_sq(i,:)', (1:size(dist_X_pred_sq,2))'], 1);
        C = unique([indices_min;tmp(:,2)],'stable');
        indices_min = C(1:M_collected);%[indices_min, ; tmp(1:floor(M/Hp),2)];
        if (M_collected == M)
            break
        end
    end
    if (M_collected == M)
        break
    end
end

Z_train_new = Z_train(:,indices_min(1:M));
Y_train_new = Y_train(:,indices_min(1:M));

end
