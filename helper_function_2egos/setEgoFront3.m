function ego_obs = setEgoFront3(ego)

N_egos = length(ego)-1;
s_egos = zeros(1,N_egos);
ego_obs = ones(2,N_egos)*(N_egos+1);

for i = 1:N_egos
    s_egos(i) = ego{i}.x_current(1);
end

sorted_egos = sort(s_egos);

for i=1:N_egos-1
    [~,ind_1] = find(sorted_egos(i) == s_egos);   %to find index of the current ego
    [~,ind_2] = find(sorted_egos(i+1) == s_egos); %to find index of the ego in front of the current
    ego_obs(1,ind_1) = ind_2;
end

for i=2:N_egos
    [~,ind_1] = find(sorted_egos(i) == s_egos);   %to find index of the current ego
    [~,ind_2] = find(sorted_egos(i-1) == s_egos); %to find index of the ego in front of the current
    ego_obs(2,ind_1) = ind_2;
end
