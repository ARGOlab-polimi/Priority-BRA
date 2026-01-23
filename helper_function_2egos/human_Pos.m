function brt_hp = human_Pos(ego_state, hum_state, brt_hp1, brt_hp2)

X_rel = ego_state(3)-hum_state(3);

if X_rel == 0
    brt_hp = brt_hp1;
else
    brt_hp = brt_hp2;
end