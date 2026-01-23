function [V, onoff, p5, p6, bhj]=evaluateVPBhj(ego_x, u_e, veic2_x, u_v, model, brt)

    V = inf;    
    onoff = 0;   
    p5 = 0;
    p6 = 0;
    bhj = 0;

    if brt.g.dim == 6
        [rel_states, rel_dyn] = get_relative(ego_x, u_e, veic2_x, u_v, model);
        if rel_states(3) <= -3*pi/4 && rel_states(3) >= -5*pi/4
            rel_states(3) = rel_states(3) + 2*pi;
        end
        [limiti, ~] = check_limits(rel_states, brt.g);
        if limiti == 1
            [V, p] = interpolateBrt(brt, rel_states);
            onoff = 1;
            p5 = p(5);
            p6 = p(6);

            Mhj_0 = [p(5); p(6)];
            grad_V = [p(1); p(2); p(3); p(4); p(5); p(6)];
    
            bhj = grad_V'*rel_dyn'-Mhj_0'*u_e';
        end
    else
        [rel_states, rel_dyn] = get_relative7s(ego_x, u_e, veic2_x, u_v, model);
        if rel_states(3) <= -3*pi/4 && rel_states(3) >= -5*pi/4
            rel_states(3) = rel_states(3) + 2*pi;
        end
        [limiti, ~] = check_limits7s(rel_states, brt.g);
        if limiti == 1
            [V, p] = interpolateBrt7s(brt, rel_states);
            onoff = 1;
            p5 = p(6);  % not to create a further function for the constraint evaluation
            p6 = p(7); 

            Mhj_0 = [p(6); p(7)];
            grad_V = [p(1); p(2); p(3); p(4); p(5); p(6); p(7)];
    
            bhj = grad_V'*rel_dyn'-Mhj_0'*u_e';
        end
    end