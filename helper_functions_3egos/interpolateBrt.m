function [V, p] = interpolateBrt(brt, X_rel)

grid_vectors = {brt.g.vs{1}(:), brt.g.vs{2}(:), brt.g.vs{3}(:), brt.g.vs{4}(:), brt.g.vs{5}(:),brt.g.vs{6}(:)};

F_V = griddedInterpolant(grid_vectors, brt.data);
F_p1 = griddedInterpolant(grid_vectors, brt.grad{1});
F_p2 = griddedInterpolant(grid_vectors, brt.grad{2});
F_p3 = griddedInterpolant(grid_vectors, brt.grad{3});
F_p4 = griddedInterpolant(grid_vectors, brt.grad{4});
F_p5 = griddedInterpolant(grid_vectors, brt.grad{5});
F_p6 = griddedInterpolant(grid_vectors, brt.grad{6});

V = F_V(X_rel(1), X_rel(2), X_rel(3), X_rel(4), X_rel(5), X_rel(6));
p = [F_p1(X_rel(1), X_rel(2), X_rel(3), X_rel(4), X_rel(5), X_rel(6)), ...
    F_p2(X_rel(1), X_rel(2), X_rel(3), X_rel(4), X_rel(5), X_rel(6)), ...
    F_p3(X_rel(1), X_rel(2), X_rel(3), X_rel(4), X_rel(5), X_rel(6)), ...
    F_p4(X_rel(1), X_rel(2), X_rel(3), X_rel(4), X_rel(5), X_rel(6)), ...
    F_p5(X_rel(1), X_rel(2), X_rel(3), X_rel(4), X_rel(5), X_rel(6)), ...
    F_p6(X_rel(1), X_rel(2), X_rel(3), X_rel(4), X_rel(5), X_rel(6))];

end