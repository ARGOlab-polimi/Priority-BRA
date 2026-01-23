function h = human_integrate(h,i,dt)

h.simX(i,:) = [
    h.simX_0(4)*cos(h.simX_0(3)),...
    h.simX_0(4)*sin(h.simX_0(3)),...
    h.simU_0(1),...
    h.simU_0(2)]*dt+h.simX_0; 

h.simX_0 = h.simX(i,:);
h.simU_0 = h.simU(i,:);