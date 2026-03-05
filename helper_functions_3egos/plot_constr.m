clear all
close all
clc

theta = 0; %in rad
D1 = 4.3;
y_lim = 8.0;

C_left = @(y) exp(y+D1/2*sin(theta)-(y_lim/4));

C_right = @(y) exp(-(y+D1/2*sin(theta)+(y_lim/4)));


num_l = linspace(-4,4,1000);
num_r = linspace(0,-4,1000);

val_l = C_left(num_l);
val_r = C_right(num_l);

tot = val_r+val_l;

figure

% 1. Attiva l'asse Y di DESTRA e disegna il grafico
plot(num_l, tot, 'k', 'LineWidth', 1.5);
ylabel('$C_{\mathrm{right}}+C_{\mathrm{left}}$', 'Interpreter', 'latex', 'FontSize', 18);

% 2. Attiva l'asse Y di SINISTRA e imposta la sua etichetta
% yyaxis left;
% ylabel('$C_{\mathrm{left}}$', 'Interpreter', 'latex', 'FontSize', 18);

% 3. Imposta l'etichetta dell'asse X
xlabel('$y$ [m]', 'Interpreter', 'latex', FontSize= 18);

% 4. Inverti la direzione dell'asse X e ottieni l'handle degli assi
ax = gca; 
ax.XDir = 'reverse';

% --- MODIFICHE CHIAVE ---

% 5. Sincronizza i limiti degli assi Y e assicurati che i colori siano corretti
% ax.YAxis(1).Limits = ax.YAxis(2).Limits; % Assicura che le scale siano identiche
% ax.FontSize = 12;
ax.YAxis(1).Color = 'k'; 
% ax.YAxis(2).Color = 'k';

% 6. Attiva esplicitamente sia la griglia verticale che quella orizzontale
ax.XGrid = 'on';
ax.YGrid = 'on';

