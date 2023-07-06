% Define keypoints

% Passive compression plane
p0 = [0; 0; 0];
p1 = [-0.142; 0; 1.1];    % Passive compression
p2 = [0; 70; 100];         % Force when inflated and length unchanged
p3 = [0.50; 20; 0];       % Free extension line
p4 = [0.142; 0; -3];      % Passive extension

P = [p0, p1, p2, p3, p4];

f_e = P(1, :);
f_p = P(2, :);
f_f = P(3, :);

[xData, yData, zData] = prepareSurfaceData( f_e, f_p, f_f );

fitresult = make_bellow_force_func;

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'untitled fit 1', 'f_F vs. f_e, f_p', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'f_e', 'Interpreter', 'none' );
ylabel( 'f_p', 'Interpreter', 'none' );
zlabel( 'f_F', 'Interpreter', 'none' );
grid on
view( 15.5, 31.7 );