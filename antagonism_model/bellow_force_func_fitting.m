[fitresult, X] = make_bellow_force_func;

f_e = X(1, :);
f_p = X(2, :);
f_f = X(3, :);

[xData, yData, zData] = prepareSurfaceData( f_e, f_p, f_f );


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