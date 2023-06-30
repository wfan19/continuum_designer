function f = bellowForce(s, p)
        % Passive compression plane
        % p0 = [0; 0; 0];
        % p1 = [-0.142; 0; 1.1];    % Passive compression
        % p2 = [0; 20; 10];         % Force when inflated and length unchanged
        % plane_normal = null([p0'; p1'; p2']);
        n_compression = [0.9897, -0.0639, 0.1278];
    
        % Actuation plane
        % p0 = [0; 0; 0];
        % p1 = [0; 20; 10];         % Force when inflated and length unchanged
        % p2 = [0.50; 20; 0];       % Free extension line
        % plane_normal = null([p0'; p1'; p2']);
        n_actuation = [0.9984   -0.0250    0.0499];
    
        % Passive extension plane
        % p0 = [0; 0; 0];
        % p2 = [0.50; 20; 0];       % Free extension line
        % p1 = [0.142; 0; -3];      % Passive extension
        % plane_normal = null([p0'; p1'; p2']);
        n_extension = [0.9986   -0.0250   0.0473];
        if s > 0
            if p/s < 20/0.5
                % Extension - passive and active
                n = n_extension;
            else
                % Actuation
                n = n_actuation;
            end
        else
            % Compression - passive and active
            n = n_compression;
        end
        f = -dot(n(1:2), [s, p]) / n(3);
    end