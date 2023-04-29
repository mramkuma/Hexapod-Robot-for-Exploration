function [F] = solver(f, F, dt)

%% Numerical Solver (Runge - Kutta 4th order algorithm)

    % Get the shape of the differential equation
    [rows, columns] = size(f);
    
    % Iterates through all the rows (i)
    for i = 1 : rows

        % Iterates through all the columns (j)
        for j = 1 : columns
            
            % First term
            k1 = f(i, j);
            
            % Second term
            k2 = f(i, j) + (0.5 * k1 * dt);
            
            % Third term
            k3 = f(i, j) + (0.5 * k2 * dt);
            
            % Fourth term
            k4 = f(i, j) + (k3 * dt);
            
            % Numerical solution of current cell (i, j) of differential equation "f"
            F(i, j) = F(i, j) + ((1 / 6) * (k1 + (2 * k2) + (2 * k3) + k4) * dt);

        end

    end
        
end