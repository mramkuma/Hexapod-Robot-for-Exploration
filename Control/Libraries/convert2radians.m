function [a] = convert2radians(a)
    
%% This function calculates the real values of any angle that is greater than 2 * pi
    [m, ~] = size(a);

    % Convert to number of cycles
    n = a * (1 / (2 * pi));
      
    % Convert to number of cycles (integer)
    N = fix(n);
      
    % Get angles bounded from -2pi to 2 pi
    a = 2 * pi * (n - N);
      
    % Iterate through each joint position
    for i = 1 : m

        % If position of joint is less than -pi
        if a(i, 1) < - pi
            
            % Sum 2 * pi
            a(i, 1) = a(i, 1) + (2 * pi);
    
        % Else, if position of joint is greater than pi
        elseif a(i, 1) > pi
            
            % Subtract 2 * pi
            a(i, 1) = a(i, 1) - (2 * pi);

        end
    end
      
end