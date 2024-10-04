% ikine_numerical.m
% Descrption: A custom inverse kinematics function calculating the numerical solution
% Input: desired_T (the pose of the end-effector as an 4x4 SE(3) homogeneous transformation)
       % DH (a struct containing d, a, alpha, offset)
% Output: q_solution (solution of joint configuration)

function q_solution = ikine_numerical(desired_T,DH)

% 初始关节角度猜测
initial_guess = pi/3 * ones(1, 6); 

% 设置 fsolve 选项
options = optimoptions('fsolve', 'Display', 'iter', 'Algorithm', 'levenberg-marquardt');

% 调用 fsolve
[q_solution, fval, exitflag, output] = fsolve(@(q) ikine_target_function(q, desired_T, DH), initial_guess, options);

% 检查解的有效性
if exitflag > 0
    % 解有效
    fprintf('Solution found:\n');
    disp(q_solution);
else
    % 解无效
    fprintf('No solution found. Exit flag: %d\n', exitflag);
end

function F = ikine_target_function(q, desired_T, DH)
    % 计算正运动学
    T = fkine_c(q, DH);
    
    % 计算位置误差
    position_error = desired_T(1:3, 4) - T(1:3, 4);
    
    % 计算方向误差
    orientation_error = [
        desired_T(1:3, 1) - T(1:3, 1);
        desired_T(1:3, 2) - T(1:3, 2);
        desired_T(1:3, 3) - T(1:3, 3)
    ];
    
    % 合并位置和方向误差
    F = [position_error; orientation_error(:)];
end

end
