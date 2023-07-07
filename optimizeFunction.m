function [t,s]=optimizeFunction(separationTime,AwarenessETA,ownshipETA)
per=[AwarenessETA(2,1),AwarenessETA(2,2),AwarenessETA(2,3),AwarenessETA(2,4),AwarenessETA(2,5),AwarenessETA(2,6),ownshipETA(2,1)];
e=[AwarenessETA(1,1),AwarenessETA(1,2),AwarenessETA(1,3),AwarenessETA(1,4),AwarenessETA(1,5),AwarenessETA(1,6),ownshipETA(1,1)];
% 创建优化变量
x18 = optimvar("x",1,7,"LowerBound",0);
y15 = optimvar("y",7,7,"Type","integer","LowerBound",0,"UpperBound",1);

% 设置求解器的初始起点
initialPoint13.x = per;
initialPoint13.y = zeros(size(y15));

% 创建问题
problem = optimproblem;

% 定义问题目标

h=x18*y15;
problem.Objective = (x18(1)-per(1))^2+(x18(2)-per(2))^2+(x18(3)-per(3))^2+(x18(4)-per(4))^2+(x18(5)-per(5))^2+(x18(6)-per(6))^2+(x18(7)-per(7))^2;
% 定义问题约束
problem.Constraints.constraint1 = sum(y15(1,:)) == 1;
problem.Constraints.constraint2 = sum(y15(2,:)) == 1;
problem.Constraints.constraint3 = sum(y15(3,:)) == 1;
problem.Constraints.constraint4 = sum(y15(4,:)) == 1;
problem.Constraints.constraint5 = sum(y15(5,:)) == 1;
problem.Constraints.constraint6 = sum(y15(6,:)) == 1;
problem.Constraints.constraint7 = sum(y15(7,:)) == 1;
problem.Constraints.constraint11 = sum(y15(:,1)) == 1;
problem.Constraints.constraint12 = sum(y15(:,2)) == 1;
problem.Constraints.constraint13 = sum(y15(:,3)) == 1;
problem.Constraints.constraint14 = sum(y15(:,4)) == 1;
problem.Constraints.constraint15 = sum(y15(:,5)) == 1;
problem.Constraints.constraint16 = sum(y15(:,6)) == 1;
problem.Constraints.constraint17 = sum(y15(:,7)) == 1;

problem.Constraints.constraint18 = x18(1) >= e(1);
problem.Constraints.constraint19 = x18(2) >= e(2);
problem.Constraints.constraint20 = x18(3) >= e(3);
problem.Constraints.constraint21 = x18(4) >= e(4);
problem.Constraints.constraint22 = x18(5) >= e(5);
problem.Constraints.constraint23 = x18(6) >= e(6);
problem.Constraints.constraint24 = x18(7) >= e(7);

problem.Constraints.constraint31 = h(2)-h(1) >= separationTime(1);
problem.Constraints.constraint32 = h(3)-h(2) >= separationTime(1);
problem.Constraints.constraint33 = h(4)-h(3) >= separationTime(1);
problem.Constraints.constraint34 = h(5)-h(4) >= separationTime(1);
problem.Constraints.constraint35 = h(6)-h(5) >= separationTime(1);
problem.Constraints.constraint36 = h(7)-h(6) >= separationTime(1);
% 设置非默认求解器选项
%options = optimoptions("ga","NonlinearConstraintAlgorithm","penalty");
%options2 = optimoptions("intlinprog","Display","final","RootLPAlgorithm",...
%   "primal-simplex");
% 显示问题信息
show(problem);

% 求解问题
[solution,objectiveValue,reasonSolverStopped] = solve(problem,initialPoint13);
% 求解问题
%[solution,objectiveValue,reasonSolverStopped] = solve(problem,initialPoint13,"Solver","ga","Options",options);
%[solution,objectiveValue,reasonSolverStopped] = solve(problem,initialPoint13,...
 %   "Solver","intlinprog","Options",options2);
% 显示结果
solution
solution.y
reasonSolverStopped
objectiveValue

t=solution.x;
s=solution.y;
t
s
% 清除变量
clearvars x18 y15 initialPoint13 reasonSolverStopped objectiveValue
end