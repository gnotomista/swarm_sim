clc
clear
close all

N = 8;
N_OBST = 8;
ROBOT_RADIUS = 0.075;
D_MIN = 2*ROBOT_RADIUS;
GAMMA = 10;
OPTIM_OPT = optimoptions(@quadprog, 'Display', 'Off');
DT = 0.01;
T = 10;
robots = cell(1,N);
for i = 1 : N
    robots{i} = SingleIntegrator('width',ROBOT_RADIUS,...
        'initialState',-1+2*rand(3,1),...
        'simulationTimeStep',DT);
end
environment = 2*[1 -1 -1 1; 1 1 -1 -1];

s = Swarm('robots',robots,'environment',environment);

obstacles_centers = 0.5*randi([-2,2],2,N_OBST);
obstacles_radii = 0.05+0.2*rand(1,N_OBST);

s.plotFigure()
s.plotEnvironment('LineWidth', 5, 'Color', [0.5 0.5 0.5])
for i = 1 : N_OBST
    patch(obstacles_centers(1,i)+obstacles_radii(i)*cos(0:2*pi/100:2*pi),...
        obstacles_centers(2,i)+obstacles_radii(i)*sin(0:2*pi/100:2*pi),...
        [0.2,0.4,0.6],...
        'EdgeColor', 'none')
end

th = 2*pi*rand(1,N);
u_nom = 2*[cos(th); sin(th)];

for t = 0 : DT : T
    tic
    
    q = s.getPoses();
    
    % compute nominal input and make robots bounce off environment boundaries
    for i = 1 : s.N
        if q(1,i) < min(s.environment(1,:))+ROBOT_RADIUS
            q(1,i) = min(s.environment(1,:))+ROBOT_RADIUS;
            s.robots{i}.setPose(q(:,i));
            u_nom(1,i) = -u_nom(1,i);
        end
        if q(1,i) > max(s.environment(1,:))-ROBOT_RADIUS
            q(1,i) = max(s.environment(1,:))-ROBOT_RADIUS;
            s.robots{i}.setPose(q(:,i));
            u_nom(1,i) = -u_nom(1,i);
        end
        if q(2,i) < min(s.environment(2,:))+ROBOT_RADIUS
            q(2,i) = min(s.environment(2,:))+ROBOT_RADIUS;
            s.robots{i}.setPose(q(:,i));
            u_nom(2,i) = -u_nom(2,i);
        end
        if q(2,i) > max(s.environment(2,:))-ROBOT_RADIUS
            q(2,i) = max(s.environment(2,:))-ROBOT_RADIUS;
            s.robots{i}.setPose(q(:,i));
            u_nom(2,i) = -u_nom(2,i);
        end
    end
    
    % centralized CBF-QP for collision avoidance
    Aqp = zeros(s.N*(s.N-1)+s.N*N_OBST,2*s.N);
    bqp = zeros(s.N*(s.N-1)+s.N*N_OBST,1);
    constraint_idx = 0;
    for i = 1 : s.N
        if inpolygon(q(1,i),q(2,i),environment(1,:),environment(2,:))
            % robot-robot avoidance
            for j = 1 : s.N
                if j ~= i
                    constraint_idx = constraint_idx + 1;
                    Aqp(constraint_idx,2*i-1:2*i) = -2*(q(1:2,i)-q(1:2,j))';
                    bqp(constraint_idx,1) = GAMMA*(norm(q(1:2,i)-q(1:2,j))^2-D_MIN^2);
                end
            end
            % robot-obstacle avoidance
            for j = 1 : N_OBST
                constraint_idx = constraint_idx + 1;
                Aqp(constraint_idx,2*i-1:2*i) = -2*(q(1:2,i)-obstacles_centers(:,j))';
                bqp(constraint_idx,1) = GAMMA*(norm(q(1:2,i)-obstacles_centers(:,j))^2-(obstacles_radii(j)+ROBOT_RADIUS)^2);
            end
        end
    end
    u = quadprog(2*eye(2*N), -2*reshape(u_nom,1,2*s.N), Aqp, bqp, [], [], [], [], [], OPTIM_OPT);
    u = reshape(u,2,s.N);
    
    s.moveSingleIntegrators(u)
    
    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    
    drawnow limitrate
    pause(DT-toc)
end
