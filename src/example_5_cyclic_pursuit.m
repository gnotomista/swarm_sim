clc
clear
close all

PLOT_GRAPH = true;
R = @(t) [cos(t) -sin(t); sin(t) cos(t)];

N = 20;
robots = cell(1,N);
for i = 1 : N
    robots{i} = Unicycle('width',0.1,...
        'length',0.1,...
        'initialState',-2+4*rand(3,1),...
        'simulationTimeStep',0.01,...
        'vLinMax',10,...
        'vAngMax',10);
end
th0 = pi/N;
th = pi/N-th0;
Rth = R(th);

L = full(gallery('tridiag',N,0,1,-1));
L(N,1) = -1;
s = Swarm('robots',robots,'L',L);

s.plotFigure()

for t = 1 : 1e4
    tic
    
    q = s.getPoses();
    
    u = zeros(2,s.N);
    for i = 1 : s.N
        for j = s.getNeighbors(i)
            u(:,i) = 10 * Rth * (q(1:2,j)-q(1:2,i));
        end
    end
    
    s.moveSingleIntegrators(u)
    
    if PLOT_GRAPH
        s.plotGraph('Color',[0,0.145,0.298],'LineWidth',10)
    end
    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    
    drawnow limitrate
    pause(s.robots{1}.DT-toc)
end
