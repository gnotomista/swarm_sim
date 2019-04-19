clc
clear
close all

PLOT_GRAPH = true;

N = 50;
DT = 0.01;
T = 10;
robots = cell(1,N);
for i = 1 : N
    robots{i} = SingleIntegrator('width',0.05,...
            'initialState',-1+2*rand(3,1),...
            'simulationTimeStep',DT);
end

s = Swarm('robots',robots,'L','cycle');

s.plotFigure()

for t = 0 : DT : T
    tic
    
    q = s.getPoses();
    
    % centralized version
    % u = reshape(-2*kron(s.L,eye(2))*reshape(q(1:2,:),2*s.N,1),2,s.N);
    
    % decentralized version
    u = zeros(2,s.N);
    for i = 1 : s.N
        for j = s.getNeighbors(i)
            u(:,i) = u(:,i) + 10 * (q(1:2,j)-q(1:2,i));
        end
    end
    
    s.moveSingleIntegrators(u)
    
    if PLOT_GRAPH
        s.plotGraph('Color',[0,0.145,0.298],'LineWidth',5)
    end
    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    
    drawnow limitrate
    pause(DT-toc)
end
