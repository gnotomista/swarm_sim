clc
clear
close all

PLOT_GRAPH = true;

N = 10;
robots = cell(1,N);
for i = 1 : N
    robots{i} = SingleIntegrator('width',0.05,...
            'initialState',-1+2*rand(3,1),...
            'simulationTimeStep',0.01,...
            'vLinMax',10);
end

s = Swarm('robots',robots,'L','cycle');

s.plotFigure()

for t = 1 : 1e3
    tic
    
    q = s.getPoses();
    
    % centralized version
    % u = reshape(-2*kron(s.L,eye(2))*reshape(q(1:2,:),2*s.N,1),2,s.N);
    % decentralized version
    u = zeros(2,s.N);
    for i = 1 : s.N
        for j = s.getNeighbors(i)
            u(:,i) = u(:,i) + 1*(q(1:2,j)-q(1:2,i));
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
