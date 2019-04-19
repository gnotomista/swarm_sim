clc
clear
close all

PLOT_GRAPH = true;

N = 6;
DT = 0.01;
T = 10;
robots = cell(1,N);
for i = 1 : N
    robots{i} = SingleIntegrator('width',0.05,...
            'initialState',-1+2*rand(3,1),...
            'simulationTimeStep',DT);
end

L = [3 -1 -1 -1 0 -1 ; ...
    -1 3 -1 0 -1 0 ; ...
    -1 -1 3 -1 0 -1 ; ...
    -1 0 -1 3 -1 0 ; ...
    0 -1 0 -1 3 -1 ; ...
    -1 0 -1 0 -1 3];
l = 0.4;
weights = [0 l sqrt(3)*l 2*l 0 l; ...
    l 0 l 0 2*l 0; ...
    sqrt(3)*l l 0 l 0 2*l; ...
    2*l 0 l 0 l 0; ...
    0 2*l 0 l 0 l; ...
    l 0 2*l 0 l 0];
s = Swarm('robots',robots,'L',L);

s.plotFigure()

for t = 0 : DT : T
    tic
    
    q = s.getPoses();
    
    u = zeros(2,s.N);
    for i = 1 : s.N
        for j = s.getNeighbors(i)
            u(:,i) = u(:,i) + 10 * (norm(q(1:2,j)-q(1:2,i))^2-weights(i,j)^2)*(q(1:2,j)-q(1:2,i));
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
