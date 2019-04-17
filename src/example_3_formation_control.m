clc
clear
close all

PLOT_GRAPH = true;

N = 6;
robots = cell(1,N);
for i = 1 : N
    robots{i} = SingleIntegrator('width',0.05,...
            'initialState',-1+2*rand(3,1),...
            'simulationTimeStep',0.01,...
            'vLinMax',10);
end

L = [3 -1 -1 -1 0 -1 ; ...
    -1 3 -1 0 -1 0 ; ...
    -1 -1 3 -1 0 -1 ; ...
    -1 0 -1 3 -1 0 ; ...
    0 -1 0 -1 3 -1 ; ...
    -1 0 -1 0 -1 3];
l = 0.4;
ldiag = 2*l;
weights = [0 l sqrt(3)*l ldiag 0 l; ...
    l 0 l 0 ldiag 0; ...
    sqrt(3)*l l 0 l 0 ldiag; ...
    ldiag 0 l 0 l 0; ...
    0 ldiag 0 l 0 l; ...
    l 0 ldiag 0 l 0];
s = Swarm('robots',robots,'L',L);

s.plotFigure()

for t = 1 : 1e3
    tic
    
    q = s.getPoses();
    
    u = zeros(2,s.N);
    for i = 1 : s.N
        for j = s.getNeighbors(i)
            u(:,i) = u(:,i) + 10*(norm(q(1:2,j)-q(1:2,i))^2-weights(i,j)^2)*(q(1:2,j)-q(1:2,i));
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
