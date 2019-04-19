clc
clear
close all

PLOT_GRAPH = true;

N = 10;
DT = 0.01;
T = 1;
robots = cell(1,N);
for i = 1 : N
    robots{i} = Unicycle('width',0.1,...
        'length',0.1,...
        'initialState',[-0.5+1*rand(2,1); 2*pi*rand()],...
        'simulationTimeStep',DT);
end

s = Swarm('robots',robots,'L','complete');

s.plotFigure()

for t = 0 : DT : T
    tic
    
    q = s.getPoses();
    
    u = zeros(2,s.N);
    u(1,:) = 1;
    for i = 1 : s.N
        th_avg_neigh_i = 0;
        Ni = s.getNeighbors(i);
        for j = Ni
            th_avg_neigh_i = th_avg_neigh_i + q(3,j);
        end
        th_avg_neigh_i = th_avg_neigh_i / numel(Ni);
        u(2,i) = u(2,i) + 10 * (th_avg_neigh_i-q(3,i));
    end
    
    s.moveUnicycles(u)
    
    if PLOT_GRAPH
        s.plotGraph('Color',[0,0.145,0.298],'LineWidth',5)
    end
    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    
    drawnow limitrate
    pause(DT-toc)
end
