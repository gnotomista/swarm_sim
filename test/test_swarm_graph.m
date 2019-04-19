clc
clear
close all

PLOT_GRAPH = true;
UNI_SI = 'si';

N = 6;
robots = cell(1,N);
for i = 1 : N
    if strcmpi(UNI_SI,'uni')
        robots{i} = Unicycle('width',0.1,...
            'length',0.1,...
            'initialState',-1+2*rand(3,1),...
            'simulationTimeStep');
    elseif strcmpi(UNI_SI,'si')
        robots{i} = SingleIntegrator('width',0.05,...
            'initialState',-1+2*rand(3,1),...
            'simulationTimeStep',0.01);
    end
end

s = Swarm('robots',robots,'L','cycle');

s.plotFigure()

n = 0;
while true
    tic
    
    n = n + 1;
    
    q = s.getPoses();
    
    u = reshape(-kron(s.L,eye(2))*reshape(q(1:2,:),2*s.N,1),2,s.N);
    
    s.moveSingleIntegrators(u)
    
    s.plotRobots([0.2 0.4 0.6])
    if PLOT_GRAPH
        s.plotGraph('Color',[0.75 0 0],'LineWidth',2)
    end
    
    drawnow limitrate
    pause(s.robots{1}.DT-toc)
end
