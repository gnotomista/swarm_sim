clc
clear
close all

N = 5;
robots = cell(1,N);
for i = 1 : N
    robots{i} = Unicycle('width',0.1,...
        'length',0.1,...
        'initialState',-1+2*rand(3,1),...
        'simulationTimeStep',0.01);
end
environment = [cos(linspace(0,2*pi,6)); sin(linspace(0,2*pi,6))];
s = Swarm('robots',robots,...
    'environment',environment);

s.plotFigure()
s.plotEnvironment('LineWidth', 2, 'Color', [.75 0 0])

n = 0;
while true
    tic
    
    n = n + 1;
    
    q = s.getPoses();
    
    if n < 500
        s.moveSingleIntegrators([-0.5*ones(1,N); 0.2*ones(1,N)])
    else
        s.goToPoints([linspace(-0.5,0.5,N); zeros(1,N)], 1)
    end
    
    s.plotRobots([0.2 0.4 0.6])
    
    pause(s.robots{1}.DT-toc)
end