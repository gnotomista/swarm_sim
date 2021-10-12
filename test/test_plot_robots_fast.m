clc
clear
close all

PLOT_VORONOI = true;
PLOT_CENTROID = true;
PLOT_DENSITY = true;
UNI_SI = 'si'; % 'uni' or 'si'
DENSITY = 'uniform'; % '' or 'uniform'

N = 1000;
robots = cell(1,N);
for i = 1 : N
    if strcmpi(UNI_SI,'uni')
        robots{i} = Unicycle('width',0.1,...
            'length',0.1,...
            'initialState',[0.1;-0.3;0]+0.001*rand(3,1),...
            'simulationTimeStep',0.01);
    elseif strcmpi(UNI_SI,'si')
        robots{i} = SingleIntegrator('width',0.05,...
            'initialState',[0.1;-0.5;0]+0.001*rand(3,1),...
            'simulationTimeStep',0.01);
    end
end
environment = 2*[cos(linspace(0,2*pi,6)); sin(linspace(0,2*pi,6))];
if strcmp(DENSITY, 'uniform')
    phi = 'uniform';
else
    phi = @(x,y) exp(-(x.^2+y.^2)/0.1);
end

s = Swarm('robots',robots,...
    'environment',environment,...
    'densityFunction',phi);

s.plotFigure()
s.plotEnvironment('LineWidth', 5, 'Color', [0 0 0])
if PLOT_DENSITY
    s.plotDensity(linspace(0,1.5,16), 'LineWidth', 2)
end

n = 0;
while true
    tic
    
    n = n + 1;
    
    q = s.getPoses();
    
    [G,A,VC] = s.coverageControl();
    
     s.setPoses(G);
    
    if mod(n,1) == 0
        s.plotRobotsFast(q, 1000, [0.2 0.4 0.6], '.');
    end
    
    drawnow limitrate
    pause(s.robots{1}.DT-toc)
end
