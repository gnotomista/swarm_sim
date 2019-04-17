clc
clear
close all

PLOT_VORONOI = true;
PLOT_CENTROID = true;
PLOT_DENSITY = true;
UNI_SI = 'uni';
DENSITY = ''; % or 'uniform'

N = 50;
robots = cell(1,N);
for i = 1 : N
    if strcmpi(UNI_SI,'uni')
        robots{i} = Unicycle('width',0.1,...
            'length',0.1,...
            'initialState',[0.1;-0.3;0]+0.001*rand(3,1),...
            'simulationTimeStep',0.01,...
            'vLinMax',10,...
            'vAngMax',1);
    elseif strcmpi(UNI_SI,'si')
        robots{i} = SingleIntegrator('width',0.05,...
            'initialState',[0.1;-0.5;0]+0.001*rand(3,1),...
            'simulationTimeStep',0.01,...
            'vLinMax',10);
    end
end
environment = 2*[cos(linspace(0,2*pi,6)); sin(linspace(0,2*pi,6))];
if strcmp(DENSITY, 'uniform')
    phi = 'uniform';
else
    phi = @(x,y) exp(-((x-0.4).^2+(y-0.6).^2)/0.3) + 0.5*exp(-((x+0.4).^2+(y+0.2).^2)/0.15);
end

s = Swarm('robots',robots,...
    'environment',environment,...
    'densityFunction',phi);

s.plotFigure()
if PLOT_DENSITY
    s.plotDensity(linspace(0,1.5,24), 'LineWidth', 2)
end
s.plotEnvironment('LineWidth', 10, 'Color', [0 0 0])

for t = 1 : 1e3
    tic
    
    q = s.getPoses();
    
    [G,A,VC] = s.coverageControlFast();
    
    s.goToPoints(G,100);
    
    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    if PLOT_VORONOI
        s.plotVoronoiCells(VC,'Color',[0.25 0.25 0.25],'LineWidth',2)
    end
    if PLOT_CENTROID
        s.plotCentroids(G,'.','Color',[0.5 0.5 0.5],'MarkerSize',20)
    end
    
    drawnow limitrate
    pause(s.robots{1}.DT-toc)
end
