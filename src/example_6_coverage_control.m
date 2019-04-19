clc
clear
close all

PLOT_VORONOI = true;
PLOT_CENTROID = true;
PLOT_DENSITY = true;
UNI_SI = 'uni'; % 'uni' or 'si'
DENSITY = ''; % '' or 'uniform'

N = 50;
DT = 0.01;
T = 10;
robots = cell(1,N);
for i = 1 : N
    if strcmpi(UNI_SI,'uni')
        robots{i} = Unicycle('width',0.1,...
            'length',0.1,...
            'initialState',[-1;0.1;0]+0.001*rand(3,1),...
            'simulationTimeStep',DT);
    elseif strcmpi(UNI_SI,'si')
        robots{i} = SingleIntegrator('width',0.05,...
            'initialState',[-1;0.1;0]+0.01*rand(3,1),...
            'simulationTimeStep',DT);
    end
end
environment = 2*[cos(linspace(0,2*pi,6)); sin(linspace(0,2*pi,6))];
if strcmp(DENSITY, 'uniform')
    phi = 'uniform';
else
    phi = @(x,y) exp(-((x-1).^2+(y-0.6).^2)/0.3) + 0.5*exp(-((x-0.2).^2+(y+0.2).^2)/0.1);
end

s = Swarm('robots',robots,...
    'environment',environment,...
    'densityFunction',phi);

s.plotFigure()
if PLOT_DENSITY
    s.plotDensity(linspace(0,1.5,24), 'LineWidth', 2)
end
s.plotEnvironment('LineWidth', 5, 'Color', [0 0 0])

for t = 0 : DT : T
    tic
    
    q = s.getPoses();
    
    [G,A,VC] = s.coverageControl();
    
    s.goToPoints(G,25)
    
    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    if PLOT_VORONOI
        s.plotVoronoiCells(VC,'Color',[0.25 0.25 0.25],'LineWidth',2)
    end
    if PLOT_CENTROID
        s.plotCentroids(G,'.','Color',[0.5 0.5 0.5],'MarkerSize',20)
    end
    
    drawnow limitrate
    pause(DT-toc)
end
