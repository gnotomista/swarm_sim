clc
clear
close all

N = 50;
robots = cell(1,N);
environment = [cos(linspace(0,2*pi,6)); sin(linspace(0,2*pi,6))];
s = Swarm('robots',robots,...
    'environment',environment,...
    'densityFunction','uniform');

p = -0.5+rand(2,N);

[G,A,VC] = s.coverageControl(p);

s.plotFigure()
s.plotEnvironment('LineWidth', 5, 'Color', [0 0 0])
scatter(p(1,:),p(2,:),100,'k','.')
s.plotVoronoiCells(VC,'Color',[0.5 0 0.5],'LineWidth',2)
s.plotCentroids(G,'.','Color',[0.75 0 0.75],'MarkerSize',10)