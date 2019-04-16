clc
clear
close all

dd = Unicycle('width',0.1,...
    'length',0.1,...
    'initialState',[0;0;-pi/2],...
    'simulationTimeStep',0.01,...
    'vLinMax',0.1,...
    'vAngMax',1);

figure, hold on, axis equal, axis([-2 2 -2 2])

n = 0;
while true
    tic
    
    n = n + 1;
    
    dd.getPose()
    
    if n < 100
        dd.moveUnicycle([-1;1])
    elseif n < 200
        dd.moveSingleIntegrator([-1;0.01])
    elseif n < 700
        dd.goToPoint([1;1], 1)
    elseif n < 1500
        dd.goToPoint2([-1;1], [1 10])
    else
        dd.goToPose([-1;0;pi], [1 10 100])
    end
    
    dd.plotRobot([1 0 0],'EdgeColor','none')
    
    pause(dd.DT-toc)
end