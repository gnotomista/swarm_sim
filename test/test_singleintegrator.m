clc
clear
close all

si = SingleIntegrator('width',0.1,...
    'initialState',[1;1],...
    'simulationTimeStep',0.01);

figure, hold on, axis equal, axis([-2 2 -2 2])

n = 0;
while true
    tic
    
    n = n + 1;
    
    si.getPose()
    
    if n < 500
        si.moveSingleIntegrator([-0.3;-0.1])
    else
        si.goToPoint([1;-1], 2)
    end
    
    si.plotRobot([0 1 0])
    
    pause(si.DT-toc)
end