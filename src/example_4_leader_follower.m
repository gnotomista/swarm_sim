clc
clear
close all

PLOT_GRAPH = true;

Nf = 7;
Nl = 5;
N = Nf+Nl;
DT = 0.01;
T = 5;

robots = cell(1,N);
for i = 1 : N
    robots{i} = SingleIntegrator('width',0.05,...
        'initialState',-1+2*rand(3,1),...
        'simulationTimeStep',DT);
end

s = Swarm('robots',robots,'L','cycle');

L = kron(s.L,eye(2));
Lf = L(1:2*Nf,1:2*Nf);
l = L(1:2*Nf,2*Nf+1:end);
A = [-Lf -l; zeros(2*Nl,2*N)];
B = [zeros(2*Nf,2*Nl); eye(2*Nl)];
Q = eye(2*N);
R = eye(2*Nl);
Qtilde = A'*Q*A;
Rtilde = B'*Q*B + R;

X0 = [linspace(-0.5,0.5,N);
    zeros(1,N)];
XT = [0.25*[cos(linspace(0,2*pi-2*pi/Nf,Nf));
    sin(linspace(0,2*pi-2*pi/Nf,Nf))],0.5*[cos(linspace(0,2*pi-2*pi/Nl,Nl));
    sin(linspace(0,2*pi-2*pi/Nl,Nl))]];

x0 = reshape(X0,2*N,1);
xT = reshape(XT,2*N,1);

M = [A -1/2*B*inv(Rtilde)*B';
    -2*Qtilde -A'];
EMT = expm(M*T);
N1 = EMT(1:2*N,1:2*N);
N2 = EMT(1:2*N,2*N+1:end);
lambda0 = N2\(xT-N1*x0);

xlambda0 = [x0; lambda0];
x = x0;

q = s.getPoses();
while max(max(abs(q(1:2,:)-X0))) > 0.01
    s.goToPoints(X0)
    q = s.getPoses();
end

s.plotFigure()
if PLOT_GRAPH
    s.plotGraph('Color',[0,0.145,0.298],'LineWidth',5)
end
s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
for i = Nf+1:N
    s.robots{i}.hR.FaceColor = [0.702,0.639,0.412];
end

for t = 0 : DT : T
    tic
    
    xlambda = expm(M*t)*xlambda0;
    X = reshape(xlambda(1:2*N),2,N);
    
    s.goToPoints(X, 100)
    
    if PLOT_GRAPH
        s.plotGraph('Color',[0,0.145,0.298],'LineWidth',5)
    end
    s.plotRobots([0.933,0.698,0.067],'EdgeColor','none')
    
    drawnow limitrate
    pause(DT-toc)
end




