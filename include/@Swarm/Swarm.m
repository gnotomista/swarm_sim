classdef Swarm < handle
    
    properties
        N
        robots
        L
        environment
        phi
        hG
    end
    
    methods
        function obj = Swarm(varargin)
            ip = inputParser;
            addParameter(ip, 'robots', [])
            addParameter(ip, 'L', [])
            addParameter(ip, 'environment', [])
            addParameter(ip, 'densityFunction', 'uniform')
            parse(ip,varargin{:})
            
            obj.N = length(ip.Results.robots);
            obj.robots = ip.Results.robots;
            if obj.N == 1
                obj.L = 0;
            else
                obj.L = ip.Results.L;
                if isempty(obj.L)
                    obj.L = zeros(obj.N);
                elseif ~isnumeric(obj.L)
                    if obj.N < 3 && strcmp(obj.L,'cycle')
                        obj.L = 'line';
                    end
                    switch obj.L
                        case 'line'
                            obj.L = full(gallery('tridiag',obj.N,-1,2,-1));
                            obj.L(1,1) = 1;
                            obj.L(obj.N,obj.N) = 1;
                        case 'cycle'
                            obj.L = toeplitz([2;-1;zeros(obj.N-3,1);-1]);
                        case 'complete'
                            obj.L = obj.N*eye(obj.N)-ones(obj.N);
                    end
                else
                    assert(sqrt(numel(obj.L))==obj.N, 'Laplacian is not a square matrix whose dimension is the number of robots.')
                end
            end
            obj.environment = ip.Results.environment;
            if ~isempty(obj.environment)
                if norm(obj.environment(:,end)-obj.environment(:,1)) > 1e-3
                    obj.environment = [obj.environment obj.environment(:,1)];
                end
            end
            obj.phi = ip.Results.densityFunction;
            obj.hG = struct('figure',[],'graph',[],'env',[],'voronoiCells',[],'voronoiCentroids',[]);
        end
        
        function q = getPoses(obj)
            d = size(obj.robots{1}.getPose(),1);
            q = NaN(d,obj.N);
            for i = 1 : obj.N
                q(:,i) = obj.robots{i}.getPose();
            end
            if d == 2
                q = [q; zeros(1,obj.N)];
            end
        end
        
        function neighbors = getNeighbors(obj, idx)
            neighbors = find(obj.L(idx, :) ~= 0);
            neighbors = neighbors(neighbors~=idx);
        end
        
        function moveSingleIntegrators(obj, v)
            for i = 1 : obj.N
                obj.robots{i}.moveSingleIntegrator(v(:,i))
            end
        end
        
        function moveUnicycles(obj, v)
            for i = 1 : obj.N
                obj.robots{i}.moveUnicycle(v(:,i))
            end
        end
        
        function goToPoints(obj, p, varargin)
            for i = 1 : obj.N
                obj.robots{i}.goToPoint(p(:,i), varargin{:})
            end
        end
        
        function [G,A,VC] = coverageControl(obj,varargin)
            if isempty(varargin)
                p = eye(2,3)*obj.getPoses();
            else
                p = varargin{1};
            end
            P = [p obj.mirrorRobotsAboutEnvironmentBoundary(p)];
            [V,C] = voronoin(P');
            V(V==Inf) = 1e3*max(abs(obj.environment(:)));
            G = nan(2,obj.N);
            A = nan(1,obj.N);
            VC = cell(1,obj.N);
            for i = 1 : obj.N
                VC{i} = [V(C{i},1) V(C{i},2)]';
                [Gi, Ai] = obj.centroid(VC{i});
                G(:,i) = Gi;
                A(i) = Ai;
            end
        end
        
        function c = evaluateCoverageCost(obj, VC, varargin)
            p = eye(2,3)*obj.getPoses();
            c = 0;
            for i = 1 : length(VC)
                if isempty(varargin)
                    idx = i;
                else
                    idx = varargin{1};
                end
                P = VC{i};
                xP = P(1,:);
                yP = P(2,:);
                if strcmp(obj.phi, 'uniform')
                    f = @(x,y) norm(p(:,idx)-[x;y])^2;
                else
                    f = @(x,y) norm(p(:,idx)-[x;y])^2 * obj.phi(x,y);
                end
                trngltn = delaunay(xP, yP);
                ci = 0;
                for n = 1 : size(trngltn, 1)
                    ci = ci + Swarm.intOfFOverT(f, 8, P(:,trngltn(n,:)));
                end
                c = c + ci;
            end
        end
        
        function plotFigure(obj)
            obj.hG.figure = figure('units','normalized','position',[0 0 1 1],'MenuBar','none','ToolBar','none','NumberTitle','off');
            hold on, axis equal
            if ~isempty(obj.environment)
                axis([min(obj.environment(1,:))-1 max(obj.environment(1,:))+1 min(obj.environment(2,:))-1 max(obj.environment(2,:))+1])
            else
                axis([-1 1 -1 1])
            end
            set(gca,'Visible','off')
            obj.hG.figure.CurrentAxes.Clipping = 'off';
        end
        
        function plotRobots(obj, varargin)
            for i = 1 : obj.N
                obj.robots{i}.plotRobot(varargin{:})
            end
        end
        
        function plotGraph(obj, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'Color', [0 0 0], 'LineWidth', 2};
            end
            q = obj.getPoses();
            if isempty(obj.hG.graph)
                for i = 1 : obj.N
                    for j = obj.getNeighbors(i)
                        obj.hG.graph(end+1) = plot(zeros(1,2),zeros(1,2),args{:});
                    end
                end
            else
                edge_counter = 0;
                for i = 1 : obj.N
                    for j = obj.getNeighbors(i)
                        edge_counter = edge_counter + 1;
                        set(obj.hG.graph(edge_counter), 'XData', q(1,[i,j]), 'YData', q(2,[i,j]));
                    end
                end
            end
        end
        
        function plotEnvironment(obj, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'LineWidth', 5, 'Color', [0 0 0]};
            end
            obj.hG.env = plot(obj.environment(1,:), obj.environment(2,:), args{:});
        end
        
        function plotDensity(obj, varargin)
            if ~strcmp(obj.phi, 'uniform')
                [x,y] = meshgrid(min(obj.environment(1,:)):0.01:max(obj.environment(1,:)), min(obj.environment(2,:)):0.01:max(obj.environment(2,:)));
                z = obj.phi(x,y);
                caxis([min(z(:)),max(x(:))])
                if ~isempty(varargin)
                    args = varargin;
                else
                    args = {linspace(min(z(:)),max(x(:)),10), 'LineWidth', 2};
                end
                contour(x, y, z, args{:})
                obj.fillout(obj.environment(1,:),obj.environment(2,:),[min(obj.environment(1,:))-1 max(obj.environment(1,:))+1 min(obj.environment(2,:))-1 max(obj.environment(2,:))+1],0.94*[1 1 1]);
            end
        end
        
        function plotVoronoiCells(obj, VC, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'Color', [0 0 0], 'LineWidth', 2};
            end
            vcite = zeros(size(cell2mat(VC))+[0 length(VC)-1]);
            idx = 0;
            for i = 1 : length(VC)
                l_vcite = size(VC{i},2) + 1;
                if i == 1
                    vcite(:,1:l_vcite) = VC{i}(:,[1:end,1]);
                    idx = idx + l_vcite;
                else
                    vcite(:,idx+1:idx+l_vcite+1) = [NaN(2,1) VC{i}(:,[1:end,1])];
                    idx = idx + l_vcite + 1;
                end
            end
            if isempty(obj.hG.voronoiCells)
                obj.hG.voronoiCells = plot(vcite(1,:), vcite(2,:), args{:});
            else
                set(obj.hG.voronoiCells, 'XData', vcite(1,:), 'YData', vcite(2,:))
            end
        end
        
        function plotCentroids(obj, G, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {'.', 'Color', [0 0 0], 'MarkerSize', 10};
            end
            if isempty(obj.hG.voronoiCentroids)
                obj.hG.voronoiCentroids = plot(G(1,:), G(2,:), args{:});
            else
                set(obj.hG.voronoiCentroids, 'XData', G(1,:), 'YData', G(2,:))
            end
        end
    end
    
    methods (Access = private)
        function mirroredRobots = mirrorRobotsAboutEnvironmentBoundary(obj, p)
            mirroredRobots = nan(2,size(p,2)*(size(obj.environment,2)-1));
            for i = 1 : size(p,2)
                point = p(:,i);
                for j = 1 : size(obj.environment,2)-1
                    pointWrtSide = (point - obj.environment(:,j));
                    side = obj.environment(:,j+1) - obj.environment(:,j);
                    lengthOfPProjectedOntoL = pointWrtSide' * side / norm(side)^2;
                    projectedVector = obj.environment(:,j) + lengthOfPProjectedOntoL * side;
                    mirroredRobots(:,(i-1)*(size(obj.environment,2)-1)+j) = point - 2 * (point - projectedVector);
                end
            end
        end
        
        function [G, A] = centroid(obj, P)
            if strcmp(obj.phi, 'uniform')
                n = length(P);
                M = [0 1;-1 0];
                A = 0;
                S = 0;
                for i = 1 : n
                    ri = P(:,i);
                    if i < n
                        j = i + 1;
                    else
                        j = 1;
                    end
                    rj = P(:,j);
                    rjo = M * rj;
                    A = A + ri'*rjo;
                    S = S + (ri' * rjo * (ri + rj));
                end
                A = A / 2;
                S = S / 6;
                G = S / A;
            else
                xP = P(1,:);
                yP = P(2,:);
                phiA = @(x,y) (obj.phi(x,y));
                phiSx = @(x,y) (x.*obj.phi(x,y));
                phiSy = @(x,y) (y.*obj.phi(x,y));
                trngltn = delaunay(xP, yP);
                A = 0;
                S = 0;
                for i = 1 : size(trngltn, 1)
                    A = A + Swarm.intOfFOverT(phiA, 8, P(:,trngltn(i,:)));
                    S = S + [Swarm.intOfFOverT(phiSx, 8, P(:,trngltn(i,:)));
                        Swarm.intOfFOverT(phiSy, 8, P(:,trngltn(i,:)))];
                end
                G = S / A;
            end
        end
    end
    
    methods (Static)
        function I = intOfFOverT(f, N, T)
            x1 = T(1,1);
            x2 = T(1,2);
            x3 = T(1,3);
            y1 = T(2,1);
            y2 = T(2,2);
            y3 = T(2,3);
            xyw = Swarm.TriGaussPoints(N);
            A = abs(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2;
            NP = size(xyw(:,1), 1);
            I = 0;
            for j = 1 : NP
                x = x1*(1-xyw(j,1)-xyw(j,2))+x2*xyw(j,1)+x3*xyw(j,2);
                y = y1*(1-xyw(j,1)-xyw(j,2))+y2*xyw(j,1)+y3*xyw(j,2);
                I = I + f(x,y)*xyw(j,3);
            end
            I = A*I;
        end
        
        function xw = TriGaussPoints(n)
            % https://github.com/FMenhorn/BGCEGit/blob/master/Prototypes/MATLAB/Sandbox/BennisChaos/marchingCubes/L2Projection/TriGaussPoints.m
            xw = zeros(n,3);
            if n==1
                xw = [0.33333333333333 0.33333333333333 1.00000000000000];
            elseif n==2
                xw = [0.16666666666667 0.16666666666667 0.33333333333333
                    0.16666666666667 0.66666666666667 0.33333333333333
                    0.66666666666667 0.16666666666667 0.33333333333333];
            elseif n==3
                xw = [0.33333333333333 0.33333333333333 -0.56250000000000
                    0.20000000000000 0.20000000000000 0.52083333333333
                    0.20000000000000 0.60000000000000 0.52083333333333
                    0.60000000000000 0.20000000000000 0.52083333333333];
            elseif n==4
                xw = [0.44594849091597 0.44594849091597 0.22338158967801
                    0.44594849091597 0.10810301816807 0.22338158967801
                    0.10810301816807 0.44594849091597 0.22338158967801
                    0.09157621350977 0.09157621350977 0.10995174365532
                    0.09157621350977 0.81684757298046 0.10995174365532
                    0.81684757298046 0.09157621350977 0.10995174365532];
            elseif n==5
                xw = [0.33333333333333 0.33333333333333 0.22500000000000
                    0.47014206410511 0.47014206410511 0.13239415278851
                    0.47014206410511 0.05971587178977 0.13239415278851
                    0.05971587178977 0.47014206410511 0.13239415278851
                    0.10128650732346 0.10128650732346 0.12593918054483
                    0.10128650732346 0.79742698535309 0.12593918054483
                    0.79742698535309 0.10128650732346 0.12593918054483];
            elseif n==6
                xw = [0.24928674517091 0.24928674517091 0.11678627572638
                    0.24928674517091 0.50142650965818 0.11678627572638
                    0.50142650965818 0.24928674517091 0.11678627572638
                    0.06308901449150 0.06308901449150 0.05084490637021
                    0.06308901449150 0.87382197101700 0.05084490637021
                    0.87382197101700 0.06308901449150 0.05084490637021
                    0.31035245103378 0.63650249912140 0.08285107561837
                    0.63650249912140 0.05314504984482 0.08285107561837
                    0.05314504984482 0.31035245103378 0.08285107561837
                    0.63650249912140 0.31035245103378 0.08285107561837
                    0.31035245103378 0.05314504984482 0.08285107561837
                    0.05314504984482 0.63650249912140 0.08285107561837];
            elseif n==7
                xw = [0.33333333333333 0.33333333333333 -0.14957004446768
                    0.26034596607904 0.26034596607904 0.17561525743321
                    0.26034596607904 0.47930806784192 0.17561525743321
                    0.47930806784192 0.26034596607904 0.17561525743321
                    0.06513010290222 0.06513010290222 0.05334723560884
                    0.06513010290222 0.86973979419557 0.05334723560884
                    0.86973979419557 0.06513010290222 0.05334723560884
                    0.31286549600487 0.63844418856981 0.07711376089026
                    0.63844418856981 0.04869031542532 0.07711376089026
                    0.04869031542532 0.31286549600487 0.07711376089026
                    0.63844418856981 0.31286549600487 0.07711376089026
                    0.31286549600487 0.04869031542532 0.07711376089026
                    0.04869031542532 0.63844418856981 0.07711376089026];
            elseif n==8
                xw = [0.33333333333333 0.33333333333333 0.14431560767779
                    0.45929258829272 0.45929258829272 0.09509163426728
                    0.45929258829272 0.08141482341455 0.09509163426728
                    0.08141482341455 0.45929258829272 0.09509163426728
                    0.17056930775176 0.17056930775176 0.10321737053472
                    0.17056930775176 0.65886138449648 0.10321737053472
                    0.65886138449648 0.17056930775176 0.10321737053472
                    0.05054722831703 0.05054722831703 0.03245849762320
                    0.05054722831703 0.89890554336594 0.03245849762320
                    0.89890554336594 0.05054722831703 0.03245849762320
                    0.26311282963464 0.72849239295540 0.02723031417443
                    0.72849239295540 0.00839477740996 0.02723031417443
                    0.00839477740996 0.26311282963464 0.02723031417443
                    0.72849239295540 0.26311282963464 0.02723031417443
                    0.26311282963464 0.00839477740996 0.02723031417443
                    0.00839477740996 0.72849239295540 0.02723031417443];
            end
        end
        
        function h = fillout(x,y,lims,varargin)
            % MMA 23-3-2006, mma@odyle.net
            h = [];
            if nargin <2
                disp(['## ',mfilename,' : more input arguments required']);
                return
            end
            
            if numel(x) > length(x)
                x = Swarm.var_border(x);
            end
            if numel(y) > length(y)
                y = Swarm.var_border(y);
            end
            if length(x) ~= length(y)
                disp(['## ',mfilename,' : x and y must have the same size']);
                return
            end
            if nargin<4
                varargin={'g'};
            end
            if nargin<3
                lims=[min(x) max(x) min(y) max(y)];
            else
                if lims(1) > min(x), lims(1)=min(x); end
                if lims(2) < max(x), lims(2)=max(x); end
                if lims(3) > min(y), lims(3)=min(y); end
                if lims(4) < max(y), lims(4)=max(y); end
            end
            xi=lims(1); xe=lims(2);
            yi=lims(3); ye=lims(4);
            i=find(x==min(x)); i=i(1);
            x=x(:);
            y=y(:);
            x=[x(i:end)' x(1:i-1)' x(i)];
            y=[y(i:end)' y(1:i-1)' y(i)];
            x=[xi   xi xe xe xi xi   x(1) x];
            y=[y(1) ye ye yi yi y(1) y(1) y];
            h=fill(x,y,varargin{:});
            set(h,'edgecolor','none');
        end
        
        function [x,xc] = var_border(M)
            % MMA 18-8-2004, martinho@fis.ua.pt
            x  = [];
            xc = [];
            if nargin == 0
                return
            end
            xl = M(:,1);
            xt = M(end,:);  xt = xt';
            xr = M(:,end);  xr = flipud(xr);
            xb = M(1,:);    xb = flipud(xb');
            x =  [xl; xt; xr; xb];
            xc =  [xl(1) xl(end) xr(1) xr(end)];
        end
    end
end
