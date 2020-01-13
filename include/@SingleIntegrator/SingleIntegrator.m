classdef SingleIntegrator < handle
    
    properties
        DT
        W
        % VLINMAX
        x
        y
        hR
    end
    
    methods
        function obj = SingleIntegrator(varargin)
            ip = inputParser;
            addParameter(ip, 'width', 1e-1)
            addParameter(ip, 'initialState', [0; 0])
            addParameter(ip, 'simulationTimeStep', 1e-1)
            % addParameter(ip, 'vLinMax', 1e6)
            parse(ip,varargin{:})

            obj.W = ip.Results.width;
            obj.DT = ip.Results.simulationTimeStep;
            % obj.VLINMAX = ip.Results.vLinMax;
            obj.x = ip.Results.initialState(1);
            obj.y = ip.Results.initialState(2);
            
            obj.hR = [];
        end
        
        function q = getPose(obj)
            q = [obj.x; obj.y];
        end
        
        function setPose(obj, q)
            obj.x = q(1);
            obj.y = q(2);
        end
        
        function moveSingleIntegrator(obj, v)
            obj.x = obj.x + v(1)*obj.DT;
            obj.y = obj.y + v(2)*obj.DT;
        end
        
        function goToPoint(obj, p, varargin)
            if ~isempty(varargin)
                K = varargin{1};
            else
                K = 1;
            end
            v = K * (p - [obj.x; obj.y]);
            obj.moveSingleIntegrator(v)
        end
        
        function plotRobot(obj, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {[.5 .5 .5], 'EdgeColor', 'none'};
            end
            rr = [obj.x; obj.y] + obj.W*[cos(linspace(0,2*pi,100));
                sin(linspace(0,2*pi,100))];
            if isempty(obj.hR)
                obj.hR = patch(rr(1,:), rr(2,:), args{:});
            else
                set(obj.hR, 'XData', rr(1,:), 'YData', rr(2,:))
            end
            drawnow limitrate
        end
    end
    
end
