classdef Unicycle < handle
    
    properties
        DT
        W
        L
        % VLINMAX
        % VANGMAX
        x
        y
        th
        hR
    end
    
    methods
        function obj = Unicycle(varargin)
            ip = inputParser;
            addParameter(ip, 'width', 1e-1)
            addParameter(ip, 'length', 1e-2)
            addParameter(ip, 'initialState', [0; 0; 0])
            addParameter(ip, 'simulationTimeStep', 1e-1)
            % addParameter(ip, 'vLinMax', 1e6)
            % addParameter(ip, 'vAngMax', 1e3*2*pi)
            parse(ip,varargin{:})
            
            obj.DT = ip.Results.simulationTimeStep;
            obj.W = ip.Results.width;
            obj.L = ip.Results.length;
            % obj.VLINMAX = ip.Results.vLinMax;
            % obj.VANGMAX = ip.Results.vAngMax;
            obj.x = ip.Results.initialState(1);
            obj.y = ip.Results.initialState(2);
            obj.th = ip.Results.initialState(3);
            
            obj.hR = [];
        end
        
        function q = getPose(obj)
            q = [obj.x; obj.y; obj.th];
        end
        
        function setPose(obj, q)
            obj.x = q(1);
            obj.y = q(2);
            obj.th = q(3);
        end
        
        function moveUnicycle(obj, vOmega)
            obj.x = obj.x + (vOmega(1)*cos(obj.th))*obj.DT;
            obj.y = obj.y + (vOmega(1)*sin(obj.th))*obj.DT;
            obj.th = obj.th + vOmega(2)*obj.DT;
            obj.th = atan2(sin(obj.th), cos(obj.th));
        end
        
        function moveSingleIntegrator(obj, v)
            vOmega = [1 0; 0 1/obj.L] * [cos(obj.th) sin(obj.th); -sin(obj.th) cos(obj.th)] * v;
            obj.moveUnicycle(vOmega)
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
        
        function goToPoint2(obj, p, varargin)
            if ~isempty(varargin)
                K = varargin{1};
            else
                K = [1; 10];
            end
            e = [norm(p - [obj.x; obj.y]);
                atan2(p(2) - obj.y, p(1) - obj.x)];
            vOmega = [K(1) * e(1) * cos(e(2) - obj.th); K(2) * e(1) * sin(e(2) - obj.th)];
            obj.moveUnicycle(vOmega)
        end
        
        function goToPose(obj, q, varargin)
            if ~isempty(varargin)
                K = varargin{1};
            else
                K = [1; 10; 100];
            end
            ex = (obj.x - q(1));
            ey = (obj.y - q(2));
            rho = sqrt(ex^2 + ey^2);
            gamma = atan2(ey, ex) - obj.th + pi;
            delta = gamma + obj.th - q(3);
            gamma = -pi + mod(gamma - pi, 2*pi);
            delta = -pi + mod(delta - pi, 2*pi);
            vOmega = [K(1) * rho * cos(gamma);
                K(2) * gamma + K(1) * sinc(gamma) * cos(gamma) * (gamma + K(3)*K(1)*delta)];
            obj.moveUnicycle(vOmega)
        end
        
        function plotRobot(obj, varargin)
            if ~isempty(varargin)
                args = varargin;
            else
                args = {[.5 .5 .5], 'EdgeColor', 'none'};
            end
            RT = [cos(obj.th - pi/2) -sin(obj.th - pi/2) obj.x;
                sin(obj.th - pi/2) cos(obj.th - pi/2) obj.y;
                0 0 1];
            rr = RT * [-obj.W/2 obj.W/2 0;
                -obj.W/2 -obj.W/2 obj.W;
                1 1 1];
            if isempty(obj.hR)
                obj.hR = patch(rr(1,:), rr(2,:), args{:});
            else
                set(obj.hR, 'XData', rr(1,:), 'YData', rr(2,:))
            end
            drawnow limitrate
        end
    end
    
end
