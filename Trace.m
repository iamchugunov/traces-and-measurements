classdef Trace
    
    properties (Access = public)
        posts;                  % ���������� ������
        posts_number;           % ���������� ������
        sigma_n;                % ��� ����������
        X;                      % ������ ���������
        ToA;                    % ���������, ���
        ranges;                 % ���������
        mode;                   % ����� ������ 1-V-const, 2-V-const/acc-const/V-const, 3-V-const/V_ug-const/V-const
        start_time_str;         % ��������� �����
        start_time_sec;         % ��������� �����, ���
        T_period = 0.02;        % ������ ��������� (�����������), ���
        pause = 0;              % ��������� �������� ����� ������ ���������, ���
        N_periods = 100;        % ����� ��������, ������� ����� ���� ���������
        sigma_T = 0.;           % ��� �������� ��������, ���
        max_coord;              % ������� �� ������� ����
        max_V;                  % ������� �� ��������
        max_acc;                % ������� �� ����������
        V = 200;                % ������ ��������, �/�
        acc=5;                  % ���������
        V_ug = 0.01;           % ������� �������� ��� ��������
        angle;
        N_measurements = 200;   % ����� ���������
        c;                      % �������� �����, �/c
        hei;                    % ������ ���, �
        Measurements;           % ������ ��������� �� ������
        StateVector;            % ������ ��������� �� ������
        fligth_time = 600       % ����� ������, ���
       
    end
    
    methods (Access = public)
        function obj = Trace(config, start_time,mode) % �����������
           obj.mode=mode;
           obj.start_time_str = datestr(datetime(start_time),'dd.MM.yyyy HH:mm:ss.FFF');
           obj.start_time_sec = start_time(4)*3600 + start_time(5)*60 + start_time(6);
           obj.posts = config.posts;
           obj.posts_number = config.posts_number;
           obj.hei = config.hei;
           obj.c = config.c;
           obj.max_coord = config.max_coord;
           obj.max_V = config.max_V;
           obj.max_acc = config.max_acc;
           obj.sigma_n = config.sigma_t;
%            obj.Measurements = zeros(obj.posts_number,obj.N_measurements);
           obj = Initialization(obj);
%            obj.StateVector = zeros(length(obj.X),obj.N_measurements);
        end
        
        function obj = Go(obj)
            stop_time = obj.start_time_sec + obj.fligth_time;
            i = 1;
            time=obj.X(6);
            switch obj.mode
                case 1
                    
                while obj.X(6) < stop_time
                obj = obj.UpdateTrace_mode1;
                obj.StateVector(:,i) = obj.X;
                obj.Measurements(:,i) = obj.ToA;
                if obj.X(1) > abs(obj.max_coord) || obj.X(3) > abs(obj.max_coord)
                    break
                end
                i = i + 1;
                end
                
                case 2
                    
                    while obj.X(6)< stop_time
                if obj.X(6) < time + round(2*(stop_time-time)/3) && obj.X(6) > time + round((stop_time-time)/3)
                    obj = obj.UpdateTrace_mode2;
                else
                    obj = obj.UpdateTrace_mode1;
                 end
                obj.StateVector(:,i) = obj.X;
                obj.Measurements(:,i) = obj.ToA;
                if obj.X(1) > abs(obj.max_coord) || obj.X(3) > abs(obj.max_coord)
                    break
                end
                i = i + 1;
                    end
                    
                case 3
                while obj.X(6)< stop_time
                if obj.X(6) < time+round(2*(stop_time-time)/3) && obj.X(6) > time + round((stop_time-time)/3)
                    obj = obj.UpdateTrace_mode3;
                else
                    obj = obj.UpdateTrace_mode1;
                 end
                obj.StateVector(:,i) = obj.X;
                obj.Measurements(:,i) = obj.ToA;
                if obj.X(1) > abs(obj.max_coord) || obj.X(3) > abs(obj.max_coord)
                    break
                end
                i = i + 1;
                end
            end
            
        end
        
        function show(obj)
            plot(obj.posts(1,:),obj.posts(2,:),'^')
            hold on
            plot(obj.StateVector(1,:),obj.StateVector(3,:),'.')
            grid on
            axis([-obj.max_coord obj.max_coord -obj.max_coord obj.max_coord])
            daspect([1 1 1])
        end
        
    end
    
    methods (Access = public)
        
        function obj = Initialization(obj)
            obj.X(1,1) = randi([-obj.max_coord obj.max_coord]);
            obj.X(3,1) = randi([-obj.max_coord obj.max_coord]);
            obj.angle = randi([0 2*314])/100;
            obj.X(2,1) = obj.V * cos(obj.angle);
            obj.X(4,1) = obj.V * sin(obj.angle);
            obj.X(5,1) = obj.T_period;
            obj.X(6,1) = obj.start_time_sec + randi([0 obj.pause]);
        end
        
        function obj = UpdateTrace_mode1(obj)
            T = obj.X(5,1);
            F = [1 T 0 0 0 0;
                 0 1 0 0 0 0;
                 0 0 1 T 0 0;
                 0 0 0 1 0 0;
                 0 0 0 0 0 0;
                 0 0 0 0 1 1];
            obj.X = F*obj.X;
            obj.X(5) = obj.T_period * (1 + randi([0 obj.N_periods])) + normrnd(0,obj.sigma_T);
            
            for i = 1:obj.posts_number
                obj.ranges(i,1) = norm(obj.posts(:,i) - [obj.X(1); obj.X(3); obj.hei]);
                obj.ToA(i,1) = obj.X(6) + obj.ranges(i,1)/obj.c + normrnd(0,obj.sigma_n);
            end
        end
        
        function obj = UpdateTrace_mode2(obj)
            T = obj.X(5,1);
            F = [1 T 0 0 0 0;
                 0 1 0 0 0 0;
                 0 0 1 T 0 0;
                 0 0 0 1 0 0;
                 0 0 0 0 0 0;
                 0 0 0 0 1 1];
            obj.X = F*obj.X;
            
            %adding acceleration
            obj.X(2) = obj.X(2)+obj.acc*cos(obj.angle)*T;
            obj.X(4) = obj.X(4)+obj.acc*sin(obj.angle)*T;
            
            obj.X(5) = obj.T_period * (1 + randi([0 obj.N_periods])) + normrnd(0,obj.sigma_T);
            
            for i = 1:obj.posts_number
                obj.ranges(i,1) = norm(obj.posts(:,i) - [obj.X(1); obj.X(3); obj.hei]);
                obj.ToA(i,1) = obj.X(6) + obj.ranges(i,1)/obj.c + normrnd(0,obj.sigma_n);
            end
        end
        
        function obj = UpdateTrace_mode3(obj)
            T = obj.X(5,1);
            %increasing angle
            obj.angle=obj.angle+obj.V_ug;

             obj.X(1,1) = obj.X(1,1)+obj.X(2,1)*T;
             obj.X(2,1) = obj.V * cos(obj.angle);
             obj.X(3,1) = obj.X(3,1)+obj.X(4,1)*T;
             obj.X(4,1) = obj.V * sin(obj.angle);
             obj.X(6,1) = obj.X(5,1)+ obj.X(6,1);
             obj.X(5,1) = obj.T_period * (1 + randi([0 obj.N_periods])) + normrnd(0,obj.sigma_T);
            
            for i = 1:obj.posts_number
                obj.ranges(i,1) = norm(obj.posts(:,i) - [obj.X(1); obj.X(3); obj.hei]);
                obj.ToA(i,1) = obj.X(6) + obj.ranges(i,1)/obj.c + normrnd(0,obj.sigma_n);
            end
        end
        
    end
    
end

