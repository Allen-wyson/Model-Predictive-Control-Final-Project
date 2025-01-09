        function plot_tracking_error(result, ref)
            % Calculate tracking error
            time = linspace(0, 15, length(result.myCar.X(1, :)));
            y_error = result.myCar.X(2, :) - ref(1);
            V_error = result.myCar.X(4, :) - ref(2);

            % Plot lateral position error
            figure;
            subplot(2, 1, 1);
            plot(time, y_error, 'r', 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Lateral Position Error [m]');
            title('Lateral Position Tracking Error');
            grid on;

            % Plot velocity error
            subplot(2, 1, 2);
            plot(time, V_error, 'b', 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Velocity Error [m/s]');
            title('Velocity Tracking Error');
            grid on;
        end