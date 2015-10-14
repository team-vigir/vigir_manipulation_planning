function name_joint_values( qs, robot_model, large_values_only, value_limit )
%NAME_JOINT_VALUES Summary of this function goes here
%   Detailed explanation goes here
            if ( nargin < 3 )
                large_values_only = false;
            end
            
            if ( nargin < 4 )
                value_limit = 0.1;
            end

            if ( large_values_only )
                fprintf('Showing only values larger than %.3f\n', value_limit);
            end
            
            for i = 1:length(robot_model.body)
                joint_name = robot_model.body(i).jointname;
                position_num = robot_model.body(i).position_num;
                
                if ( isempty(joint_name) )
                    continue;
                end
                
                if ( large_values_only && isempty(find(abs(qs(position_num)) > value_limit)))
                    continue;
                end
                    
                fprintf('%s:\t\t%s\n', joint_name, sprintf('%.3f   ', qs(position_num)'));
            end
end

