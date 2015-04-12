function position = setRelativeTargetPosition(diff_left, diff_front, diff_height, mat) 
            pos_vec = [diff_front, diff_left, diff_height, 1]';
            res_vec = mat*pos_vec;
            position.x = res_vec(1);
            position.y = res_vec(2);
            position.z = res_vec(3);
end