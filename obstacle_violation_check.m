function [violation] = obstacle_violation_check(r,obs,d_min)
    violation = false;
    r_vec = reshape(r',2,[])';
    for jj = 1:size(obs,1)
        [~,obs_d] = normr(r_vec - obs(jj,1:2));
        if any(obs_d < d_min(jj))
            violation = true;
            break
        end   
    end
end