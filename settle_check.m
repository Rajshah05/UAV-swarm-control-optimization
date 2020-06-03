function [settled,total_error] = settle_check(v,vl)
    allowable_error = 0.05;
   
    settled = false;
    
    % Reformatting:
    v_vec = reshape(v',2,[])';
    
    % Calculate error signals:
    [~,e_via1] = normr(v_vec(1,:) - v_vec(2:end,:));
    [~,e_via2] = normr(v_vec(2,:) - v_vec(3:end,:));
    e_via3 = norm(v_vec(3,:) - v_vec(4,:)); % error for 3
    [~,vl_error] = normr(v_vec - vl(3:4));
    ia_error = [e_via3; e_via2; e_via1];
    total_error = norm(ia_error) + norm(vl_error);
    
    % Must all be past the obstacle (heading away from it):
    if total_error < allowable_error
        settled = true;
    end
end