function [ts,total_error,broke] = simulate_dev(gains,visualize,write_to_video)
    % Run simulation:
    init
    [ts,total_error,broke,ii,vl_rv,r,u] = simulate(gains);
    total_error(total_error == 0) = [];
    
    %% Visualize (Optional):
    if nargin == 1
        visualize = false;
        write_to_video = false;
    elseif nargin == 2
        write_to_video = false;
    end
    L = ii;
    
    if visualize
        if write_to_video
           vid = VideoWriter('flock_animation.avi'); 
           open(vid)
        end

        figure('units','normalized','outerposition',[0 0 1 1])
        % Initialize the animation:
        jj = 1;
        virtual_leader = plot(vl_rv(1,1),vl_rv(2,1),'sr','MarkerSize',10,'MarkerFaceColor','r'); hold on
        obstacles = circle(obs(1), obs(2), obs(3));

        num_agents = size(r,1)/2;
        agents = gobjects(num_agents,1);
        for ii = 1:2:2*num_agents
            agents(jj) = plot(r(ii,1),r(ii+1,1),'.k','MarkerSize',20);
            jj = jj+1;
        end
        r_vec = reshape(r(:,ii)',2,[])';
        u_vec = reshape(u(:,ii)',2,[])';
        actuations = quiver(r_vec(:,1),r_vec(:,2),u_vec(:,1),u_vec(:,2),'color','r');
        axis equal
        grid on
        legend([virtual_leader, agents(1),obstacles(1)],...
               'Virtual Leader','Agents','Obstacles',...
               'location','northwest')

        % Actuall show the animation:
        for ii = 1:L
            % Add track history:
            r_vec = reshape(r(:,ii)',2,[])';
            u_vec = reshape(u(:,ii)',2,[])';
            plot(r_vec(:,1),r_vec(:,2),'.','color',[.5 .5 .5],'MarkerSize',5); hold on
            for jj = 1:num_agents
                set(agents(jj),'XData',r_vec(jj,1),'YData',r_vec(jj,2));
            end
            set(actuations,'XData',r_vec(:,1),'YData',r_vec(:,2),...
                           'UData',max_u./u_vec(:,1),'VData',max_u./u_vec(:,2));
            set(virtual_leader,'XData',vl_rv(1,ii),'YData',vl_rv(2,ii));
%             WIDTH = max([abs(vl_rv(1,1)), abs(vl_rv(1,end))]);
%             xlim([-WIDTH WIDTH])
%             ylim([-WIDTH WIDTH])
            
            drawnow
            if write_to_video
                frame = getframe(gcf);
                writeVideo(vid,frame);
            end
        end
        if write_to_video
            close(vid)
        end
    end
end