function animate(t_all,z_all,parms,video)

fps = video.fps;
pauseTime = video.pauseTime;
write = video.write;
name = video.name;

if write % if write == 1
    mov = VideoWriter(name); 
    open(mov);
end

%%%% Interpolate linearly using fps %%%%%
z_all_plot = [z_all(:,1) z_all(:,3)];
nn = size(z_all_plot,2);
total_frames = round(t_all(end)*fps);
t = linspace(0,t_all(end),total_frames);
z = zeros(total_frames,nn);
for i=1:nn
    z(:,i) = interp1(t_all,z_all_plot(:,i),t);
end



%%%%% Now animate the results %%%%%%% 
l1 = parms.l1;
l2 = parms.l2;
c1 = parms.c1;
c2 = parms.c2;
ll = 2.2*l1;

mm = size(z,1);
for i=1:mm
    
    theta1 = z(i,1); theta2 = z(i,2);
    O = [0 0];
    P = [l1*sin(theta1) -l1*cos(theta1)]; 
    G1 = [c1*sin(theta1) -c1*cos(theta1)];
    
    G2 = P + c2*[sin(theta1+theta2) -cos(theta1+theta2)];
    Q = P + l2*[sin(theta1+theta2) -cos(theta1+theta2)];
    
    h1 = plot(G1(1),G1(2),'ko','MarkerFaceColor','k','Markersize',10); hold on;
    h2 = plot(G2(1),G2(2),'ko','MarkerFaceColor','k','Markersize',10);
    h3 = line([O(1) P(1)],[O(2) P(2)],'Color','red','Linewidth',2);
    h4 = line([P(1) Q(1)],[P(2) Q(2)],'Color','red','Linewidth',2);
    
    axis('equal')
    axis([-ll ll -ll ll]);
    
    if (i==1)
        pause(0.1)
    end
    
    if write % if write == 1
        % axis off
        writeVideo(mov,getframe);
    end
    
    %delay to create animation
    pause(pauseTime);
   
   if (i~=mm)     %delete all but not the last entry
       delete(h1);
       delete(h2);
       delete(h3);
       delete(h4);
   end
   
   
end

if (write)
    close(mov);
end

end