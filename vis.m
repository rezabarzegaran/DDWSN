% This script visualizes the results
clear all
close all
clc
load("TCs\Cat 2\T1\TC.mat")
% This script is meant to visualize the results
%video recording
recvideo = true;
% Prepare video
if recvideo
    videoWriterObj = VideoWriter('TC.mp4','MPEG-4');
    videoWriterObj.FrameRate = 10;
    videoWriterObj.Quality = 100;
    open(videoWriterObj);
end


%% PLOT
% Set up the main plot
cmap = hsv(N);
panel = figure;
panel.Position = [50 5 1200 900];
panel.Color = 'white';
sgtitle('CPCC WSN Simulator');
% Set up subplots
%errPlotParent = subplot(4,4,1);
%title("Tracking Error")
%hold on
%errPlotParent.XTick = [];
%xlabel("time")
%ylabel("error")
%errPlotParent.Position = [0.03 0.65 0.25 0.25];
%errPlotParent.YLim = [0 round(norm(Omega_Bounds(:,2)))+1];
%errPlotParent.Clipping = 'off';

% Axis for drone error
%for n=1:N
    %linename = strcat('Drone',num2str(n));
    %errplot(n) = plot(errPlotParent,NaN,NaN,'Color',cmap(n,:),'LineWidth',1.5,'DisplayName',linename); 
%end
%errlgd = legend;
%errlgd.NumColumns = 2;
%axErr.YTick = [];
%axPend.Visible = 'off';
%axPend.Clipping = 'off';
%axis equal
%axis([-1.2679 1.2679 -1 1]);
%plot(0.001,0,'.k','MarkerSize',50);
%hold off
trajplotParent=subplot(4,3,[1,2,3,4,5,6,7,8, 9]);
hold on
title('3D view')
trajplotParent.XLim = X_bound ;
trajplotParent.YLim = Y_bound ;
trajplotParent.ZLim = Z_bound ;
xlabel('X', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Y', 'FontSize', 14, 'FontWeight', 'bold');
zlabel('Z', 'FontSize', 14, 'FontWeight', 'bold');
ax = gca;
ax.XAxis.FontSize = 12;
ax.XAxis.FontWeight = 'bold';
ax.YAxis.FontSize = 12;
ax.YAxis.FontWeight = 'bold';
ax.ZAxis.FontSize = 12;
ax.ZAxis.FontWeight = 'bold';

[YContour,XContour] = meshgrid(xDomain,yDomain);
for i=1:size(xDomain,2)
    for j=1:size(yDomain,2)
    eventDistribution(i,j)= probabDist(xDomain(i),yDomain(j));
    
    end
end
colormap(parula)
s = contourf(XContour, YContour, 200*eventDistribution,"FaceAlpha",0.8);  % plots a 3D surface

obstaclePlot = [];
for i=1:length(obstacles)
    [Obstacle_x,Obstacle_y,Obstacle_z] = sphere;
    Obstacle_x = Obstacle_x * obstacles(i).distance;
    Obstacle_y = Obstacle_y * obstacles(i).distance;
    Obstacle_z = Obstacle_z * obstacles(i).distance;
    linename = strcat('Obstacle',num2str(i));
    obstaclePlot(i) = surf(trajplotParent,Obstacle_x+obstacles(i).x,Obstacle_y+obstacles(i).y,Obstacle_z+obstacles(i).z,'FaceColor',"red",'DisplayName',linename); 
end
for n=1:N
    linename = strcat('Drone',num2str(n));
    trajplot(n) = plot3(trajplotParent,NaN,NaN,NaN,'Color',cmap(n,:),'LineWidth',1.5,'DisplayName',linename);
    droneObject(n) = MakeDrone(gca);
    droneViewObject(n) = makeDroneView(gca, drone(n).s_param(8));
end
%errlgd = legend;
%errlgd.NumColumns = 2;
%map = imagesc(reshape(R,[length(x2),length(x1)]));
%axMap = map.Parent;
%axMap.XTickLabels = {'-pi' '0' 'pi'};
%axMap.XTick = [1 floor(length(x1)/2) length(x1)];
%axMap.YTickLabels = {'-pi' '0' 'pi'};
%axMap.YTick = [1 floor(length(x2)/2) length(x2)];
%axMap.XLabel.String = 'Angle';
%axMap.YLabel.String = 'Angular Rate';
%axMap.Visible = 'on';
%axMap.Color = [0.3 0.3 0.5];
%axMap.XLim = [1 length(x1)];
%axMap.YLim = [1 length(x2)];
%axMap.Box = 'off';
%axMap.FontSize = 14;
%caxis([3*min(R),max(R)])
%pathmap = plot(NaN,NaN,'.g','MarkerSize',30); 
%map.CData = V;
trajectoryplotObjs = [obstaclePlot,trajplot,droneObject,droneViewObject];
hold off
view(trajplotParent,135,45);
% Connectivity Graph
%connectivityPlotParent = subplot(4,4,13);
%title('Connectivity')
%hold on
%connectivityPlotParent.XTick = [];
%connectivityPlotParent.YTick = [];
%connectivityPlotParent.XLim = X_bound;
%connectivityPlotParent.YLim = Y_bound;
%h = gca;
%h.XAxis.Visible = 'off';
%h.YAxis.Visible = 'off';
%xlabel("time")
%ylabel("edges")
%connectivityPlotParent.Clipping = 'off';
% Axis for edges
%for n=1:N
 %   linename = strcat('Drone',num2str(n));
 %   connplot(n) = scatter(connectivityPlotParent,NaN,NaN,"filled",'Color',cmap(n,:),'DisplayName',linename); 
%end
%for n=N+1:(N*(N+1)/2)+N+1
    %connplot(n) = plot(connectivityPlotParent,NaN,NaN,'Color',cmap(n,:),'DisplayName',linename); 
%end
%edgelgd = legend;
%errlgd.NumColumns = 2;
%axErr.YTick = [];
%axPend.Visible = 'off';
%axPend.Clipping = 'off';
%axis equal
%axis([-1.2679 1.2679 -1 1]);
%plot(0.001,0,'.k','MarkerSize',50);
%hold off

%Coverage Performance
%coveragePlotParent = subplot(4,4,9);
%hold on
%title('Coverage')
%coveragePlotParent.XLim = X_bound;
%coveragePlotParent.XTick = [];
%coveragePlotParent.YLim = Y_bound;
%coveragePlotParent.YTick = [];
%xlabel('X')
%ylabel('Y')
%[coverageplot,coverageplotdata]=contourf(coveragePlotParent,XContour, YContour, zeros(size(XContour)),"FaceAlpha",0.8);  % plots a 3D surface
%set(coverageplot,'XData',XContour);
%set(coverageplot,'YData',YContour);
%legend('Location','southeast');
%hold off


%Collision Monitor
%collisionPlotParent = subplot(4,4,13);
%title('Collision')

% X-Y trajectory
O = length(obstacles);
XYtrajPlotParent = subplot(4,3,10);
title('X-Y view')
XYtrajPlotParent.XLim = X_bound;
XYtrajPlotParent.YLim = Y_bound;
XYtrajPlotParent.ZLim = Z_bound;
copyobj(trajectoryplotObjs,XYtrajPlotParent);
XYtrajPlotObjects = get(XYtrajPlotParent,'Children');
XYtrajectoryPlot = XYtrajPlotObjects(O+1:O+N);
XYDroneObjects = XYtrajPlotObjects(O+N+1:O+N+N);
XYDroneViewObjects = XYtrajPlotObjects(O+N+N+1:end);
view(XYtrajPlotParent,0,90);
%X-Z trajectory
XZtrajPlotParent = subplot(4,3,11);
title('X-Z view')
XZtrajPlotParent.XLim = X_bound;
XZtrajPlotParent.YLim = Y_bound;
XZtrajPlotParent.ZLim = Z_bound;
copyobj(trajectoryplotObjs,XZtrajPlotParent);
XZtrajPlotObjects = get(XZtrajPlotParent,'Children');
XZtrajectoryPlot = XZtrajPlotObjects(O+1:O+N);
XZDroneObjects = XZtrajPlotObjects(O+N+1:O+N+N);
XZDroneViewObjects = XZtrajPlotObjects(O+N+N+1:end);
view(XZtrajPlotParent,0,0);

%Y-Z trajectory
YZtrajPlotParent = subplot(4,3,12);
title('Y-Z view')
YZtrajPlotParent.XLim = X_bound;
YZtrajPlotParent.YLim = Y_bound;
YZtrajPlotParent.ZLim = Z_bound;
copyobj(trajectoryplotObjs,YZtrajPlotParent);
YZtrajPlotObjects = get(YZtrajPlotParent,'Children');
YZtrajectoryPlot = YZtrajPlotObjects(O+1:O+N);
YZDroneObjects = YZtrajPlotObjects(O+N+1:O+N+N);
YZDroneViewObjects = YZtrajPlotObjects(O+N+N+1:end);
view(YZtrajPlotParent,90,0);
Ittmax = 300;
for step=1:1:Ittmax
    if ~ishandle(panel)
        if recvideo
            close(videoWriterObj);
        end
        break;
    end
    if recvideo
        frame = getframe(gcf);
        writeVideo(videoWriterObj,frame);
    end
    for n = 1:N
        P(n,:) = drone(n).position{step};
        uu(n,:) = drone(n).control{step};
        err(n,:) = P(n,:) - uu(n,:);
    end
    
    updateTraj(droneViewObject,droneObject,trajplot,P);
    updateTraj(XYDroneViewObjects,XYDroneObjects,XYtrajectoryPlot,P);
    updateTraj(XZDroneViewObjects,XZDroneObjects,XZtrajectoryPlot,P);
    updateTraj(YZDroneViewObjects,YZDroneObjects,YZtrajectoryPlot,P);
    %updateConnectivity(connplot,step,P);
    drawnow;
end
    

if recvideo
    close(videoWriterObj);
end

function updateTraj(combiedviewObject, combinedobject,h,p)
%hold on 

    for n =1:length(h)
        translation = makehgtform('translate',p(n,:));
        scale = makehgtform('scale',p(n,3));
        set(combinedobject(n),'matrix',translation);
        set(combiedviewObject(n),'matrix',translation*scale);
        plotX = get(h(n),'XData');
        plotX(end+1)=p(n,1);
        plotY = get(h(n),'YData'); 
        plotY(end+1)=p(n,2);
        plotZ = get(h(n),'ZData'); 
        plotZ(end+1)=p(n,3);
        set(h(n),'XData',plotX);
        set(h(n),'YData',plotY);
        set(h(n),'ZData',plotZ);
        hold on
        hold off
    end
%hold off
end


function updateErr(h,step,p)
    for n =1:length(h)
        plotX = get(h(n),'XData');
        plotX(end+1)=step;
        set(h(n),'XData',plotX);
        plotY = get(h(n),'YData');
        plotY(end+1)=norm(p(n,:));
        set(h(n),'YData',plotY);
    end

end

function updateConnectivity(h,step,p)
    for n =1:N
        plotX = get(h(n),'XData');
        %plotX(end+1)=step;
        plotX(1)=p(n,1);
        set(h(n),'XData',plotX);
        plotY = get(h(n),'YData');
        plotY(1)=p(n,2);
        %plotY(end+1)=p;
        set(h(n),'YData',plotY);
    end

end
function updateCoverage(h,step,p)
    for n =1:length(h)
        plotX = get(h(n),'XData');
        plotX(end+1)=step;
        set(h(n),'XData',plotX);
        plotY = get(h(n),'YData');
        %plotY(end+1)=norm(p(n,:));
        plotY(end+1)=p;
        set(h(n),'YData',plotY);
    end

end