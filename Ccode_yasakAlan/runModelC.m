function runModel(ResZones,RefX,RefY,initPosX,initPosY,mode,WZone,WZoneElvEn,WZoneTraEn,VelDisplay)
if nargin==9
    VelDisplay=0;
end
assignin('base','ResZones',ResZones);
assignin('base','RefX',RefX);
assignin('base','RefY',RefY);
assignin('base','initPosX',initPosX);
assignin('base','initPosY',initPosY);
assignin('base','mode',mode);
assignin('base','WZone',WZone);
assignin('base','WZoneElvEn',WZoneElvEn);
assignin('base','WZoneTraEn',WZoneTraEn);

simout=sim('yasakAlanTestModelCcode.slx','CaptureErrors','off',...
        'ZeroCross','on', ...
        'SaveTime','off','TimeSaveName','tout', ...
        'SaveState','off','StateSaveName','xoutNew',...
        'SaveOutput','off','OutputSaveName','youtNew',...
        'SignalLogging','off','SignalLoggingName','logsout');

curposx=simout.curposx;
curposy=simout.curposy;
curvelx=simout.curvelx;
refvelx=simout.refvelx;
curvely=simout.curvely;
refvely=simout.refvely;
% curRZone=simout.CurRZone;
curRZone=simout.CurRZone.Data(:,:,end);

c=linspace(1,10,length(curposx.Data));
figure
lgnd=[];
scatter(curposx.Data(1:end),curposy.Data(1:end),[],c,'filled');
lgnd=[lgnd,"Current Position Of The System"];
hold on
plot(curposx.Data(1:1),curposy.Data(1:1),'k','marker','+' , 'LineWidth' , 10);
lgnd=[lgnd,"Init Position"];
hold on
if mode==1
    plot(RefX,RefY,'r','marker','+' , 'LineWidth' , 10);
    lgnd=[lgnd,"Target Position"];
end
grid on;
xlim([-180 180]);
ylim([-40 70]);
NumOfRestArea = 6;
NumOfRestArea = length(curRZone);

for i=1:NumOfRestArea
    en = curRZone(i,1);
    xmin = curRZone(i,2);
    xmax = curRZone(i,4);
    ymin = curRZone(i,3);
    ymax = curRZone(i,5);
    if(en==1)
          fill([xmin,xmax,xmax,xmin],[ymin,ymin,ymax,ymax],'r','EdgeColor','r','EdgeAlpha',0.5,'FaceAlpha',0.3);
    end
end

if(WZoneElvEn==1 && WZoneTraEn==1)
    if WZone(1)<=WZone(3)
        if WZone(2)<=WZone(4)
            fill([WZone(1),WZone(3),WZone(3),WZone(1)],[WZone(2),WZone(2),WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
        else
            fill([WZone(1),WZone(3),WZone(3),WZone(1)],[-540,-540,WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
            fill([WZone(1),WZone(3),WZone(3),WZone(1)],[WZone(2),WZone(2),540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
        end
    else
        if WZone(2)<=WZone(4)
            fill([-540,WZone(3),WZone(3),-540],[WZone(2),WZone(2),WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
            fill([WZone(1),540,540,WZone(1)],[WZone(2),WZone(2),WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
        else
            fill([-540,WZone(3),WZone(3),-540],[-540,-540,WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
            fill([-540,WZone(3),WZone(3),-540],[WZone(2),WZone(2),540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
            fill([WZone(1),540,540,WZone(1)],[-540,-540,WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
            fill([WZone(1),540,540,WZone(1)],[WZone(2),WZone(2),540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
        end
    end    
elseif(WZoneElvEn==0 && WZoneTraEn==1)
    if WZone(1)<=WZone(3)
        fill([WZone(1),WZone(3),WZone(3),WZone(1)],[-540,-540,540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
    else
        fill([-540,WZone(3),WZone(3),-540],[-540,-540,540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
        fill([WZone(1),540,540,WZone(1)],[-540,-540,540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
    end
elseif(WZoneElvEn==1 && WZoneTraEn==0)
    if WZone(2)<=WZone(4)
        fill([-540,540,540,-540],[WZone(2),WZone(2),WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
    else
        fill([-540,540,540,-540],[-540,-540,WZone(4),WZone(4)],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
        fill([-540,540,540,-540],[WZone(2),WZone(2),540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
    end
elseif(WZoneElvEn==0 && WZoneTraEn==0)
    fill([-540,540,540,-540],[-540,-540,540,540],'g','EdgeColor','g','EdgeAlpha',0.3,'FaceAlpha',0.2);
end
% xmin = curRZone.Data(2);
% xmax = curRZone.Data(4);
% ymin = curRZone.Data(3);
% ymax = curRZone.Data(5);
% fill([xmin,xmax,xmax,xmin],[ymin,ymin,ymax,ymax],'r','EdgeColor','b','EdgeAlpha',0.5,'FaceAlpha',0.3);

hold on
title('Displacement Of The System Under Specified Reference Position')
legend(lgnd)
if VelDisplay
    figure
    lgnd=[];
    plot(refvelx.Time(1:end),curvelx.Data(1:end),'g' , 'LineWidth' , 3);
    lgnd=[lgnd,"Traverse Speed Fdb"];
    hold on;
    if mode==2
        plot(refvelx.Time(1:end),refvelx.Data(1:end),'r' , 'LineWidth' , 3);
        lgnd=[lgnd,"Traverse Speed Ref"];
        hold on;
    end
    plot(refvelx.Time(1:end),curvely.Data(1:end),'b' , 'LineWidth' , 3);
    lgnd=[lgnd,"Elevation Speed Fdb"];
    hold on;
    if mode==2
        plot(refvelx.Time(1:end),refvely.Data(1:end),'c' , 'LineWidth' , 3);
        lgnd=[lgnd,"Elevation Speed Ref"];
    end
    grid on;
    title('Velocity Reference & Feedback')
    legend(lgnd)
end
fprintf('---------------------------------------------------------------------------------------------');    
end
% 
% 
