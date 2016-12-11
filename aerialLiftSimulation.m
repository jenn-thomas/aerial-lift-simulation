% README
% For each tcp command sent, the user must hit start tcp. It will then wait
% for a command and close the port. To reopen, hit the start tcp button
% again. This way you can switch between modes of submitting x,y,z or theta
% values. The GUI has information explaining the modes.

function aerialLiftSimulation
close all
ipaddress = char(getHostAddress(java.net.InetAddress.getLocalHost));
port = 6232;
ipInfo = sprintf('IP: %s port: %d',ipaddress,port);
% lengths of links in ft
l1 = 8;
l2 = 20;
l3 = 1;
l4 = 5;
% starting theta values
x = 0; y = 25; z = 0;
pointString = sprintf('Point: %d, %d, %d', x,y,z);
xPoints = []; yPoints = []; zPoints = [];
theta1 = 0; theta2 = 0; theta3 = 0;
thetaString = sprintf('Values: %d, %d, %d', theta1,theta2,l3);
% get values and points and graph them
% draw GUI
% GUI variables
scrsz = get(groot,'ScreenSize');
plotWidth = scrsz(3)/1.2/3.5;
plotHeight = scrsz(4)/3;
% figure and graphics
h = figure('Position',[scrsz(4)/8 scrsz(4)/2 scrsz(3)/1.2 scrsz(4)/1.1],...
    'Resize','off','Name','Exam Problem 2','Toolbar','none',...
    'NumberTitle','off','MenuBar','none','Visible','off');
hp = uipanel('Title','Aerial Lift Simulation','FontSize',30,...
    'BackgroundColor','white',...
    'Position',[0.005 0.005 .99 .99]);
% CONTROLS
hsp = uipanel('Parent',hp,'Title','Control Panel','FontSize',14,...
    'Position',[.025 .025 .5 .47]);
% radio buttons
radioBtns = uibuttongroup('Parent',hsp,'Title','Send:',...
    'Position',[.008 .79 .985 .2],...
    'SelectionChangedFcn',@radioSelection);
% Create two radio buttons in the button group.
r1 = uicontrol(radioBtns,'Style','radiobutton',...
    'String','Point X,Y,Z',...
    'Position',[200 0 100 55],...
    'HandleVisibility','off');
r2 = uicontrol(radioBtns,'Style',...
    'radiobutton',...
    'String','Thetas and L',...
    'Position',[100 0 100 55],...
    'HandleVisibility','off');

% Text showing x,y,z position
pointLocation = uipanel('Parent',hsp,...
     'Position',[.6 .77 .25 .22]);
pointText = uicontrol('Parent',pointLocation,'Style','text',...
        'Position',[0 0 125 43],'fontsize',15,...
        'String', pointString);

% theta group
thetaValues = uipanel('Parent',hsp,'Title','Theta and L Values',...
     'Position',[.008 .05 .985 .7],'Visible','off','FontSize',14);
txt1 = uicontrol('Parent',thetaValues,'Style','text',...
        'Position',[60 135 390 60],'FontSize',14,...
        'String','Send through TCP a string containing theta1,theta2,L3. View the diagram for the first part of problem1. The values recieved will appear here and plotted on the graphs. It will then send the thetas back.');
tcpThetas = uicontrol('Parent',thetaValues,'Style','text',...
        'Position',[90 95 300 40],'FontSize',14,...
        'String',thetaString);
thetaWarning = uicontrol('Parent',thetaValues,'Style','text','ForegroundColor','red',...
        'FontSize',16,'Position',[60 30 390 60],...
        'String','Warning: This position is impossible since the third link is negative, though it has been plotted anyways.','Visible', 'off');
% enter point group
points = uipanel('Parent',hsp,'Title','Enter a Point',...
    'Position',[.008 .05 .985 .7],'Visible','on','FontSize',14);
btxt1 = uicontrol('Parent',points,'Style','text',...
        'FontSize',14,...
        'Position',[100 135 310 60],...
        'String','Send through TCP a string containing X,Y,Z. The values recieved will appear here and plotted on the graphs. It will then send the point back.');
tcpPoint = uicontrol('Parent',points,'Style','text',...
        'FontSize',14,...
        'Position',[175 105 145 20],...
        'String',pointString);

pointsWarning = uicontrol('Parent',points,'Style','text','ForegroundColor','red',...
        'FontSize',16,'Position',[60 30 390 60],...
        'String','Warning: This position is impossible since the third link is negative, though it has been plotted anyways.','Visible', 'off');
calcValues();

%tcp button
ip = uicontrol('Parent',hsp,'Style','text',...
        'FontSize',14,...
        'Position',[20 10 200 20],...
        'String',ipInfo);
pushBtn = uicontrol('Parent',hsp,'Style', 'pushbutton', 'String', 'Start TCP',...
        'Position', [400 10 100 20],...
        'Callback', @buttonPushed);  

% various plots
xygraph = axes('Parent',hp,'Units','pixels','Title','XY Plane',...
    'Position', [39 scrsz(4)/2 plotWidth plotHeight]);
xy = plot(xPoints, yPoints, 'b-o'); title('XY Plane'); xlabel('x'); ylabel('y');
axis([-40 40 0 60]);
xzgraph = axes('Parent',hp,'Units','pixels','Title','XZ Plane',...
    'Position', [388 scrsz(4)/2 plotWidth plotHeight]);
xz = plot(xPoints, zPoints, 'b-o'); title('ZX Plane'); xlabel('x'); ylabel('z');
axis([-40 40 0 40]);
yzgraph = axes('Parent',hp,'Units','pixels','Title','YZ Plane',...
    'Position', [738 scrsz(4)/2 plotWidth plotHeight]);
yz = plot(yPoints, zPoints, 'b-o'); title('ZY Plane'); xlabel('y'); ylabel('z');
axis([0 60 0 40]);
threed = axes('Parent',hp,'Units','pixels','Title','3d Plot',...
    'Position', [650 50 plotWidth scrsz(4)/2.7]);
plot3d = plot3(xPoints,yPoints,zPoints,'b-o'); xlabel('x'); ylabel('y'); zlabel('z')
        axis([-40 40 0 60 0 40]); grid on; hold on;
        fill3([-40 40 40 -40],[50 50 60 60],[20 20 20 20],[.9 .9 .9])
        fill3([-40 40 40 -40],[50 50 60 60],[10 10 10 10],[.9 .9 .9])
        fill3([-40 40 40 -40],[50 50 60 60],[30 30 30 30],[.9 .9 .9])
        fill3([-40 -40 0 0 40 40],[50 50 50 50 50 50],[0 40 40 20 20 0],[135 206 250]/255)
        fill3([-40 -40 -40 -40],[50 60 60 50],[0 0 40 40],[135 206 250]/255)
        fill3([40 40 40 40],[50 60 60 50],[0 0 40 40],[135 206 250]/255)
        fill3([-40 -40 -40 -40],[25 45 45 25],[4 4 14 14],[.75 .75 .75])
        fill3([-32 -32 -32 -32],[25 45 45 25],[4 4 14 14],[.75 .75 .75])
        fill3([-40 -40 -32 -32],[25 45 45 25],[14 14 14 14],[.75 .75 .75])
        fill3([-40 -40 -32 -32],[25 45 45 25],[4 4 4 4],[.75 .75 .75])
        plot3([-40 -40 -40 -40],[25 25 45 45],[0 4 4 0],'black',[-32 -32],[25 25],[4 0],'black')
        hold off;
h.Visible = 'on';
refreshdata();

% GUI manipulation functions
% radio button switch event
    function radioSelection(hObject, eventdata, handles)
        switch get(eventdata.NewValue,'String')
            case 'Thetas and L'
                thetaValues.Visible = 'on';
                points.Visible = 'off';
            case 'Point X,Y,Z'
                thetaValues.Visible = 'off';
                points.Visible = 'on';
        end
    end
    
% function to calculate theta values and then points at end of links
    function calcValues()
        if strcmp(points.Visible,'on')
            theta1 = atan2(y,x);
            theta1deg = theta1*180/pi;
            r = sqrt(x^2+y^2);
            theta2 = atan2(r-l4,(z-l1));
            l3 = sqrt((r-l4)^2+(z-l1)^2)-l2;
            theta2deg = theta2*180/pi;
            theta3 = pi/2-theta2;
            theta3deg = theta3*180/pi;
        end
        % points of arm:
        % origin point
        x0 = 0; y0 = 0; z0 = 0;
        % origin to link 1
        x1 = x0; y1 = y0; z1 = l1;
        % link1 to link2
        x2 = l2*cos(pi/2-theta2)*cos(theta1); y2 = l2*cos(pi/2-theta2)*sin(theta1); z2 = z - l3*cos(theta2);
        % link2 to link3
        x3 = (l2+l3)*cos(pi/2-theta2)*cos(theta1); y3 = (l2+l3)*cos(pi/2-theta2)*sin(theta1); z3 = z;
        % link3 to link4
        x4 = x; y4 = y; z4 = z;
        xPoints = [x0 x1 x2 x3 x4];
        yPoints = [y0 y1 y2 y3 y4];
        zPoints = [z0 z1 z2 z3 z4];
        % update graph data
        xy.XData = xPoints; xz.XData = xPoints; plot3d.XData = xPoints;
        xy.YData = yPoints; yz.XData = yPoints; plot3d.YData = yPoints;
        xz.YData = zPoints; yz.YData = zPoints; plot3d.ZData = zPoints;
        pointString = sprintf('Point: %g, %g, %g', round(x,1),round(y,1),round(z,1));
        pointText.String = pointString;
        tcpPoint.String = pointString;
        thetaString = sprintf('Values: %d, %d, %d', round(theta1,3),round(theta2,3),round(l3,3));
        tcpThetas.String = thetaString;
        if (l3 < 1)
%                 thetaWarning.Visible = 'on';
%                 pointsWarning.Visible = 'on';
        else
            thetaWarning.Visible = 'off';
            pointsWarning.Visible = 'off';
        end
    end

    function buttonPushed(hObject, eventdata, handles)
        display('start TCP')
        pointCurrent = [x y z];
        t = tcpip('0.0.0.0', port, 'NetworkRole', 'Server');
        fopen(t);
        data = fscanf(t,'%s');
        num = strsplit(data,',');
        if strcmp(points.Visible,'on')
            x = str2num(num{1});
            y = str2num(num{2});
            z = str2num(num{3});
            pointNew = [x y z];
            fwrite(t,data,'char');
            fclose(t);
            animateArm(pointCurrent, pointNew);
        else strcmp(thetaValues.Visible,'on')
            theta1 = str2num(num{1})*pi/180;
            theta2 = str2num(num{2})*pi/180;
            l3 = str2num(num{3});
            calcValues();
            fwrite(t,data,'char');
            fclose(t);
        end
        display('TCP ended');
    end

% move the arm from one point to another
    function animateArm(pointCurrent, pointNew)
        increment = (pointCurrent - pointNew);
        direction = sign(increment);
        addArray = direction*-.25;
        pointCurrent = round(pointCurrent);
        while (~isequal(round(pointCurrent),pointNew))
            if round(pointCurrent(1)) ~= pointNew(1)
                pointCurrent(1) = pointCurrent(1) + addArray(1);
            end
            if round(pointCurrent(2),1) ~= pointNew(2)
                pointCurrent(2) = pointCurrent(2) + addArray(2);
            end
            if round(pointCurrent(3),1) ~= pointNew(3)
                pointCurrent(3) = pointCurrent(3) + addArray(3);
            end
            x = pointCurrent(1);
            y = pointCurrent(2);
            z = pointCurrent(3);
            calcValues()
            refreshdata();
            pause(.0001)
        end
        x = pointNew(1);
        y = pointNew(2);
        z = pointNew(3);
        calcValues();
        refreshdata();
    end

end
