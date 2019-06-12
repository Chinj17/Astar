clear all
clc

global cmap;
cmap = [1 1 1; ...  % 1 - White - map
    0 0 0; ...      % 2 - Black - obstacle
    1 0 0; ...      % 3 - Red - Destination
    1 0.5 0; ...    % 4 - Orange - Closed List
    0 1 0; ...      % 5 - Green - Start Node
    1 1 0; ...      % 6 - Yellow - Open List
    0 0 1];         % 7 - Blue - Final Path 

colormap(cmap);

AStar();
    
function [map,X,Y] = CreateMap()
global cmap
res = 1;
x = 0:res:250;
y = 0:res:250;
[X,Y] = meshgrid(x,y);
cols = size(y,2);
rows = size(x,2);
map = zeros(cols,rows);

%Original map
circle = (X-190).^2 +(Y-130).^2 <=225;
rectangle = (X<100)&(X>50)&(Y<112.5)&(Y>67.5);
ellipse = ((X-140).^2/225) + ((Y-120).^2/36) <=1;
polygon1 = (Y>15)&(37*X-20*Y<6101)&(38*X+23*Y<8530)&(38*X-7*Y>5830)&(X>163);
polygon2 =(2*X+19*Y<1314)&(41*X+25*Y>6525)&(X<164)&(Y>15);

map(circle | rectangle | ellipse | polygon1| polygon2) = 2;

end

function DrawMap(map)
global cmap
image(map)
set(gca, 'ydir', 'normal')
set(gca, 'xtick', 0:5:250)
set(gca, 'ytick', 0:5:150)
xlabel('X in mm')
ylabel('Y in mm')
grid on;
axis image;
axis([0 250 0 150])
end

function AStar()
global cmap
[map,X,Y] = CreateMap();
[rows,cols] = size(Y);
startx = input('Enter the start x coordinate: ');
starty = input('Enter the start y coordinate: ');
destx = input('Enter destination x coordinate: ');
desty = input('Enter destination y coordinate: ');


if (isa(startx,'double') == 1)
    startx = round(startx);
end

if (isa(starty,'double') == 1)
    starty = round(starty);
end

if (isa(desty,'double') == 1)
    desty = round(desty);
end

if (isa(destx,'double') == 1)
    destx = round(destx);
end

if (startx == 0)
    startx = startx + 1;
end

if (starty == 0)
    starty = starty + 1;
end

if (destx > 250)
    if (desty > 250)
        desty = input('Out of bounds. Enter y coordinate again: ');
    end
    startx = input('Enter the start x coordinate: ');
end
% Checking if destination or start coordinates inside obsracle space
start_lin = sub2ind(size(Y), starty, startx);
dest_lin  = sub2ind(size(Y), desty,  destx);

while(map(start_lin) == 2)
    startx = input('Enter the start x coordinate: ');
    starty = input('Enter the start y coordinate: ');
end
while(map(dest_lin) == 2)
    destx = input('Enter destination x coordinate: ');
    desty = input('Enter destination y coordinate: ');
end

% Generate linear indices of start and dest nodes

start_lin = sub2ind(size(Y), starty, startx);
dest_lin  = sub2ind(size(Y), desty,  destx);

% Giving color to the coordinates

map(start_lin) = 5; 
map(dest_lin)  = 3;

% Defining the parent matrix
parent_node = zeros(rows,cols);

% Calculating the Heuristic function
% Eucledian distance
xd = destx;
yd = desty;
dx = abs(X-xd); dy = abs(Y-yd);
H = sqrt((dx.^2) + (dy.^2));

% Initializing F and G arrays with infinite cost
f = Inf(rows,cols);
g = Inf(rows,cols);
g(start_lin) = 0;
f(start_lin) = H(start_lin);

while true
    map(start_lin) = 5;
    map(dest_lin) = 3;
    
    % Drawing the path continuously
%     if (true)
%         DrawMap(map)
%         drawnow;
%     end
    
   
    % Find the node with the minimum f value
    [minimumf, current_node] = min(f(:));
    % Condition to check for Destination node
    if ((current_node == dest_lin) || isinf(minimumf))
        break;
    end
    
    map(current_node) = 4;
    [i, j] = ind2sub(size(f), current_node);
    
    % Down
    if (i-1 > 1) 
        if (map(i-1,j)~=2 && map(i-1,j)~=5)
           if g(i-1,j) > (g(i,j) + 1)
               g(i-1,j) = g(i,j) + 1;
               f(i-1,j) = g(i-1,j) + H(i-1,j);
               map(i-1,j) = 6;
               parent_node(i-1,j) = current_node;
            end
        end
    end
    
    % Up
    if (i+1 < rows) 
        if (map(i+1,j)~=2 && map(i+1,j)~=5)
            if g(i+1,j) > (g(i,j) + 1)
                g(i+1,j) = g(i,j) + 1;
                f(i+1,j) = g(i+1,j) + H(i+1,j);
                map(i+1,j) = 6;
                parent_node(i+1,j) = current_node;
            end
        end
    end
    
    % Left
    if (j-1 > 1 ) 
        if (map(i,j-1)~=2 && map(i,j-1)~=5)
            if g(i,j-1) > (g(i,j) + 1)
                g(i,j-1) = g(i,j) + 1;
                f(i,j-1) = g(i,j-1) + H(i,j-1);
                map(i,j-1) = 6;
                parent_node(i,j-1) = current_node;
            end
        end
    end
    
    %Right
    if (j+1 < cols) 
        if (map(i,j+1)~=2 && map(i,j+1)~=5)
            if g(i,j+1) > (g(i,j) + 1)
               g(i,j+1) = g(i,j) + 1;
                f(i,j+1) = g(i,j+1) + H(i,j+1);
                map(i,j+1) = 6;
                parent_node(i,j+1) = current_node;
            end
        end
    end
    
    % Diagonal down right
    if (i-1 > 1 && j+1 < cols) 
        if (map(i-1,j+1)~=2 && map(i-1,j+1)~=5)
            if g(i-1,j+1) > (g(i,j) + sqrt(2))
                g(i-1,j+1) = g(i,j) + sqrt(2);
                f(i-1,j+1) = g(i-1,j+1) + H(i-1,j+1);
                map(i-1,j+1) = 6;
                parent_node(i-1,j+1) = current_node;
            end
        end
    end
    
    % Diagonal up right 
    if (i+1 < rows && j+1 < cols) 
        if (map(i+1,j+1)~=2 && map(i+1,j+1)~=5)
            if g(i+1,j+1) > (g(i,j) + sqrt(2))
                g(i+1,j+1) = g(i,j) + sqrt(2);
                f(i+1,j+1) = g(i+1,j+1) + H(i+1,j+1);
                map(i+1,j+1) = 6;
                parent_node(i+1,j+1) = current_node;
            end
        end
    end
    
    % Diagonal up left
    if (i+1 < rows && j-1 > 1)
        if (map(i+1,j-1)~=2 && map(i+1,j-1)~=5)
            if g(i+1,j-1) > (g(i,j) + sqrt(2))
                g(i+1,j-1) = g(i,j) + sqrt(2);
                f(i+1,j-1) = g(i+1,j-1) + H(i+1,j-1);
                map(i+1,j-1) = 6;
                parent_node(i+1,j-1) = current_node;
            end
        end
    end
    
    % Diagonal down left
    if (i-1 > 1 && j-1 > 1) 
        if (map(i-1,j-1)~=2 && map(i-1,j-1)~=5)
            if g(i-1,j-1) > (g(i,j) + sqrt(2))
                g(i-1,j-1) = g(i,j) + sqrt(2);
                f(i-1,j-1) = g(i-1,j-1) + H(i-1,j-1);
                map(i-1,j-1) = 6;
                parent_node(i-1,j-1) = current_node;
            end
        end
    end
    
    f(current_node) = Inf;
    
end
% Constructing the route
% If F value of destination is infinity then no path found

if (isinf(f(dest_lin)))
    route = [];
else
    route = [dest_lin];
    
    while (parent_node(route(1)) ~= 0)
        route = [parent_node(route(1)), route];
    end

    % Drawing the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7; % giving it a blue colour
%         DrawMap(map)
%         drawnow;
    end
end
DrawMap(map)
end