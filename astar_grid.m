function path = astar_grid(grid, startNode, goalNode)
%ASTAR_GRID Generic 2-D A* path planner.
%
% grid      : 0 = free cell, 1 = obstacle
% startNode : [row col]
% goalNode  : [row col]
% path      : N-by-2 array of [row col] coordinates
%
% Example:
%   G = zeros(30,30); G(10:20,15) = 1;
%   p = astar_grid(G,[2 2],[28 28]);

[nRows,nCols] = size(grid);

if grid(startNode(1),startNode(2)) ~= 0 || grid(goalNode(1),goalNode(2)) ~= 0
    error('Start and goal must lie in free cells.');
end

gScore = inf(nRows,nCols);
fScore = inf(nRows,nCols);
closed = false(nRows,nCols);
cameFrom = zeros(nRows,nCols,2);

gScore(startNode(1),startNode(2)) = 0;
fScore(startNode(1),startNode(2)) = heuristic(startNode,goalNode);

openList = startNode;

neighbors = [ ...
    -1  0;
     1  0;
     0 -1;
     0  1;
    -1 -1;
    -1  1;
     1 -1;
     1  1];

while ~isempty(openList)
    vals = arrayfun(@(k) fScore(openList(k,1),openList(k,2)), ...
                    1:size(openList,1));
    [~,idx] = min(vals);
    current = openList(idx,:);
    openList(idx,:) = [];

    if isequal(current,goalNode)
        path = reconstructPath(cameFrom,current,startNode);
        return
    end

    closed(current(1),current(2)) = true;

    for i = 1:size(neighbors,1)
        nb = current + neighbors(i,:);

        if nb(1) < 1 || nb(1) > nRows || nb(2) < 1 || nb(2) > nCols
            continue
        end
        if grid(nb(1),nb(2)) ~= 0 || closed(nb(1),nb(2))
            continue
        end

        stepCost = norm(neighbors(i,:));
        tentativeG = gScore(current(1),current(2)) + stepCost;

        if tentativeG < gScore(nb(1),nb(2))
            cameFrom(nb(1),nb(2),:) = current;
            gScore(nb(1),nb(2)) = tentativeG;
            fScore(nb(1),nb(2)) = tentativeG + heuristic(nb,goalNode);

            if ~ismember(nb,openList,'rows')
                openList(end+1,:) = nb; %#ok<AGROW>
            end
        end
    end
end

path = [];
warning('No feasible path found.');
end

function h = heuristic(a,b)
% Euclidean heuristic.
h = norm(a-b);
end

function path = reconstructPath(cameFrom,current,startNode)
path = current;
while ~isequal(current,startNode)
    parent = squeeze(cameFrom(current(1),current(2),:))';
    if all(parent == 0)
        path = [];
        return
    end
    current = parent;
    path = [current; path]; %#ok<AGROW>
end
end
