function ind = calcLesserNeighbor(n,drone)
% This function calculates the Lesser Neighborhood of a drone
% Please refer to the Lesser Neighbor Algorithm
ind=[];
if (n==1)
    return;
end

currentGilbert = drone(n).gilbert{end};
visited = zeros(1, n);
path = [];
allPaths = {};
allPaths = findAllPathsRecursive(n, drone, visited, path, allPaths, currentGilbert);



toberemovedLogic=[];
for i=1:length(allPaths)
    for j = 1:length(allPaths)
        if(i==j)
            continue
        end
        if(isSubArray(allPaths{i},allPaths{j}))
            toberemovedLogic(end+1)=i;
        end
    end
end


allPaths(toberemovedLogic)=[];
maxNeighbors = zeros(1,length(allPaths));
for i =1:length(allPaths)
    allPaths{i}(1)=[];
    maxNeighbors(i) = max(allPaths{i});
end
ind = unique(maxNeighbors);


function allPaths = findAllPathsRecursive(currentNode, drone, visited, path, allPaths, mainGilbert)
    visited(currentNode) = 1;
    path = [path, currentNode];
    
    neighbors = drone(currentNode).gilbert{end};
    % If the current node is the destination node, add the path to allPaths
    if isempty(neighbors)
        allPaths{end+1} = path;
    else
        % Otherwise, explore neighbors
        logicalIndices = ismember(neighbors, mainGilbert);
        neighbors = neighbors(logicalIndices);
        if isempty(neighbors)
            allPaths{end+1} = path;
        else
        for i = 1:length(neighbors)
            if ~visited(neighbors(i))
                allPaths = findAllPathsRecursive(neighbors(i), drone, visited, path, allPaths, mainGilbert);
            end
        end
        end
    end
    
    % Backtrack: mark current node as unvisited and remove from path
    visited(currentNode) = 0;
    path = path(1:end-1);
end

function result = isSubArray(A,B)
lenA = length(A)-1;
lenB = length(B)-1;

result = false;
if(lenA > lenB)
    return;
end

if(B(lenB-lenA+2:end)==A(2:end))
    result = true;
end

end


end