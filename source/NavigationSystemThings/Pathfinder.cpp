#include "Pathfinder.h"

#include <iostream>
#include <queue>
#include <set>

#include "FunnelPathSmoothing.h"

PathfindingResult& Pathfinder::FindPath(NavMesh& navMesh, const glm::vec3& start, const glm::vec3& end)
{
    int startNodeID = NavigationUtility::FindNodeIDByPosition(start, navMesh);
    int endNodeID = NavigationUtility::FindNodeIDByPosition(end, navMesh);
    if (startNodeID == -1 || endNodeID == -1)
    {
        std::cout << "Pathfinder: Start or End position is outside the NavMesh." << std::endl;
        static PathfindingResult noPathResult;
        noPathResult.bPathFound = false;
        return noPathResult;
    }
    if (startNodeID == endNodeID)
    {
        std::cout << "Pathfinder: Start and End positions are in the same node." << std::endl;
        static PathfindingResult trivialResult;
        trivialResult.pathNodeIDs = { startNodeID };
        trivialResult.bPathFound = true;
        return trivialResult;
    }

    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> openSet;
    std::set<int> closedSet;

    std::map<int, int> cameFrom;
    std::map<int, float> gCost;
    std::map<int, float> fCost;

    gCost[startNodeID] = 0.0f;
    fCost[startNodeID] = CalculateCost(navMesh, startNodeID, endNodeID);
    openSet.push({fCost[startNodeID], startNodeID});
    
    while (!openSet.empty())
    {
        int currentNodeID = openSet.top().second;
        openSet.pop();

        if (currentNodeID == endNodeID)
        {
            std::vector<int> pathIDs = ReconstructPath(cameFrom, currentNodeID);
            PathfindingResult& result = *new PathfindingResult();
            result.bPathFound = true;
            result.pathNodeIDs = pathIDs;
            for (int nodeID : pathIDs)
                if (nodeID == startNodeID)
                    result.pathPoints.push_back(start);
                else if (nodeID == endNodeID)
                    result.pathPoints.push_back(end);
                else
                    result.pathPoints.push_back(NavigationUtility::GetNodeCenter(nodeID, navMesh));
            std::cout << "Pathfinder: Path found from node " << startNodeID << " to node " << endNodeID << "." << std::endl;
            return result;
        }

        if (closedSet.count(currentNodeID))
            continue;
        closedSet.insert(currentNodeID);

        for (const auto& neighborEdge : NavigationUtility::GetNodeNeighbors(currentNodeID, navMesh))
        {
            int neighborID = neighborEdge.neighborFaceIndex;
            if (closedSet.count(neighborID))
                continue;
            
            float gCostDummy = gCost[currentNodeID] + CalculateCost(navMesh, currentNodeID, neighborID);
            if (gCost.count(neighborID) == 0 || gCostDummy < gCost[neighborID])
            {
                cameFrom[neighborID] = currentNodeID;
                gCost[neighborID] = gCostDummy;
                fCost[neighborID] = gCostDummy + CalculateCost(navMesh, neighborID, endNodeID);
                openSet.push({fCost[neighborID], neighborID});
            }
        }
    }

    std::cout << "Pathfinder: No path found from start to end." << std::endl;
    static PathfindingResult noPathResult;
    noPathResult.bPathFound = false;
    return noPathResult;
}


// Helperss


float Pathfinder::CalculateCost(NavMesh& navMesh, int nodeA, int nodeB)
{
    return glm::distance(NavigationUtility::GetNodeCenter(nodeA, navMesh), NavigationUtility::GetNodeCenter(nodeB,navMesh));
}

std::vector<int> Pathfinder::ReconstructPath(std::map<int, int>& cameFrom, int currentID)
{
    std::vector<int> totalPath;
    totalPath.push_back(currentID);
    while (cameFrom.count(currentID))
    {
        currentID = cameFrom[currentID];
        totalPath.push_back(currentID);
    }
    std::reverse(totalPath.begin(), totalPath.end());
    std::cout << "Pathfinder: Path reconstructed with " << totalPath.size() << " nodes." << std::endl;
    std::cout << "Path: ";
    for (int nodeID : totalPath)
        std::cout << nodeID << " ";
    std::cout << std::endl;
    return totalPath;
}

