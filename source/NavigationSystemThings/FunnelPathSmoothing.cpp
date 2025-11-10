#include "FunnelPathSmoothing.h"


std::vector<glm::vec3> FunnelPathSmoothing::SmoothPath(const std::vector<int>& pathNodeIDs, NavMesh& navMesh)
{
    if (pathNodeIDs.size() < 2)
    {
        static std::vector<glm::vec3> emptyPath;
        return emptyPath;
    }
    
    std::vector<Portal> portals;
    portals.reserve(pathNodeIDs.size() + 1);
    
    glm::vec3 start = navMesh.Start;
    glm::vec3 goal  = navMesh.End;
    portals.push_back({ start, start });

    for (int i = 0; i  < static_cast<int>(pathNodeIDs.size()) - 1; ++i)
    {
        int currentNodeID = pathNodeIDs[i];
        int nextNodeID = pathNodeIDs[i + 1];
        
        const auto& currentNode = NavigationUtility::GetPathfindingNodes(navMesh)[currentNodeID];
        const auto& nextNode = NavigationUtility::GetPathfindingNodes(navMesh)[nextNodeID];

        bool bPointsFound = false;
        glm::vec3 edgeStart, edgeEnd;
        
        for (const auto& neighbor : currentNode.neighbors)
            if (neighbor.neighborFaceIndex == nextNodeID)
            {
                edgeStart = neighbor.edgeStart;
                edgeEnd = neighbor.edgeEnd;
                bPointsFound = true;
                break;
            }
        if (!bPointsFound)
            for (const auto& e : nextNode.neighbors)
                if (e.neighborFaceIndex == currentNodeID)
                {
                    edgeStart = e.edgeEnd;
                    edgeEnd = e.edgeStart;
                    bPointsFound = true;
                    break;
                }
        portals.push_back(MakePortal(currentNode.centerPoint, nextNode.centerPoint, edgeStart, edgeEnd));
    }
    
    portals.push_back({ goal, goal });
    
    glm::vec3 currentPoint = portals[0].left;
    glm::vec3 left = portals[0].left;
    glm::vec3 right = portals[0].right;
    int currentPointIndex = 0, leftIndex = 0, rightIndex = 0;
    
    std::vector<glm::vec3> smoothedPath;
    smoothedPath.push_back(currentPoint);

    for (int i = 1; i < static_cast<int>(portals.size()); ++i)
    {
        glm::vec3& newLeft  = portals[i].left;
        glm::vec3& newRight = portals[i].right;
        
        if (NavigationUtility::CrossProductXZ(newLeft - currentPoint, left - currentPoint) >= 0.0f)
        {
            if (NavigationUtility::CrossProductXZ(newLeft - currentPoint, right - currentPoint) > 0.0f)
            {
                currentPoint = right;
                currentPointIndex = rightIndex;
                smoothedPath.push_back(currentPoint);
                
                left = currentPoint;
                right = currentPoint;
                leftIndex = currentPointIndex;
                rightIndex = currentPointIndex;
                
                i = currentPointIndex;
                continue;
            }
            left = newLeft;
            leftIndex = i;
        }
        
        if (NavigationUtility::CrossProductXZ(newRight - currentPoint, right - currentPoint) <= 0.0f)
        {
            if (NavigationUtility::CrossProductXZ(newRight - currentPoint, left - currentPoint) < 0.0f)
            {
                currentPoint = left;
                currentPointIndex = leftIndex;
                smoothedPath.push_back(currentPoint);

                left = currentPoint;
                right = currentPoint;
                leftIndex = currentPointIndex;
                rightIndex = currentPointIndex;

                i = currentPointIndex;
                continue;
            }
            right = newRight;
            rightIndex = i;
        }
    }
    
    if (glm::distance(smoothedPath.back(), goal) > 0.000001f)
        smoothedPath.push_back(goal);

    return smoothedPath;
}

FunnelPathSmoothing::Portal FunnelPathSmoothing::MakePortal(const glm::vec3& currentCenter, const glm::vec3& nextCenter, const glm::vec3& edgeStart, const glm::vec3& edgeEnd)
{
    glm::vec3 a = edgeStart;
    glm::vec3 b = edgeEnd;
        
    glm::vec3 direction = glm::normalize(glm::vec3(nextCenter.x - currentCenter.x, 0.0f, nextCenter.z - currentCenter.z));
    if (NavigationUtility::CrossProductXZ(b - a, direction) > 0.0f) 
        return { a, b };
    
    return { b, a };
}
