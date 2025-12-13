# Navigation Mesh System With Pathfinding

Custom Navigation Mesh and pathfinding system written in C++. The project builds a navmesh from raw geometry using ear clipping triangulation, optimizes polygons with the HertelMehlhorn algorithm, and performs A* pathfinding with Simple Stupid Funnel smoothing.
This is the first step of my custom **Navigation Mesh** system, which also includes a **Pathfinding algorithm**. For now, it doesn’t have **heightfields**, but my next goal is to implement it.
<img width="2548" height="1387" alt="bothpath" src="https://github.com/user-attachments/assets/8a39fe28-766e-4ef3-872b-8a5ad703af32" />
- For cloning repo `git clone https://github.com/haktan313/NavigationmeshSystem.git`
- Then run the Generation.bat file

🧩 Some Features
- ImGui panel for building the navmesh based on agent radius, debug options, and start/end position adjustment.
- Triangles are created by **ear clipping**. I used a custom raycast for ear clipping.
- **Hertel Mehlhorn** algorithm for optimizing triangles and creating complex poligons, which reduces the number of nodes needed for pathfinding.
- A* **pathfinding** combined with the **Simple Stupid Funnel Algorithm** for path smoothing.

## 📸 Screenshots
| NavMesh (Before Optimization) | NavMesh (After Hertel–Mehlhorn Optimization) |
|--------|-------------|
| <img width="2554" height="1384" alt="triangleswithsmoothpath" src="https://github.com/user-attachments/assets/945f73cf-c581-4436-bd6f-a0ec5a21bd29" /> | <img width="2549" height="1385" alt="justnavmesh" src="https://github.com/user-attachments/assets/3364a358-0c86-42a8-891f-77c5656612d0" /> |

| Normal Path | Smoothed Path (Funnel Algorithm) |
|--------|-------------|
| <img width="2549" height="1391" alt="Screenshot 2025-11-10 113245" src="https://github.com/user-attachments/assets/7408df89-b97d-414f-a9bf-2bf8a7f2919c" /> | <img width="2542" height="1377" alt="Screenshot 2025-11-10 113400" src="https://github.com/user-attachments/assets/c4b90997-5b73-450d-b58f-4651101f0ac1" /> |

Script Example (FunnelPathSmoothing Application)

```cpp
#pragma once
#include <vector>

#include "NavMesh.h"
#include "StructsForNavigationSystem.h"

class FunnelPathSmoothing
{
public:
    struct Portal
    {
        glm::vec3 left, right;
    };
    
    static std::vector<glm::vec3> SmoothPath(const std::vector<int>& pathNodeIDs, NavMesh& navMesh);
    static Portal MakePortal(const glm::vec3& currentCenter, const glm::vec3& nextCenter, const glm::vec3& edgeStart, const glm::vec3& edgeEnd);
};

```

```cpp
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

```
