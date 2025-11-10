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
