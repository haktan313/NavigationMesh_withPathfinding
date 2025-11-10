#pragma once
#include "NavMeshDebugger.h"
#include "Core/Scene.h"
#include "Core/Shader.h"
#include "Core/Camera.h"
#include <map>
#include "StructsForNavigationSystem.h"
#include "NavigationUtility.h"

class NavMesh
{
public:
    NavBuildParams BuildParams;
    DrawDebugInfo DebugInfo;

    glm::vec3 Start, End;

    NavMesh(const Scene& scene);
    ~NavMesh();
    
    void BuildNavMesh();
    void RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene);
    void SetStartEndMarkers(const glm::vec3& start, const glm::vec3& end);
    
private:
    
    void BuildBorderFromParams();
    void CreateDebugger();
    
    void EarClipping();
    void CreateHalfEdgeStructure();
    void FindTwinHalfEdges();

    void OptimizeEarClipping();
    void MergeTriangles();
    
    void CreatePathfindingNodes(std::map<int, int>& faceIndexToNodeIndex, std::vector<std::vector<glm::vec3>>& mergedPolygonsForDebug);
    void FindNeighborsForPathfindingNodes(std::map<int, int>& faceIndexToNodeIndex);
    
    const Scene& m_Scene;
    NavMeshDebugger* m_NavMeshDebugger;
    std::vector<glm::vec3> m_BorderVerts3D;
    std::vector<glm::vec3> m_NavMeshVerts3D;
    std::vector<NavMeshTriangle> m_NavMeshTriangles;

    std::vector<HalfEdgeVertex> m_HalfEdgeVertices;
    std::vector<HalfEdgeFace> m_HalfEdgeFaces;
    std::vector<HalfEdge> m_HalfEdges;
    std::vector<NavMeshOptimizedNode> m_PathfindingNodes;

    friend class NavigationUtility;
    
};
