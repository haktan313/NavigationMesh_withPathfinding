#pragma once
#include <vector>
#include "NavMeshDebugger.h"
#include "StructsForNavigationSystem.h"

class NavMesh;

class NavigationUtility
{
public:
    static bool IsItEqual(const glm::vec3& a, const glm::vec3& b, float epsilon = 1e-5f);
    static float CrossProductXZ(const glm::vec3& v1, const glm::vec3& v2);

    static int FindNodeIDByPosition(const glm::vec3& position, NavMesh& navMesh);
    static glm::vec3 GetNodeCenter(int nodeID, NavMesh& navMesh);
    static const std::vector<OptimizedEdge>& GetNodeNeighbors(int nodeID, NavMesh& navMesh);
    
    static bool IsPointInTriangleXZ(const glm::vec3& position, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
    static bool CanClipEar(int prevIndex, int earIndex, int nextIndex, const std::vector<glm::vec3>& vertices);
    static int FindOrCreateHalfEdgeVertexIndex(const glm::vec3& pos, std::vector<HalfEdgeVertex>& m_HalfEdgeVertices);
    static void FindRemovableEdgeIndexes(std::vector<int>& outRemovableEdgeIndexes, std::vector<HalfEdge>& m_HalfEdges, std::vector<HalfEdgeVertex>& m_HalfEdgeVertices, NavMeshDebugger* m_NavMeshDebugger);
    
    struct DummyFaceBoundaryData
    {
        std::vector<int> edgeLoop;
        std::vector<int> vertexLoop;
    };
    static DummyFaceBoundaryData CollectFaceBoundaryVertices(int faceIndex, std::vector<HalfEdge>& m_HalfEdges, std::vector<HalfEdgeFace>& m_HalfEdgeFaces);
    static std::vector<int> BuildMergedPolygonVerts(const DummyFaceBoundaryData& faceA, const DummyFaceBoundaryData& faceB, int halfEdgeAB, int halfEdgeBA, std::vector<HalfEdge>& m_HalfEdges);
    static void RebuildFaceFromPolygon(int faceKeepIndex, const std::vector<int>& mergedVertexLoop, std::vector<HalfEdge>& HalfEdges, std::vector<HalfEdgeFace
                                       >& HalfEdgeFaces);
    static void MergeTwoFacesProperley(int removeHalfEdgeABIndex, std::vector<HalfEdge>& HalfEdges, std::vector<HalfEdgeFace>& HalfEdgeFaces, std::vector<HalfEdgeVertex>& HalfEdgeVertices);

    static bool RaycastXZ(const std::vector<glm::vec3>& navMeshVertices, const glm::vec3& rayP1, const glm::vec3& rayP2, NavMeshHitInfo& hitInfo);
    static std::vector<std::vector<glm::vec3>> GetSceneObstacleSlices(float buildPlaneY, NavBuildParams& buildParams, const Scene& scene);
    static void SortObstaclesByMaxX(std::vector<std::vector<glm::vec3>>& obstacleSlices);
    static void CreateHolesWithObstacles(const std::vector<glm::vec3>& obstacleSlice, std::vector<glm::vec3>& navMeshVerts3D, const NavBuildParams& buildParams);

    static std::vector<NavMeshOptimizedNode>& GetPathfindingNodes(NavMesh& navMesh);
};
