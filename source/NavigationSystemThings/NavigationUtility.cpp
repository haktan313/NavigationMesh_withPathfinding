#include "NavigationUtility.h"
#include <algorithm>
#include <iostream>
#include <glm/vector_relational.hpp>
#include <glm/gtc/epsilon.hpp>

#include "NavMesh.h"

//===== Basic Utility Functions ======

bool NavigationUtility::IsItEqual(const glm::vec3& a, const glm::vec3& b, float epsilon)
{
    return glm::all(glm::epsilonEqual(a, b, epsilon));
}

float NavigationUtility::CrossProductXZ(const glm::vec3& v1, const glm::vec3& v2)
{
    // (x1*z2 - z1*x2) -> (x1*y2 - y1*x2) 
    return v1.x * v2.z - v1.z * v2.x;
}

int NavigationUtility::FindNodeIDByPosition(const glm::vec3& position, NavMesh& navMesh)
{
    std::vector<NavMeshOptimizedNode>& m_PathfindingNodes = navMesh.m_PathfindingNodes;
    for (int i = 0; i < static_cast<int>(m_PathfindingNodes.size()); ++i)
    {
        const auto& node = m_PathfindingNodes[i];
        if (node.polygonVerts.size() < 3)
            continue;
        const glm::vec3& verticies0 = node.polygonVerts[0];
        for (int j = 1; j < static_cast<int>(node.polygonVerts.size()) - 1; ++j)
        {
            const glm::vec3& verticies1 = node.polygonVerts[j];
            const glm::vec3& verticies2 = node.polygonVerts[j + 1];
            if (IsPointInTriangleXZ(position, verticies0, verticies1, verticies2))
                return i;
        }
    }
    return -1;
}

glm::vec3 NavigationUtility::GetNodeCenter(int nodeID, NavMesh& navMesh)
{
    std::vector<NavMeshOptimizedNode>& m_PathfindingNodes = navMesh.m_PathfindingNodes;
    if (nodeID >= 0 && nodeID < static_cast<int>(m_PathfindingNodes.size()))
    {
        return m_PathfindingNodes[nodeID].centerPoint;
    }
    return glm::vec3(0.0f);
}

const std::vector<OptimizedEdge>& NavigationUtility::GetNodeNeighbors(int nodeID, NavMesh& navMesh)
{
    std::vector<NavMeshOptimizedNode>& m_PathfindingNodes = navMesh.m_PathfindingNodes;
    if (nodeID >= 0 && nodeID < static_cast<int>(m_PathfindingNodes.size()))
        return m_PathfindingNodes[nodeID].neighbors;
    
    static const std::vector<OptimizedEdge> s_emptyNeighbors;
    return s_emptyNeighbors;
}

//===== Navigation Specific Utility Functions ======
// Barycentrik
bool NavigationUtility::IsPointInTriangleXZ(const glm::vec3& position, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    float v0x = c.x - a.x;
    float v0z = c.z - a.z;
    
    float v1x = b.x - a.x;
    float v1z = b.z - a.z;
    
    float v2x = position.x - a.x;
    float v2z = position.z - a.z;

    float dot00 = v0x * v0x + v0z * v0z;
    float dot01 = v0x * v1x + v0z * v1z;
    float dot02 = v0x * v2x + v0z * v2z;
    
    float dot11 = v1x * v1x + v1z * v1z;
    float dot12 = v1x * v2x + v1z * v2z;

    float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    
    const float epsilon = -0.000001f; 
    return (u >= epsilon) && (v >= epsilon) && (u + v <= 1.0f + epsilon);
}

bool NavigationUtility::CanClipEar(int prevIndex, int earIndex, int nextIndex, const std::vector<glm::vec3>& vertices)
{
    const glm::vec3& a = vertices[prevIndex];
    const glm::vec3& b = vertices[earIndex];
    const glm::vec3& c = vertices[nextIndex];

    for (int i = 0; i < static_cast<int>(vertices.size()); ++i)
    {
        if (i == prevIndex || i == earIndex || i == nextIndex)
            continue;
        
        const glm::vec3& currentPoint = vertices[i];

        if (IsItEqual(currentPoint, a) || IsItEqual(currentPoint, b) || IsItEqual(currentPoint, c))
            continue;

        if (IsPointInTriangleXZ(currentPoint, a, b, c))
            return false;
    }
    return true;
}

int NavigationUtility::FindOrCreateHalfEdgeVertexIndex(const glm::vec3& pos, std::vector<HalfEdgeVertex>& m_HalfEdgeVertices)
{
    for (int i = 0; i < static_cast<int>(m_HalfEdgeVertices.size()); ++i)
        if (IsItEqual(m_HalfEdgeVertices[i].position, pos))
            return i;
    
    HalfEdgeVertex newVertex;
    newVertex.position = pos;
    m_HalfEdgeVertices.push_back(newVertex);
    return static_cast<int>(m_HalfEdgeVertices.size() - 1);
}

void NavigationUtility::FindRemovableEdgeIndexes(std::vector<int>& outRemovableEdgeIndexes, std::vector<HalfEdge>& m_HalfEdges, std::vector<HalfEdgeVertex>& m_HalfEdgeVertices,
    NavMeshDebugger* m_NavMeshDebugger)
{
    std::vector<glm::vec3> removableEdgePoints;
    std::vector<glm::vec3> essentialEdgePoints;

     for (int i = 0; i < static_cast<int>(m_HalfEdges.size()); ++i)
    {
         HalfEdge& halfEdge = m_HalfEdges[i];
        
        if (halfEdge.twinHalfEdgeIndex != -1 && i < halfEdge.twinHalfEdgeIndex)
        {
            HalfEdge& twinHalfEdge = m_HalfEdges[halfEdge.twinHalfEdgeIndex];

            // start and end vertices of the edge
            const glm::vec3& pA = m_HalfEdgeVertices[halfEdge.originVertexIndex].position;
            const glm::vec3& pB = m_HalfEdgeVertices[twinHalfEdge.originVertexIndex].position;
            
            // halfEdge -> nextHalfEdge -> nextNextHalfEdge
            // vA -> vB -> vC
            // complete triangle ABC
            const HalfEdge& nextHalfEdge = m_HalfEdges[halfEdge.nextHalfEdgeIndex];
            const HalfEdge& nextNextHalfEdge = m_HalfEdges[nextHalfEdge.nextHalfEdgeIndex];
            const glm::vec3& pC = m_HalfEdgeVertices[nextNextHalfEdge.originVertexIndex].position;
            
            // twinHalfEdge -> twinNextHalfEdge -> twinNextNextHalfEdge
            // vB -> vA -> vD
            // complete triangle BAD
            const HalfEdge& twinNextHalfEdge = m_HalfEdges[twinHalfEdge.nextHalfEdgeIndex];
            const HalfEdge& twinNextNextHalfEdge = m_HalfEdges[twinNextHalfEdge.nextHalfEdgeIndex];
            const glm::vec3& pD = m_HalfEdgeVertices[twinNextNextHalfEdge.originVertexIndex].position;
            // DACB square
            
            glm::vec3 vAC = pC - pA;
            glm::vec3 vCB = pB - pC;
            glm::vec3 vBD = pD - pB;
            glm::vec3 vDA = pA - pD;

            float cross1 = CrossProductXZ(vAC, vCB);
            float cross2 = CrossProductXZ(vCB, vBD);
            float cross3 = CrossProductXZ(vBD, vDA);
            float cross4 = CrossProductXZ(vDA, vAC);
            
            const float epsilon = 0.000001f;
            bool bIsConvex = (cross1 > epsilon && cross2 > epsilon && cross3 > epsilon && cross4 > epsilon) ||
                             (cross1 < -epsilon && cross2 < -epsilon && cross3 < -epsilon && cross4 < -epsilon);
            
            if (bIsConvex && halfEdge.bIsEssential == false)
            {
                removableEdgePoints.push_back(pA);
                removableEdgePoints.push_back(pB);
                outRemovableEdgeIndexes.push_back(i);
            }
            else
            {
                essentialEdgePoints.push_back(pA);
                essentialEdgePoints.push_back(pB);
                halfEdge.bIsEssential = true;
                twinHalfEdge.bIsEssential = true;
            }
        }
    }

    std::cout << "Found " << removableEdgePoints.size() / 2 << " removable (green) edges." << std::endl;
    std::cout << "Found " << essentialEdgePoints.size() / 2 << " essential (red) edges." << std::endl;
    m_NavMeshDebugger->SetRemovableEdges(removableEdgePoints);
    m_NavMeshDebugger->SetCannotRemoveEdges(essentialEdgePoints);
}

//===== Face Merging Utility Functions ======

NavigationUtility::DummyFaceBoundaryData NavigationUtility::CollectFaceBoundaryVertices(int faceIndex, std::vector<HalfEdge>& m_HalfEdges, std::vector<HalfEdgeFace>& m_HalfEdgeFaces)
{
    DummyFaceBoundaryData data;
    const HalfEdgeFace& face = m_HalfEdgeFaces[faceIndex];

    int startEdge = face.halfEdgeIndex;
    int currentEdge = startEdge;

    std::vector<char> visitedEdges(m_HalfEdges.size(), 0);
    for (int i = 0; i < static_cast<int>(m_HalfEdges.size()); ++i)
    {
        if (visitedEdges[currentEdge])
        {
            std::cout << " CollectFaceBoundaryVertices: visitededges" << std::endl;
            break;
        }
        visitedEdges[currentEdge] = 1;
        const HalfEdge& halfEdge = m_HalfEdges[currentEdge];

        data.edgeLoop.push_back(currentEdge);
        data.vertexLoop.push_back(halfEdge.originVertexIndex);

        currentEdge = halfEdge.nextHalfEdgeIndex;
        if (currentEdge == startEdge)
            break;
    }
    
    return data;
}

std::vector<int> NavigationUtility::BuildMergedPolygonVerts(const DummyFaceBoundaryData& faceA, const DummyFaceBoundaryData& faceB, int halfEdgeAB, int halfEdgeBA, std::vector<HalfEdge>& m_HalfEdges)
{
    std::vector<int> dummyMergedVertexLoop;
    
    int aCutStart = -1;
    for (int i = 0; i < static_cast<int>(faceA.edgeLoop.size()); ++i) // i am finding the start position for cutting then i will start to collect vertices until i reach the cut position again
    {
        if (faceA.edgeLoop[i] == halfEdgeAB)
        {
            aCutStart = i;
            break;
        }
    }
    if (aCutStart == -1)
        return dummyMergedVertexLoop;
    
    int bCutStart = -1;
    for (int i = 0; i < static_cast<int>(faceB.edgeLoop.size()); ++i)
    {
        if (faceB.edgeLoop[i] == halfEdgeBA)
        {
            bCutStart = i;
            break;
        }
    }
    if (bCutStart == -1)
        return dummyMergedVertexLoop;
    
    {
        int currentID = (aCutStart + 1) % static_cast<int>(faceA.edgeLoop.size());
        while (currentID != aCutStart)
        {
            int halfEdgeIndexA = faceA.edgeLoop[currentID];
            int originVertexID = m_HalfEdges[halfEdgeIndexA].originVertexIndex;
            dummyMergedVertexLoop.push_back(originVertexID);

            currentID = (currentID + 1) % static_cast<int>(faceA.edgeLoop.size());
        }
    }
    
    {
        int currentID = (bCutStart + 1) % static_cast<int>(faceB.edgeLoop.size());
        while (currentID != bCutStart)
        {
            int halfEdgeIndexB = faceB.edgeLoop[currentID];
            int originVertexID = m_HalfEdges[halfEdgeIndexB].originVertexIndex;
            dummyMergedVertexLoop.push_back(originVertexID);

            currentID = (currentID + 1) % static_cast<int>(faceB.edgeLoop.size());
        }
    }
    return dummyMergedVertexLoop;
}

void NavigationUtility::RebuildFaceFromPolygon(int faceKeepIndex, const std::vector<int>& mergedVertexLoop, std::vector<HalfEdge>& HalfEdges, std::vector<HalfEdgeFace>& HalfEdgeFaces)
{
    if (mergedVertexLoop.size() < 3)
    {
        HalfEdgeFaces[faceKeepIndex].bIsValid = false;
        return;
    }
    
    {
        DummyFaceBoundaryData oldFaceData = CollectFaceBoundaryVertices(faceKeepIndex, HalfEdges, HalfEdgeFaces);
        for (int halfEdgeIndex : oldFaceData.edgeLoop)
            HalfEdges[halfEdgeIndex].faceID = -1;
    }
    
    int firstNewHalfEdgeIndex = static_cast<int>(HalfEdges.size());
    int polygonEdgeCount = static_cast<int>(mergedVertexLoop.size());
    
    for (int i = 0; i < polygonEdgeCount; ++i)
    {
        HalfEdge newHalfEdge;
        newHalfEdge.originVertexIndex = mergedVertexLoop[i];
        newHalfEdge.faceID = faceKeepIndex;
        newHalfEdge.twinHalfEdgeIndex = -1;
        newHalfEdge.nextHalfEdgeIndex = -1;
        HalfEdges.push_back(newHalfEdge);
    }
    
    for (int i = 0; i < polygonEdgeCount; ++i)
    {
        int currentHalfEdge = firstNewHalfEdgeIndex + i;
        int nextHalfEdge = firstNewHalfEdgeIndex + ((i + 1) % polygonEdgeCount);
        HalfEdges[currentHalfEdge].nextHalfEdgeIndex = nextHalfEdge;
    }
    
    HalfEdgeFaces[faceKeepIndex].bIsValid = true;
    HalfEdgeFaces[faceKeepIndex].halfEdgeIndex = firstNewHalfEdgeIndex;
}

void NavigationUtility::MergeTwoFacesProperley(int removeHalfEdgeABIndex, std::vector<HalfEdge>& HalfEdges, std::vector<HalfEdgeFace>& HalfEdgeFaces, std::vector<HalfEdgeVertex>& HalfEdgeVertices)
{
    HalfEdge& halfEdgeAB = HalfEdges[removeHalfEdgeABIndex];
    int halfEdgeBAID = halfEdgeAB.twinHalfEdgeIndex;
    if (halfEdgeBAID < 0)
        return;

    HalfEdge& halfEdgeBA = HalfEdges[halfEdgeBAID];

    int faceA = halfEdgeAB.faceID;
    int faceB = halfEdgeBA.faceID;
    if (faceA < 0 || faceB < 0)
        return;
    
    if (!HalfEdgeFaces[faceA].bIsValid || !HalfEdgeFaces[faceB].bIsValid)
        return;
    
    DummyFaceBoundaryData dummyDataA = CollectFaceBoundaryVertices(faceA, HalfEdges, HalfEdgeFaces);
    DummyFaceBoundaryData dummyDataB = CollectFaceBoundaryVertices(faceB, HalfEdges, HalfEdgeFaces);
    
    std::vector<int> mergedLoop = BuildMergedPolygonVerts(dummyDataA, dummyDataB, removeHalfEdgeABIndex, halfEdgeBAID, HalfEdges);
    
    RebuildFaceFromPolygon(faceA, mergedLoop, HalfEdges, HalfEdgeFaces);
    
    HalfEdgeFaces[faceB].bIsValid = false;
    HalfEdgeFaces[faceB].halfEdgeIndex = -1;
    
    halfEdgeAB.faceID = -1;
    halfEdgeBA.faceID = -1;
}

//===== Raycasting and Obstacle Utility Functions ======

bool NavigationUtility::RaycastXZ(const std::vector<glm::vec3>& navMeshVertices, const glm::vec3& rayP1, const glm::vec3& rayP2, NavMeshHitInfo& hitInfo)
{
    if (navMeshVertices.empty())
        return false;

    std::vector<NavMeshHitInfo> hits;
    
    float rayMinX = std::min(rayP1.x, rayP2.x);
    float rayMaxX = std::max(rayP1.x, rayP2.x);
    float rayMinZ = std::min(rayP1.z, rayP2.z);
    float rayMaxZ = std::max(rayP1.z, rayP2.z);
    
    for (int i = 0; i < static_cast<int>(navMeshVertices.size()); ++i)
    {
        const glm::vec3& point1 = navMeshVertices[i];
        const glm::vec3& point2 = navMeshVertices[(i + 1) % navMeshVertices.size()];
        
        float lineMinX = std::min(point1.x, point2.x);
        float lineMaxX = std::max(point1.x, point2.x);
        float lineMinZ = std::min(point1.z, point2.z);
        float lineMaxZ = std::max(point1.z, point2.z);
        
        if (rayMaxX < lineMinX || rayMinX > lineMaxX || rayMaxZ < lineMinZ || rayMinZ > lineMaxZ)
            continue;
        
        const glm::vec3 p1p2 = rayP2 - rayP1;
        const glm::vec3 q1q2 = point2 - point1;
        const float crossOfEdgesAndRay = CrossProductXZ(p1p2, q1q2);
        if (std::abs(crossOfEdgesAndRay) < 0.000001f)
            continue;
        float proccessOnRay = CrossProductXZ((point1 - rayP1), q1q2) / crossOfEdgesAndRay;
        
        if (proccessOnRay > 0.000001f && proccessOnRay <= 1.0f)
        {
            NavMeshHitInfo currentHit;
            currentHit.proccessOnRay = proccessOnRay;
            currentHit.hitPoint = rayP1 + p1p2 * proccessOnRay;
            currentHit.edgeP1 = point1;
            currentHit.edgeP2 = point2;
            hits.push_back(currentHit);
        }
    }

    if (hits.empty())
        return false;
    
    hitInfo = *std::min_element(hits.begin(), hits.end(), [](const NavMeshHitInfo& a, const NavMeshHitInfo& b)
    {
        return a.proccessOnRay < b.proccessOnRay;
    });
    
    return true;
}

std::vector<std::vector<glm::vec3>> NavigationUtility::GetSceneObstacleSlices(float buildPlaneY, NavBuildParams& buildParams, const Scene& scene)
{
    std::vector<std::vector<glm::vec3>> allSlices;
    const float radius = buildParams.agentRadius;
    
    const std::vector<Vec3f> localCubeVerts =
    {
        {-0.5f, -0.5f, -0.5f}, // 0
        { 0.5f, -0.5f, -0.5f}, // 1
        { 0.5f,  0.5f, -0.5f}, // 2
        {-0.5f,  0.5f, -0.5f}, // 3
        {-0.5f, -0.5f,  0.5f}, // 4
        { 0.5f, -0.5f,  0.5f}, // 5
        { 0.5f,  0.5f,  0.5f}, // 6
        {-0.5f,  0.5f,  0.5f}  // 7
    };
    
    for (const auto& object : scene.GetObjects())
    {
        glm::vec3 minBounds(std::numeric_limits<float>::max());
        glm::vec3 maxBounds(std::numeric_limits<float>::lowest());
        
        for (const auto& localVert : localCubeVerts)
        {
            glm::vec4 worldPos = object.modelMatrix * glm::vec4(localVert.x, localVert.y, localVert.z, 1.0f);
            
            minBounds.x = std::min(minBounds.x, worldPos.x);
            minBounds.y = std::min(minBounds.y, worldPos.y);
            minBounds.z = std::min(minBounds.z, worldPos.z);
            
            maxBounds.x = std::max(maxBounds.x, worldPos.x);
            maxBounds.y = std::max(maxBounds.y, worldPos.y);
            maxBounds.z = std::max(maxBounds.z, worldPos.z);
        }
        
        if (buildPlaneY < minBounds.y || buildPlaneY > maxBounds.y)
            continue;

        maxBounds.x += radius;
        maxBounds.z += radius;
        minBounds.x -= radius;
        minBounds.z -= radius;
        
        std::vector<glm::vec3> sliceFootprint;
        const float y = buildPlaneY;
        
        // (minX, minZ) -> (maxX, minZ) -> (maxX, maxZ) -> (minX, maxZ)
        sliceFootprint.push_back(glm::vec3(minBounds.x, y, minBounds.z));
        sliceFootprint.push_back(glm::vec3(maxBounds.x, y, minBounds.z));
        sliceFootprint.push_back(glm::vec3(maxBounds.x, y, maxBounds.z));
        sliceFootprint.push_back(glm::vec3(minBounds.x, y, maxBounds.z));
        
        allSlices.push_back(sliceFootprint);
    }

    return allSlices;
}

void NavigationUtility::SortObstaclesByMaxX(std::vector<std::vector<glm::vec3>>& obstacleSlices)
{
    auto compareX = [](const glm::vec3& a, const glm::vec3& b)
    {
        return a.x < b.x;
    };
    
    std::sort(obstacleSlices.begin(), obstacleSlices.end(), [&compareX](const std::vector<glm::vec3>& left, const std::vector<glm::vec3>& right) 
    {
        auto maxLeft = std::max_element(left.begin(), left.end(), compareX);
        auto maxRight = std::max_element(right.begin(), right.end(), compareX);
        
        if (maxLeft == left.end())
            return false;
        if (maxRight == right.end())
            return true;
        
        return maxLeft->x > maxRight->x;
    });
}

void NavigationUtility::CreateHolesWithObstacles(const std::vector<glm::vec3>& obstacleSlice, std::vector<glm::vec3>& navMeshVerts3D, const NavBuildParams& buildParams)
{
    if (obstacleSlice.empty())
        return;

    const glm::vec3 objectRayPoint = *std::max_element(obstacleSlice.begin(), obstacleSlice.end(),[](const glm::vec3& a, const glm::vec3& b)
        {
            return a.x < b.x;
        });
    const float rayLenght = (buildParams.maxBounds.x - buildParams.origin.x) * 2.0f;
    const glm::vec3 rayEndPoint = objectRayPoint + glm::vec3(rayLenght, 0.0f, 0.0f);
    
    NavMeshHitInfo hitInfo;
    if (!RaycastXZ(navMeshVerts3D, objectRayPoint, rayEndPoint, hitInfo))
    {
        std::cerr << "Obstacle line didnt hit border" << std::endl;
        return;
    }
    
    glm::vec3 connectToPos = hitInfo.edgeP1;

    NavMeshHitInfo tempHitInfo;
    bool bIsVisible = false;
    while (!bIsVisible)
        if (RaycastXZ(navMeshVerts3D, objectRayPoint, connectToPos,tempHitInfo))
            if (IsItEqual(tempHitInfo.hitPoint, connectToPos))
                bIsVisible = true;
            else
            {
                hitInfo = tempHitInfo;
                connectToPos = hitInfo.edgeP2;
            }
        else
            bIsVisible = true;
    

    std::vector<int> indices;
    for (int i = 0; i < static_cast<int>(navMeshVerts3D.size()); ++i)
        if (NavigationUtility::IsItEqual(navMeshVerts3D[i], connectToPos))
            indices.push_back(i);
    
    if (indices.empty())
    {
        std::cerr << "The hit point is not found in the main list" << std::endl;
        return;
    }

    int connectToIndex = indices.back();
    const float mostRightPointZ = objectRayPoint.z;

    for (int index : indices)
    {
        int nextIndex = (index + 1) % static_cast<int>(navMeshVerts3D.size());
        if (navMeshVerts3D[nextIndex].z > mostRightPointZ)
        {
            connectToIndex = index;
            break;
        }
    }                         
    
    auto connectToIt = navMeshVerts3D.begin() + connectToIndex;
    
    std::vector<glm::vec3> verticesToInsert = obstacleSlice;
    
    std::reverse(verticesToInsert.begin(), verticesToInsert.end());
    auto mostRightPointIt_CW = std::find_if(verticesToInsert.begin(), verticesToInsert.end(), 
        [&objectRayPoint](const glm::vec3& v)
        {
            return IsItEqual(v, objectRayPoint);
        });
    if (mostRightPointIt_CW == verticesToInsert.end())
    {
        std::cout << "Internal error: mostRightPoint not found after reverse." << std::endl;
        return; 
    }
    std::rotate(verticesToInsert.begin(), mostRightPointIt_CW, verticesToInsert.end());
    
    verticesToInsert.push_back(verticesToInsert[0]); 
    verticesToInsert.push_back(*connectToIt);
    
    navMeshVerts3D.insert(connectToIt + 1, verticesToInsert.begin(), verticesToInsert.end());
}

std::vector<NavMeshOptimizedNode>& NavigationUtility::GetPathfindingNodes(NavMesh& navMesh)
{
    return navMesh.m_PathfindingNodes;
}
