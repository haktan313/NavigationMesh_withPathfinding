#include "NavMesh.h"
#include <glm/gtc/epsilon.hpp>
#include <algorithm>
#include <deque>
#include <iostream>
#include <queue>

#include "FunnelPathSmoothing.h"
#include "Pathfinder.h"


NavMesh::NavMesh(const Scene& scene)
    : m_NavMeshDebugger(nullptr), m_Scene(scene), Start(glm::vec3(0.f, 0.0f,9.0f)), End(glm::vec3(14.f,0.0f,-1.0f))
{
    CreateDebugger();
    BuildBorderFromParams();
}

NavMesh::~NavMesh()
{
    delete m_NavMeshDebugger;
    m_NavMeshDebugger = nullptr;
    
}

void NavMesh::BuildNavMesh()
{
    m_HalfEdges.clear();
    m_HalfEdgeFaces.clear();
    m_HalfEdgeVertices.clear();
    m_NavMeshTriangles.clear();
    m_PathfindingNodes.clear();
    m_NavMeshVerts3D.clear();
    m_NavMeshVerts3D = m_BorderVerts3D;
    
    m_NavMeshDebugger->CleanBuffers();
    m_NavMeshDebugger->bNavMeshBuilt = true;
    
    BuildInputTriangles(m_Scene);
    VoxelizeInputTriangles();
    BuildHeightField();
    FilterWalkableSurfaces();
    BuildRegions();
    
    /*const float buildPlaneY = std::min(BuildParams.minCorner.y, BuildParams.maxCorner.y);
    std::vector<std::vector<glm::vec3>> obstacleSlices = NavigationUtility::GetSceneObstacleSlices(buildPlaneY, BuildParams, m_Scene); // getting obstacle slices depending on build plane Y which is Y position of navmesh
    NavigationUtility::SortObstaclesByMaxX(obstacleSlices);
    for (const auto& slice : obstacleSlices)
        NavigationUtility::CreateHolesWithObstacles(slice, m_NavMeshVerts3D, BuildParams);
    
    EarClipping();
    CreateHalfEdgeStructure();
    OptimizeEarClipping();// hertel mehlhorn
    SetStartEndMarkers(Start, End);
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->SetHoles(obstacleSlices);*/
}

void NavMesh::BuildInputTriangles(const Scene& scene)
{
    m_InputTriangles.clear();
    const auto& objects = scene.GetObjects();

    for (const auto& obj : objects)
    {
        if (!obj.mesh)
            continue;
        const glm::mat4& modelMatrix = obj.modelMatrix;
        const MeshData* mesh = obj.mesh;

        for (size_t i = 0; i < mesh->indices.size() / 3; ++i)
        {
            unsigned int idx0 = mesh->indices[i * 3];
            unsigned int idx1 = mesh->indices[i * 3 + 1];
            unsigned int idx2 = mesh->indices[i * 3 + 2];

            const Vec3f& local_v0 = mesh->vertices[idx0];
            const Vec3f& local_v1 = mesh->vertices[idx1];
            const Vec3f& local_v2 = mesh->vertices[idx2];

            glm::vec4 world_v0 = modelMatrix * glm::vec4(local_v0.x, local_v0.y, local_v0.z, 1.0f);
            glm::vec4 world_v1 = modelMatrix * glm::vec4(local_v1.x, local_v1.y, local_v1.z, 1.0f);
            glm::vec4 world_v2 = modelMatrix * glm::vec4(local_v2.x, local_v2.y, local_v2.z, 1.0f);

            Triangle tri;
            tri.verts[0] = { world_v0.x, world_v0.y, world_v0.z };
            tri.verts[1] = { world_v1.x, world_v1.y, world_v1.z };
            tri.verts[2] = { world_v2.x, world_v2.y, world_v2.z };
            m_InputTriangles.push_back(tri);
        }
    }
}

void NavMesh::VoxelizeInputTriangles()
{
    std::cout << "Voxelization step (placeholder)..." << std::endl;
    if (m_InputTriangles.empty())
    {
        std::cout << "No input triangles to voxelize." << std::endl;
        return;
    }

    if (BuildParams.minCorner.x >= BuildParams.maxCorner.x ||
        BuildParams.minCorner.y >= BuildParams.maxCorner.y ||
        BuildParams.minCorner.z >= BuildParams.maxCorner.z ||
        BuildParams.cellSize <= 0.0f || BuildParams.cellHeight <= 0.0f)
    {
        std::cout << "Invalid voxel grid parameters." << std::endl;
        return;
    }
    int totalVoxels = BuildParams.width * BuildParams.depth * BuildParams.height;
    std::cout << "Voxel Grid Dimensions: " << BuildParams.width << " x " << BuildParams.depth << " x " << BuildParams.height << " = " << totalVoxels << " voxels." << std::endl;

    Rasterization();
}

void NavMesh::Rasterization()
{
    int solidVoxels = 0;
    for (const auto& tri : m_InputTriangles)
    {
        float triMin[3] = { tri.verts[0].x, tri.verts[0].y, tri.verts[0].z };
        float triMax[3] = { tri.verts[0].x, tri.verts[0].y, tri.verts[0].z };
        for (int i = 1; i < 3; ++i)
        {
            triMin[0] = std::min(triMin[0], tri.verts[i].x);
            triMin[1] = std::min(triMin[1], tri.verts[i].y);
            triMin[2] = std::min(triMin[2], tri.verts[i].z);
            
            triMax[0] = std::max(triMax[0], tri.verts[i].x);
            triMax[1] = std::max(triMax[1], tri.verts[i].y);
            triMax[2] = std::max(triMax[2], tri.verts[i].z);
        }

        int minX = (int)((triMin[0] - BuildParams.minCorner.x) / BuildParams.cellSize);
        int minY = (int)((triMin[1] - BuildParams.minCorner.y) / BuildParams.cellHeight);
        int minZ = (int)((triMin[2] - BuildParams.minCorner.z) / BuildParams.cellSize);
        
        int maxX = (int)((triMax[0] - BuildParams.minCorner.x) / BuildParams.cellSize);
        int maxY = (int)((triMax[1] - BuildParams.minCorner.y) / BuildParams.cellHeight);
        int maxZ = (int)((triMax[2] - BuildParams.minCorner.z) / BuildParams.cellSize);

        minX = std::max(0, minX);
        minY = std::max(0, minY);
        minZ = std::max(0, minZ);

        maxX = std::min(BuildParams.width - 1, maxX);
        maxY = std::min(BuildParams.height - 1, maxY);
        maxZ = std::min(BuildParams.depth - 1, maxZ);

        for (int z = minZ; z <= maxZ; ++z)
        {
            for (int y = minY; y <= maxY; ++y)
            {
                for (int x = minX; x <= maxX; ++x)
                {
                    int index = x + z * BuildParams.width + y * BuildParams.width * BuildParams.depth;
                    if (BuildParams.data[index])
                        continue;

                    float boxcenter[3] = {
                        BuildParams.minCorner.x + (x + 0.5f) * BuildParams.cellSize,
                        BuildParams.minCorner.y + (y + 0.5f) * BuildParams.cellHeight,
                        BuildParams.minCorner.z + (z + 0.5f) * BuildParams.cellSize
                    };
                    float boxhalfsize[3] = {
                        BuildParams.cellSize * 0.5f,
                        BuildParams.cellHeight * 0.5f,
                        BuildParams.cellSize * 0.5f
                    };
                    float triverts[3][3] = {
                        { tri.verts[0].x, tri.verts[0].y, tri.verts[0].z },
                        { tri.verts[1].x, tri.verts[1].y, tri.verts[1].z },
                        { tri.verts[2].x, tri.verts[2].y, tri.verts[2].z }
                    };

                    if (NavigationUtility::TriBoxOverlap(boxcenter, boxhalfsize, triverts))
                    {
                        BuildParams.data[index] = true;
                        solidVoxels++;
                    }
                }
            }
        }
    }
    std::cout << "Rasterization complete. Solid voxels: " << solidVoxels << std::endl;
}

void NavMesh::BuildHeightField()
{
    int width = BuildParams.width;
    int depth = BuildParams.depth;
    int height = BuildParams.height;
    int numColumns = width * depth;
    
    m_HeightField.spans = new HeightFieldSpan*[numColumns];
    memset(m_HeightField.spans, 0, sizeof(HeightFieldSpan*) * numColumns);
    
    m_HeightField.spanPool.clear();
    m_HeightField.spanPool.reserve(numColumns * height);

    for (int z = 0; z < depth; ++z)
        for (int x = 0; x < width; ++x)
        {
            HeightFieldSpan* previousSpan = nullptr;
            for (int y = 0; y < height; ++y)
            {
                int index = x + (z * width) + (y * width * depth);
                bool bisSolidCurrent = BuildParams.data[index];
                
                int previousIndex = index - (width * depth);
                bool bisSolidPrevious = (y > 0) ? BuildParams.data[previousIndex] : false;
                
                if (bisSolidCurrent != bisSolidPrevious)
                    if (bisSolidCurrent)
                    {
                        HeightFieldSpan newSpan;
                        newSpan.spanMin = y;
                        newSpan.spanMax = y;
                        newSpan.areaID = 0;
                        newSpan.next = nullptr;

                        m_HeightField.spanPool.push_back(newSpan);
                        HeightFieldSpan* currentSpan = &m_HeightField.spanPool.back();

                        if (previousSpan)
                            previousSpan->next = currentSpan;
                        else
                            m_HeightField.spans[x + z * width] = currentSpan;
                        previousSpan = currentSpan;
                    }
                if (bisSolidCurrent && previousSpan)
                    previousSpan->spanMax = y;
            }
        }

    std::cout << "Heightfield built with " << m_HeightField.spanPool.size() << " spans." << std::endl;
}

void NavMesh::FilterWalkableSurfaces()
{
    const int walkableHeight = (int)ceilf(BuildParams.agentHeight / BuildParams.cellHeight);
    for (auto& span : m_HeightField.spanPool)
        span.areaID = 0;
    
    for (int z = 0; z < BuildParams.depth; ++z)
        for (int x = 0; x < BuildParams.width; ++x)
            for (HeightFieldSpan* span = m_HeightField.spans[x + (z * BuildParams.width)]; span; span = span->next)
            {
                if (span->next != nullptr)
                    continue;
                const int headroom = BuildParams.height - (int)span->spanMax;
                if (headroom >= walkableHeight)
                    span->areaID = 1;
            }

    std::cout << "Walkable surfaces filtered." << std::endl;
}

void NavMesh::BuildRegions()
{
    std::cout << "Building regions..." << std::endl;
    if (BuildParams.width == 0 || BuildParams.depth == 0)
        return;

    const int walkableClimb = BuildParams.maxClimb > 0 ? (int)floorf(BuildParams.maxClimb / BuildParams.cellHeight) : 0;
    
    unsigned int regionId = 2;
    
    struct SpanLocation {
        int x, z;
        HeightFieldSpan* span;
    };
    
    for (int z = 0; z < BuildParams.depth; ++z)
        for (int x = 0; x < BuildParams.width; ++x)
            for (HeightFieldSpan* span = m_HeightField.spans[x + (z * BuildParams.width)]; span; span = span->next)
            {
                if (span->areaID == 1)
                {
                    std::queue<SpanLocation> openList;
                    openList.push({x, z, span});
                    span->areaID = regionId;

                    while (!openList.empty())
                    {
                        SpanLocation current = openList.front();
                        openList.pop();
                        
                        for (int dir = 0; dir < 4; ++dir)
                        {
                            int dirX[] = {-1, 0, 1, 0};
                            int dirZ[] = {0, -1, 0, 1};
                            int neighborX = current.x + dirX[dir];
                            int neighborZ = current.z + dirZ[dir];
                            
                            if (neighborX < 0 || neighborZ < 0 || neighborX >= BuildParams.width || neighborZ >= BuildParams.depth)
                                continue;
                            
                            for (HeightFieldSpan* neighborSpan = m_HeightField.spans[neighborX + neighborZ * BuildParams.width]; neighborSpan; neighborSpan = neighborSpan->next)
                                if (neighborSpan->areaID == 1)
                                {
                                    const int heightDiff = abs((int)current.span->spanMax - (int)neighborSpan->spanMax);
                                    if (heightDiff <= walkableClimb)
                                    {
                                        neighborSpan->areaID = regionId;
                                        openList.push({neighborX, neighborZ, neighborSpan});
                                    }
                                }
                        }
                    }
                    regionId++;
                }
            }

    std::cout << "Regions built. Total regions found: " << regionId - 2 << std::endl;
}

//----- Render the debug tool -----

void NavMesh::RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene)
{
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->RenderDebugTool(shader, camera, scene, DebugInfo, BuildParams, m_HeightField);
    
}

void NavMesh::CreateDebugger()
{
    if (!m_NavMeshDebugger)
        m_NavMeshDebugger = new NavMeshDebugger();
    if (m_NavMeshDebugger)
        SetStartEndMarkers(Start, End);
    
    BuildParams.depth = static_cast<int>((BuildParams.maxCorner.z - BuildParams.minCorner.z) / BuildParams.cellSize);
    BuildParams.width = static_cast<int>((BuildParams.maxCorner.x - BuildParams.minCorner.x) / BuildParams.cellSize);
    BuildParams.height = static_cast<int>((BuildParams.maxCorner.y - BuildParams.minCorner.y) / BuildParams.cellHeight);
    BuildParams.data.resize(BuildParams.width * BuildParams.depth * BuildParams.height, false);
}

void NavMesh::SetStartEndMarkers(const glm::vec3& start, const glm::vec3& end)
{
    Start = start;
    End = end;
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->SetStartEndMarkers(Start, End);
    int startNodeID = NavigationUtility::FindNodeIDByPosition(Start, *this);
    int endNodeID = NavigationUtility::FindNodeIDByPosition(End, *this);
    std::cout << "Start Node ID: " << startNodeID << ", End Node ID: " << endNodeID << std::endl;

    PathfindingResult result = Pathfinder::FindPath(*this, Start, End);
    if (!result.bPathFound)
        return;
    if (m_NavMeshDebugger && result.pathPoints.size() > 0)
        m_NavMeshDebugger->SetPath(result.pathPoints);
    
    result.smoothPathPoints = FunnelPathSmoothing::SmoothPath(result.pathNodeIDs, *this);
    if (m_NavMeshDebugger && result.smoothPathPoints.size() > 0)
        m_NavMeshDebugger->SetSmoothedPath(result.smoothPathPoints);
}

//----- Build Functions -----

void NavMesh::BuildBorderFromParams()
{
    m_BorderVerts3D.clear();
    
    glm::vec3 origin = BuildParams.minCorner;
    glm::vec3 maxBounds = BuildParams.maxCorner;
    const float radius = BuildParams.agentRadius;

    glm::vec3 minPosition
    {
        std::min(origin.x,maxBounds.x) + radius,
        std::min(origin.y,maxBounds.y),
        std::min(origin.z,maxBounds.z) + radius
    };
    glm::vec3 maxPosition
    {
        std::max(origin.x,maxBounds.x) - radius,
        std::max(origin.y,maxBounds.y),
        std::max(origin.z,maxBounds.z) - radius
    };
    float y = minPosition.y;
    
    // CCW: (minX,minZ) -> (maxX,minZ) -> (maxX,maxZ) -> (minX,maxZ)
    glm::vec3 p0{minPosition.x, y, minPosition.z};
    glm::vec3 p1{maxPosition.x, y, minPosition.z};
    glm::vec3 p2{maxPosition.x, y, maxPosition.z};
    glm::vec3 p3{minPosition.x, y, maxPosition.z};

    m_BorderVerts3D = { p0, p1, p2, p3 };
    if (m_NavMeshDebugger)
        m_NavMeshDebugger->SetBorderVertices(m_BorderVerts3D);
    m_NavMeshVerts3D = m_BorderVerts3D;
}

void NavMesh::EarClipping()
{
    bool bClippedEar = false;
    
    while (m_NavMeshVerts3D.size() >= 3)
    {
        bClippedEar = false;
        for (int i = 0; i < static_cast<int>(m_NavMeshVerts3D.size()); i++)
        {
            int prevIndex = i - 1;
            if (prevIndex < 0)
                prevIndex += static_cast<int>(m_NavMeshVerts3D.size());
            int nextIndex = (i + 1) % static_cast<int>(m_NavMeshVerts3D.size());

            std::vector<glm::vec2> triangleDummy
            {
                {m_NavMeshVerts3D[prevIndex].x, m_NavMeshVerts3D[prevIndex].z},
                {m_NavMeshVerts3D[i].x, m_NavMeshVerts3D[i].z},
                {m_NavMeshVerts3D[nextIndex].x, m_NavMeshVerts3D[nextIndex].z}
            };
            glm::vec2 vector1 = {triangleDummy[1].x - triangleDummy[0].x, triangleDummy[1].y - triangleDummy[0].y};
            glm::vec2 vector2 = {triangleDummy[2].x - triangleDummy[1].x, triangleDummy[2].y - triangleDummy[1].y};
            float cross = vector1.x * vector2.y - vector1.y * vector2.x;
            
            if (cross > 0.f)
            {
                if (NavigationUtility::CanClipEar(prevIndex, i, nextIndex, m_NavMeshVerts3D))
                {
                    if (m_NavMeshDebugger)
                    {
                        NavMeshTriangle triangle = 
                        {
                            m_NavMeshVerts3D[prevIndex],
                            m_NavMeshVerts3D[i],
                            m_NavMeshVerts3D[nextIndex]
                        };
                        m_NavMeshTriangles.push_back(triangle);
                    }
                    bClippedEar = true;
                    m_NavMeshVerts3D.erase(m_NavMeshVerts3D.begin() + i);
                    break;
                }
            }
        }
        if (!bClippedEar)
        {
            m_NavMeshDebugger->SetTrianglesFill(m_NavMeshTriangles);
            std::cout << "Ear Clipping failed! No valid ear found." << std::endl;
            return;
        }
    }
    m_NavMeshDebugger->SetTrianglesFill(m_NavMeshTriangles);
}

void NavMesh::CreateHalfEdgeStructure()
{
    m_HalfEdgeVertices.clear();
    m_HalfEdgeFaces.clear();
    m_HalfEdges.clear();

    for (const auto& triangle : m_NavMeshTriangles)
    {
        int vertex0Index = NavigationUtility::FindOrCreateHalfEdgeVertexIndex(triangle.verts[0], m_HalfEdgeVertices);
        int vertex1Index = NavigationUtility::FindOrCreateHalfEdgeVertexIndex(triangle.verts[1], m_HalfEdgeVertices);
        int vertex2Index = NavigationUtility::FindOrCreateHalfEdgeVertexIndex(triangle.verts[2], m_HalfEdgeVertices);

        m_HalfEdgeFaces.push_back(HalfEdgeFace());
        HalfEdgeFace& newFace = m_HalfEdgeFaces.back();
        int faceIndex = static_cast<int>(m_HalfEdgeFaces.size() - 1);

        newFace.halfEdgeIndex = static_cast<int>(m_HalfEdges.size());
        m_HalfEdges.push_back(HalfEdge({vertex0Index, static_cast<int>(m_HalfEdges.size()) + 1, faceIndex}));
        if (m_HalfEdgeVertices[vertex0Index].halfEdgeIndex == -1)
            m_HalfEdgeVertices[vertex0Index].halfEdgeIndex = static_cast<int>(m_HalfEdges.size());
        
        m_HalfEdges.push_back(HalfEdge({vertex1Index, static_cast<int>(m_HalfEdges.size()) + 1, faceIndex}));
        if (m_HalfEdgeVertices[vertex1Index].halfEdgeIndex == -1)
            m_HalfEdgeVertices[vertex1Index].halfEdgeIndex = static_cast<int>(m_HalfEdges.size());
        
        m_HalfEdges.push_back(HalfEdge({vertex2Index, static_cast<int>(m_HalfEdges.size()) - 2, faceIndex}));
        if (m_HalfEdgeVertices[vertex2Index].halfEdgeIndex == -1)
            m_HalfEdgeVertices[vertex2Index].halfEdgeIndex = static_cast<int>(m_HalfEdges.size());
    }
    std::cout << "HalfEdge structure created with " << m_HalfEdgeVertices.size() << " vertices, " << m_HalfEdges.size() << " halfedges, and " << m_HalfEdgeFaces.size() << " faces." << std::endl;
    FindTwinHalfEdges();
}

void NavMesh::FindTwinHalfEdges()
{
    std::map<std::pair<int, int>, int> edgeMap; // Key: (start, end vertex), Value: halfEdgeIndex
    for (int i = 0; i < static_cast<int>(m_HalfEdges.size()); i++)
    {
        const HalfEdge& halfEdge = m_HalfEdges[i];
        int endVertexIndex = m_HalfEdges[halfEdge.nextHalfEdgeIndex].originVertexIndex;
        edgeMap[{halfEdge.originVertexIndex, endVertexIndex}] = i;
    }
    
    int twinCount = 0;
    for (int i = 0; i < static_cast<int>(m_HalfEdges.size()); i++)
    {
        HalfEdge& halfEdge = m_HalfEdges[i];
        if (halfEdge.twinHalfEdgeIndex != -1)
            continue; // already assigned
        int endVertexIndex = m_HalfEdges[halfEdge.nextHalfEdgeIndex].originVertexIndex;
        
        auto twinValue = edgeMap.find({endVertexIndex, halfEdge.originVertexIndex});
        if (twinValue != edgeMap.end())
        {
            int twinIndex = twinValue->second;
            HalfEdge& twinHalfEdge = m_HalfEdges[twinIndex];
            halfEdge.twinHalfEdgeIndex = twinIndex;
            twinHalfEdge.twinHalfEdgeIndex = i;
            twinCount++;
        }
    }
    std::cout << "Found and assigned " << twinCount << " twin halfedges." << std::endl;

    //For debugging
    std::vector<glm::vec3> internalEdgesLines;
    for (int i = 0; i < static_cast<int>(m_HalfEdges.size()); ++i)
    {
        const HalfEdge& halfEdge = m_HalfEdges[i];
        
        if (halfEdge.twinHalfEdgeIndex != -1)
        {
            if (i < halfEdge.twinHalfEdgeIndex)
            {
                const glm::vec3& p1 = m_HalfEdgeVertices[halfEdge.originVertexIndex].position;
                
                const HalfEdge& nextHalfEdge = m_HalfEdges[halfEdge.nextHalfEdgeIndex];
                const glm::vec3& p2 = m_HalfEdgeVertices[nextHalfEdge.originVertexIndex].position;
    
                internalEdgesLines.push_back(p1);
                internalEdgesLines.push_back(p2);
            }
        }
    }
    std::cout << "Found " << internalEdgesLines.size() / 2 << " internal edges." << std::endl;
    m_NavMeshDebugger->SetTwinEdges(internalEdgesLines);
}

void NavMesh::OptimizeEarClipping()
{
    if (m_HalfEdges.empty())
        return;
    MergeTriangles();

    std::map<int, int> faceIndexToNodeIndex;
    std::vector<std::vector<glm::vec3>> mergedPolygonsForDebug;
    CreatePathfindingNodes(faceIndexToNodeIndex, mergedPolygonsForDebug);
    FindNeighborsForPathfindingNodes(faceIndexToNodeIndex);
    
    if (m_NavMeshDebugger)
    {
        m_NavMeshDebugger->SetMergePolygons(mergedPolygonsForDebug);
        m_NavMeshDebugger->SetMergePolygonsFill(mergedPolygonsForDebug);
    }
}

void NavMesh::MergeTriangles()
{
    std::cout << "Starting merge " << std::endl;
    
    for (auto& face : m_HalfEdgeFaces)
        face.bIsValid = true;
    
    bool bMergeOccurred = true;
    while (bMergeOccurred)
    {
        bMergeOccurred = false;
        FindTwinHalfEdges();
        std::vector<int> currentRemovableEdges;
        NavigationUtility::FindRemovableEdgeIndexes(currentRemovableEdges, m_HalfEdges, m_HalfEdgeVertices, m_NavMeshDebugger);

        if (currentRemovableEdges.empty())
            break;

        for (int halfEdgeIndex : currentRemovableEdges)
        {
            if (halfEdgeIndex < 0 || halfEdgeIndex >= static_cast<int>(m_HalfEdges.size()))
                continue;
            
            HalfEdge& halfEdge = m_HalfEdges[halfEdgeIndex];
            int twinID = halfEdge.twinHalfEdgeIndex;

            if (twinID == -1 || halfEdge.faceID == -1 || m_HalfEdges[twinID].faceID == -1)
                continue;
            if (m_HalfEdgeFaces[halfEdge.faceID].bIsValid && m_HalfEdgeFaces[m_HalfEdges[twinID].faceID].bIsValid)
            {
                NavigationUtility::MergeTwoFacesProperley(halfEdgeIndex, m_HalfEdges, m_HalfEdgeFaces, m_HalfEdgeVertices);
                bMergeOccurred = true;
                break; 
            }
        }
    }
    std::cout << "Merge complete." << std::endl;
}

void NavMesh::CreatePathfindingNodes(std::map<int, int>& faceIndexToNodeIndex, std::vector<std::vector<glm::vec3>>& mergedPolygonsForDebug)
{
    m_PathfindingNodes.clear();

    for (int i = 0; i < static_cast<int>(m_HalfEdgeFaces.size()); ++i)
    {
        const auto& face = m_HalfEdgeFaces[i];
        if (face.bIsValid)
        {
            NavMeshOptimizedNode newNode;
            newNode.originalFaceIndex = i;

            glm::vec3 centerSum(0.0f);
            std::vector<glm::vec3> polygonVerts;
            int startEdgeID = face.halfEdgeIndex;
            int currentEdgeID = startEdgeID;
            
            do
            {
                const HalfEdge& currentEdge = m_HalfEdges[currentEdgeID];
                const glm::vec3& vertPos = m_HalfEdgeVertices[currentEdge.originVertexIndex].position;
                
                polygonVerts.push_back(vertPos);
                centerSum += vertPos;
                
                currentEdgeID = currentEdge.nextHalfEdgeIndex;
            }
            while (currentEdgeID != startEdgeID);

            newNode.polygonVerts = polygonVerts;
            if (!polygonVerts.empty())
                newNode.centerPoint = centerSum / static_cast<float>(polygonVerts.size());
            
            m_PathfindingNodes.push_back(newNode);
            faceIndexToNodeIndex[i] = static_cast<int>(m_PathfindingNodes.size() - 1);
            
            mergedPolygonsForDebug.push_back(polygonVerts);
        }
    }
    
    std::cout << "Found " << mergedPolygonsForDebug.size() << " final merged polygons." << std::endl;
}

void NavMesh::FindNeighborsForPathfindingNodes(std::map<int, int>& faceIndexToNodeIndex)
{
    for (int i = 0; i < static_cast<int>(m_PathfindingNodes.size()); ++i)
    {
        NavMeshOptimizedNode& node = m_PathfindingNodes[i];
        const auto& face = m_HalfEdgeFaces[node.originalFaceIndex];

        int startEdgeIdx = face.halfEdgeIndex;
        int currentEdgeIdx = startEdgeIdx;

        do
        {
            const HalfEdge& currentEdge = m_HalfEdges[currentEdgeIdx];
            int twinIndex = currentEdge.twinHalfEdgeIndex;

            if (twinIndex != -1)
            {
                const HalfEdge& twinEdge = m_HalfEdges[twinIndex];
                int neighborFaceOrigIndex = twinEdge.faceID;
                
                if (neighborFaceOrigIndex != -1 && m_HalfEdgeFaces[neighborFaceOrigIndex].bIsValid)
                {
                    auto nodeIndex = faceIndexToNodeIndex.find(neighborFaceOrigIndex);
                    if (nodeIndex != faceIndexToNodeIndex.end())
                    {
                        int neighborNodeIndex = nodeIndex->second;
                        int currentNodeIndex = i;
                        if (currentNodeIndex < neighborNodeIndex)
                        {
                            OptimizedEdge newCurrentEdge;
                            newCurrentEdge.neighborFaceIndex = neighborNodeIndex;
                            newCurrentEdge.edgeStart = m_HalfEdgeVertices[currentEdge.originVertexIndex].position;
                            newCurrentEdge.edgeEnd = m_HalfEdgeVertices[twinEdge.originVertexIndex].position;
                            node.neighbors.push_back(newCurrentEdge);
                            
                            OptimizedEdge newNeighborEdge;
                            newNeighborEdge.neighborFaceIndex = currentNodeIndex;
                            newNeighborEdge.edgeStart = m_HalfEdgeVertices[twinEdge.originVertexIndex].position;
                            newNeighborEdge.edgeEnd = m_HalfEdgeVertices[currentEdge.originVertexIndex].position;
                            
                            m_PathfindingNodes[neighborNodeIndex].neighbors.push_back(newNeighborEdge);
                        }
                    }
                }
            }
            
            currentEdgeIdx = currentEdge.nextHalfEdgeIndex;
        }
        while (currentEdgeIdx != startEdgeIdx);
    }

    std::cout << "Populated neighbors for all " << m_PathfindingNodes.size() << " nodes." << std::endl;
}
