#pragma once
#include <glm/vec3.hpp>

#include "StructsForNavigationSystem.h"
#include "Core/Camera.h"
#include "Core/Scene.h"

class NavMesh;

class NavMeshDebugger
{
public:
    bool bNavMeshBuilt = false;
    NavMeshDebugger();
    ~NavMeshDebugger();
    void CleanBuffers();

    void RenderDebugTool(Shader* shader, Camera& camera, const Scene& scene, const DrawDebugInfo& debugInfo, NavBuildParams buildParams, const
                         HeightField& heightField);
    void RenderNavMeshBound(Shader* shader, const Scene& scene, const NavBuildParams& buildParams);
    void RenderGrids(Shader* shader, const Scene& scene, const NavBuildParams& buildParams);
    void RenderWalkableSurfaces(Shader* shader, const Scene& scene, const NavBuildParams& buildParams, const HeightField& heightField);
    void RenderWalkableRegions(Shader* shader, const Scene& scene, const NavBuildParams& buildParams, const HeightField& heightField);
    
    void SetBorderVertices(const std::vector<glm::vec3>& borderVerts);
    void RenderBorder(Shader* shader);
    void SetHoles(const std::vector<std::vector<glm::vec3>>& holeVerts);
    void RenderHoles(Shader* shader);
    void FillSpaces(Shader* shader);

    void SetEarClipping(std::vector<glm::vec3> earTriangleVerts);
    void RenderEarClipping(Shader* shader);
    void SetTrianglesFill(const std::vector<NavMeshTriangle>& triangles);
    void RenderTrianglesFill(Shader* shader);

    void SetMergePolygons(const std::vector<std::vector<glm::vec3>>& mergedPolygonVerts);
    void RenderMergePolygons(Shader* shader);
    void SetMergePolygonsFill(const std::vector<std::vector<glm::vec3>>& mergedPolygonVerts);
    void RenderMergePolygonsFill(Shader* shader);
    
    void SetTwinEdges(const std::vector<glm::vec3>& twinEdgeVerts);
    void RenderTwinEdges(Shader* shader);
    void SetRemovableEdges(const std::vector<glm::vec3>& removableEdgeVerts);
    void RenderRemovableEdges(Shader* shader);
    void SetCannotRemoveEdges(const std::vector<glm::vec3>& cannotRemoveEdgeVerts);
    void RenderCannotRemoveEdges(Shader* shader);

    void SetStartEndMarkers(const glm::vec3& start, const glm::vec3& end);
    void RenderStartEndMarkers(Shader* shader);

    void SetPath(const std::vector<glm::vec3>& nodes);
    void RenderPath(Shader* shader);
    void SetSmoothedPath(const std::vector<glm::vec3>& pathPoints);
    void RenderSmoothedPath(Shader* shader);
private:
    GLsizei m_BorderCount;
    std::vector<GLsizei> m_HoleCounts;
    GLsizei m_TriangleCount;
    GLsizei m_EarClipCount;
    std::vector<GLsizei> m_MergedPolygonCounts;
    GLsizei m_MergedPolygonFillCount;

    GLsizei m_TwinEdgeCount;
    GLsizei m_RemovableEdgeCount;
    GLsizei m_CannotRemoveEdgeCount;

    GLsizei m_StartEndMarkerCount;
    GLsizei m_PathPointCount;
    GLsizei m_SmoothedPathPointCount;
    
    unsigned int m_BorderVAO, m_BorderVBO;
    std::vector<unsigned int> m_HoleVAOs;
    unsigned int m_TriangleVAO, m_TriangleVBO;
    unsigned int m_EarClipingVAO, m_EarClippingVBO;
    std::vector<unsigned int> m_MergedPolygonVAOs;
    unsigned int m_MergedPolygonFillVAO, m_MergedPolygonFillVBO;
    
    unsigned int m_TwinEdgeVAO, m_TwinEdgeVBO;
    unsigned int m_RemovableEdgeVAO, m_RemovableEdgeVBO;
    unsigned int m_CannotRemoveEdgeVAO, m_CannotRemoveEdgeVBO;

    unsigned int m_StartEndMarkerVAO, m_StartEndMarkerVBO;
    unsigned int m_PathPointVAO, m_PathPointVBO;
    unsigned int m_SmoothedPathPointVAO, m_SmoothedPathPointVBO;
};
