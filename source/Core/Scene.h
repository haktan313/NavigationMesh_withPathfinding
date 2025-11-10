#pragma once
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "Shader.h"

struct Vec3f
{
    float x, y, z;
};
struct Triangle
{
    Vec3f verts[3];
};
struct MeshData
{
    std::vector<Vec3f> vertices;
    std::vector<unsigned int> indices;

    unsigned int VAO, VBO, EBO;
};
struct SceneObject
{
    std::string name;
    MeshData* mesh;
    glm::mat4 modelMatrix;
};

class Scene
{
public:
    Scene();
    ~Scene();

    void SetupDefaultScene();
    void Render(Shader* shader);

    SceneObject* AddObject(const std::string& name, const std::string& meshName, const glm::mat4& modelMatrix = glm::mat4(1.0f));
    const std::vector<SceneObject>& GetObjects() const { return m_Objects; }
    const MeshData* GetMesh(const std::string& meshName) const;
private:
    void CreateDefaultMeshes();
    
    std::unordered_map<std::string, MeshData> m_Meshes;
    std::vector<SceneObject> m_Objects;
};
