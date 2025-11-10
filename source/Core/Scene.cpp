#include "Scene.h"

#include <iostream>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>

Scene::Scene() : m_Meshes(), m_Objects()
{
    CreateDefaultMeshes();
}

Scene::~Scene()
{
    for (auto& pair : m_Meshes)
    {
        glDeleteVertexArrays(1, &pair.second.VAO);
        glDeleteBuffers(1, &pair.second.VBO);
        glDeleteBuffers(1, &pair.second.EBO);
    }
}

void Scene::SetupDefaultScene()
{
    m_Objects.clear();

    // Ground
    glm::mat4 groundTransform = glm::mat4(1.0f);
    groundTransform = glm::translate(groundTransform, glm::vec3(0.0f, -0.05f, 0.0f));
    groundTransform = glm::scale(groundTransform, glm::vec3(30.0f, 0.1f, 30.0f));
    AddObject("Ground", "Cube", groundTransform);
    /*// Ground2
    glm::mat4 groundTransform2 = glm::mat4(1.0f);
    groundTransform2 = glm::translate(groundTransform2, glm::vec3(0.0f, 5.f, 0.0f));
    groundTransform2 = glm::scale(groundTransform2, glm::vec3(30.0f, 0.1f, 30.0f));
    AddObject("Ground2", "Cube", groundTransform2);*/

    // Box
    glm::mat4 obstacleTransform = glm::mat4(1.0f);
    obstacleTransform = glm::translate(obstacleTransform, glm::vec3(-6.f, 2.0f, -4.0f));
    obstacleTransform = glm::scale(obstacleTransform, glm::vec3(4.0f, 4.0f, 4.0f));
    AddObject("Obstacle", "Cube", obstacleTransform);

    // Another Box
    glm::mat4 obstacleTransform2 = glm::mat4(1.0f);
    obstacleTransform2 = glm::translate(obstacleTransform2, glm::vec3(5.0f, 1.0f, 7.0f));
    obstacleTransform2 = glm::scale(obstacleTransform2, glm::vec3(2.0f, 2.0f, 2.0f));
    AddObject("Obstacle2", "Cube", obstacleTransform2);

    // Another Box
    glm::mat4 obstacleTransform3 = glm::mat4(1.0f);
    obstacleTransform3 = glm::translate(obstacleTransform3, glm::vec3(10.0f, 1.0f, 13.0f));
    obstacleTransform3 = glm::scale(obstacleTransform3, glm::vec3(2.0f, 2.0f, 2.0f));
    AddObject("Obstacle3", "Cube", obstacleTransform3);

    // Another Box
    glm::mat4 obstacleTransform5 = glm::mat4(1.0f);
    obstacleTransform5 = glm::translate(obstacleTransform5, glm::vec3(-2.0f, 0.0f, 2.0f));
    obstacleTransform5 = glm::scale(obstacleTransform5, glm::vec3(7.0f, 2.0f, 2.0f));
    AddObject("Obstacle5", "Cube", obstacleTransform5);

    // Another Box
    glm::mat4 obstacleTransform4 = glm::mat4(1.0f);
    obstacleTransform4 = glm::translate(obstacleTransform4, glm::vec3(-10.0f, 0.0f, 10.0f));
    obstacleTransform4 = glm::scale(obstacleTransform4, glm::vec3(7.0f, 2.0f, 2.0f));
    AddObject("Obstacle4", "Cube", obstacleTransform4);

    // Another Box
    glm::mat4 obstacleTransform6 = glm::mat4(1.0f);
    obstacleTransform6 = glm::translate(obstacleTransform6, glm::vec3(12.0f, 0.0f, -4.0f));
    obstacleTransform6 = glm::scale(obstacleTransform6, glm::vec3(2.0f, 2.0f, 15.0f));
    AddObject("Obstacle6", "Cube", obstacleTransform6);

    // Another Box
    glm::mat4 obstacleTransform7 = glm::mat4(1.0f);
    obstacleTransform7 = glm::translate(obstacleTransform7, glm::vec3(5.0f, 0.0f, -8.0f));
    obstacleTransform7 = glm::scale(obstacleTransform7, glm::vec3(7.0f, 2.0f, 10.0f));
    AddObject("Obstacle7", "Cube", obstacleTransform7);

}

void Scene::Render(Shader* shader)
{
    if (!shader || m_Objects.empty())
        return;

    shader->Use();
    
    for (const auto& object : m_Objects)
    {
        if (!object.mesh)
            continue;
        
        shader->SetMat4("model", object.modelMatrix);
        
        if (object.name == "Ground")
            shader->SetVec4("ourColor", glm::vec4(0.4f, 0.4f, 0.4f, 1.0f));
        else
            shader->SetVec4("ourColor", glm::vec4(0.8f, 0.2f, 0.2f, 1.0f));
        
        glBindVertexArray(object.mesh->VAO);
        glDrawElements(GL_TRIANGLES, object.mesh->indices.size(), GL_UNSIGNED_INT, 0);
    }
    glBindVertexArray(0);
}

SceneObject* Scene::AddObject(const std::string& name, const std::string& meshName, const glm::mat4& modelMatrix)
{
    if (m_Meshes.find(meshName) == m_Meshes.end())
    {
        std::cout << "Mesh " << meshName << " not found!" << std::endl;
        return nullptr;
    }
    SceneObject newObj;
    newObj.name = name;
    newObj.mesh = &m_Meshes.at(meshName);
    newObj.modelMatrix = modelMatrix;
    m_Objects.push_back(newObj);
    return &m_Objects.back();
}

const MeshData* Scene::GetMesh(const std::string& meshName) const
{
    auto it = m_Meshes.find(meshName);
    if (it != m_Meshes.end())
        return &it->second;
    return nullptr;
}

void Scene::CreateDefaultMeshes()
{
    MeshData cubeMesh;
    cubeMesh.vertices = {
        {-0.5f, -0.5f, -0.5f}, // 0
        { 0.5f, -0.5f, -0.5f}, // 1
        { 0.5f,  0.5f, -0.5f}, // 2
        {-0.5f,  0.5f, -0.5f}, // 3
        {-0.5f, -0.5f,  0.5f}, // 4
        { 0.5f, -0.5f,  0.5f}, // 5
        { 0.5f,  0.5f,  0.5f}, // 6
        {-0.5f,  0.5f,  0.5f}  // 7
    };
    cubeMesh.indices = {
        0, 1, 2,   0, 2, 3, // Front
        4, 7, 6,   4, 6, 5, // Back
        0, 3, 7,   0, 7, 4, // Left
        1, 5, 6,   1, 6, 2, // Right
        3, 2, 6,   3, 6, 7, // Up
        0, 4, 5,   0, 5, 1  // Down
    };
    m_Meshes["Cube"] = cubeMesh;

    for (auto& [key, mesh] : m_Meshes)
    {
        glGenVertexArrays(1, &mesh.VAO);
        glGenBuffers(1, &mesh.VBO);
        glGenBuffers(1, &mesh.EBO);

        glBindVertexArray(mesh.VAO);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
        glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(Vec3f), mesh.vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW);
        
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3f), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }
}
