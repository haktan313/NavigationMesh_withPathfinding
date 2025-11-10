#pragma once
#include <string>
#include <sstream>
#include <glm/glm.hpp>

class Shader
{
public:

    unsigned int ID;
    
    Shader(const char* vertexPath, const char* fragmentPath);
    
    void Use();
    
    void SetBool(const std::string &name, bool value) const;
    void SetInt(const std::string &name, int value) const;
    void SetFloat(const std::string &name, float value) const;
    void SetVec4(const std::string &name, const glm::vec4 &value) const;
    void SetMat4(const std::string &name, const glm::mat4 &mat) const;

private:
    void CheckCompileErrors(unsigned int shader, std::string type);
};