#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv; // 与光线相交的点在三角形中的重心坐标 
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};