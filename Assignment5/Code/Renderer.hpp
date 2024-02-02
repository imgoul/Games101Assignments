#pragma once
#include "Scene.hpp"


// 没有直接定义交点的位置信息，通过其他信息可以推出交点位置信息。
// hit_obj、index、uv信息    =>   交点位置信息   
struct hit_payload
{
    float tNear;    // 与光线的距离
    uint32_t index; // 与光线相交的交点所在的三角的索引
    Vector2f uv; // 与光线相交的点在三角形中的重心坐标 
    Object* hit_obj; // 与光线相交的物体


    
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};