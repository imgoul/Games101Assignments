//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"


struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex) :
         color(col), normal(nor), tex_coords(tc), texture(tex) {}


    Eigen::Vector3f view_pos;//顶点摄像机坐标系下的位置
    Eigen::Vector3f color;//顶点颜色
    Eigen::Vector3f normal;//该点的法线向量
    Eigen::Vector2f tex_coords;//顶点对应的uv坐标
    Texture* texture;//顶点引用的贴图
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;
};

#endif //RASTERIZER_SHADER_H
