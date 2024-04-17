// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v);
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, _v);
    return alpha >= 0 && beta >= 0 && gamma >= 0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    float min_x = width-1;
    float min_y = height-1;
    float max_x = 0;
    float max_y = 0;

    for (int i = 0; i <= 2; i++) {
        min_x = std::min(min_x, floor(v[i].x()));
        min_y = std::min(min_y, floor(v[i].y()));

        max_x = std::max(max_x, ceil(v[i].x()));
        max_y = std::max(max_y, ceil(v[i].y()));
    }

    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= width) max_x = width-1;
    if (max_y >= height) max_y = height-1;

    const int sample_count = msaa*msaa;

    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            int pixel_index = get_index(x, y);
            int count = 0;
            for (int i = 0; i < sample_count; i++) {
                float samplePointWidth = 1.0/msaa;
                int col = i%msaa;
                int row = i/msaa;
                float sampleX = x + col*samplePointWidth + samplePointWidth/2;
                float sampleY = y + row*samplePointWidth + samplePointWidth/2;

                if (!insideTriangle(sampleX, sampleY, t.v)) continue;

                auto[alpha, beta, gamma] = computeBarycentric2D(sampleY, sampleY, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if (isinf(sample_depth_buf[pixel_index][i])  || z_interpolated > sample_depth_buf[pixel_index][i]) {
                    sample_depth_buf[pixel_index][i] = z_interpolated;
                    sample_color_buf[pixel_index][i] = t.getColor();
                    count++;
                }
            }
            if (count) {
                Vector3f res = {0.0, 0.0, 0.0};
                for (auto&& color : sample_color_buf[pixel_index]) {
                    res += color;
                }
                set_pixel({x, y, 0}, res/(sample_count));
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        for (auto&& cb : sample_color_buf) {
            std::fill(cb.begin(), cb.end(), Eigen::Vector3f{0, 0, 0});
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        for (auto&& db : sample_depth_buf) {
            std::fill(db.begin(), db.end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h), msaa(2)
{
    frame_buf.resize(w * h);
    sample_depth_buf.resize(w * h);
    for (auto&& db : sample_depth_buf) {
        db.resize(msaa*msaa);
    }
    sample_color_buf.resize(w * h);
    for (auto&& cb : sample_color_buf) {
        cb.resize(msaa*msaa);
    }
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on