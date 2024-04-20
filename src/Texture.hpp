//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        float s = u_img-(int)u_img + 0.5;
        float t = v_img-(int)v_img + 0.5;

        if (s > 1) {
            s = s-1;
        } else {
            u_img = u_img-1;
        }
        if (t > 1) {
            t = t-1;
        } else {
            v_img = v_img-1;
        }

        auto u00 = image_data.at<cv::Vec3b>(v_img, u_img);
        auto u10 = image_data.at<cv::Vec3b>(v_img, u_img + 1);
        auto u01 = image_data.at<cv::Vec3b>(v_img + 1, u_img);
        auto u11 = image_data.at<cv::Vec3b>(v_img + 1, u_img + 1);

        auto u0 = u00 + s * (u10 - u00);
        auto u1 = u01 + t * (u11 - u01);

        auto color = u0 + t * (u1 - u0);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
