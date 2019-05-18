//
// Created by madina on 11/05/19.
//
#pragma once
#ifndef SHAPEDETECTOR_H_
#define SHAPEDETECTOR_H_

#include <iostream>
#include <vector>
#include <string>

using namespace std;

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;

class ShapeDetector
{
public:
    ShapeDetector();
    ~ShapeDetector();
    void detect(const Mat &curve);
    string get_shape_type();

private:
    string m_shape;
};

#endif