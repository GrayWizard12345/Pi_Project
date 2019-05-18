//
// Created by madina on 11/05/19.
//

#include "ShapeDetector.h"

#include "ShapeDetector.h"

ShapeDetector::ShapeDetector()
{
}

ShapeDetector::~ShapeDetector()
{
}

void ShapeDetector::detect(const Mat &curve)
{
    string shape = "unidentified";
    double peri = arcLength(curve, true);
    Mat approx;
    approxPolyDP(curve, approx, 0.04 * peri, true); // 0.01~0.05
    const int num_of_vertices = approx.rows;

    // if the shape is a triangle, it will have 3 vertices
    if (num_of_vertices == 3)
    {
        shape = "triangle";
    }
    else if (num_of_vertices == 4)
    {// if the shape has 4 vertices, it is either a square or a rectangle
        // Compute the bounding box of the contour and
        // use the bounding box to compute the aspect ratio
        Rect rec = boundingRect(approx);
        double ar = 1.0 * rec.width / rec.height;

        // A square will have an aspect ratio that is approximately
        // equal to one, otherwise, the shape is a rectangle
        if (ar >= 0.95 && ar <= 1.05)
        {
            shape = "square";
        }
        else
        {
            shape = "rectangle";
        }
    }
    else if (num_of_vertices == 5)
    {// if the shape is a pentagon, it will have 5 vertices
        shape = "pentagon";
    }
    else
    {// otherwise, we assume the shape is a circle
        shape = "circle";
    }
    m_shape = shape;
}

string ShapeDetector::get_shape_type()
{
    return m_shape;
}