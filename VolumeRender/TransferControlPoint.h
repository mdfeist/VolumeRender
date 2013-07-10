#pragma once

#include "Eigen\Geometry.h"

class TransferControlPoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector4f Color;
    int IsoValue;

    /// <summary>
    /// Constructor for color control points.
    /// Takes rgb color components that specify the color at the supplied isovalue.
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <param name="z"></param>
    /// <param name="isovalue"></param>
    TransferControlPoint(float r, float g, float b, int isovalue)
    {
        Color.x() = r;
        Color.y() = g;
        Color.z() = b;
        Color.w() = 1.0f;
        this->IsoValue = isovalue;
    }

    /// <summary>
    /// Constructor for alpha control points.
    /// Takes an alpha that specifies the alpha at the supplied isovalue.
    /// </summary>
    /// <param name="alpha"></param>
    /// <param name="isovalue"></param>
    TransferControlPoint(float alpha, int isovalue)
    {
        Color.x() = 0.0f;
        Color.y() = 0.0f;
        Color.z() = 0.0f;
        Color.w() = alpha;
        this->IsoValue = isovalue;
    }
};
