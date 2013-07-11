#pragma once

/// <summary>
/// Cubic class that calculates the cubic spline from a set of control points/knots
/// and performs cubic interpolation.
/// 
/// Based on the natural cubic spline code from: http://www.cse.unsw.edu.au/~lambert/splines/natcubic.html
/// </summary>
class Cubic
{
private:
	float a, b, c, d; // a + b*s + c*s^2 +d*s^3 

public:
	Cubic() {}
	Cubic(float a, float b, float c, float d)
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
	}

	float A() { return a; }
	float B() { return b; }
	float C() { return c; }
	float D() { return d; }

	//evaluate the point using a cubic equation
	float GetPointOnSpline(float s)
	{
		return (((d * s) + c) * s + b) * s + a;
	}

	static Cubic* calcNaturalCubic(int n, float* x) {
		float* gamma = new float[n + 1];
		gamma[0] = 1.0 / 2.0;
		for (int i = 1; i < n; ++i)
			gamma[i] = 1 / (4 - gamma[i - 1]);
		gamma[n] = 1 / (2 - gamma[n - 1]);

        float* delta = new float[n + 1];
        delta[0] = 3 * (x[1] - x[0]) * gamma[0];
        for (int i = 1; i < n; ++i)
            delta[i] = (3 * (x[i + 1] - x[i - 1]) - delta[i - 1]) * gamma[i];
        delta[n] = (3 * (x[n] - x[n - 1])-delta[n - 1]) * gamma[n];

        float* D = new float[n + 1];
        D[n] = delta[n];
        for (int i = n - 1; i >= 0; --i) {
            D[i] = delta[i] - gamma[i] * D[i + 1];
        }

        // Calculate the cubic segments.
        Cubic* C = new Cubic[n];
        for (int i = 0; i < n; i++) {
            float a = x[i];
            float b = D[i];
            float c = 3 * (x[i + 1] - x[i]) - 2 * D[i] - D[i + 1];
            float d = 2 * (x[i] - x[i + 1]) + D[i] + D[i + 1];
            C[i] = Cubic(a, b, c, d);
        }

        return C;
    }
};
