#pragma once
#ifndef VOLUME_STRUCTS_H
#define VOLUME_STRUCTS_H

struct float3 {										// A three dimensional float
													// Variables for each dimension
	float x;										// x dimension
	float y;										// y dimension
	float z;										// z dimension								
	
	float3() {										// Generic constructor
		x = 0.f;
		y = 0.f;
		z = 0.f;
	}

	float3(float _x, float _y, float _z) {			// Constructor with given values
		x = _x;
		y = _y;
		z = _z;
	}

	float3 operator + (const float3 &b) const {		// Addition operator
		return float3(x + b.x, y + b.y, z + b.z);
	}

	float3 operator - (const float3 &b) const {		// Subtraction operator
		return float3(x - b.x, y - b.y, z - b.z);
	}
};

struct Vertex										// Vertex used for creating VBO (Includes Normals)
{
    float3 m_Pos;									// Position of vertex
    float3 m_Color;									// Color of vertex
	float3 m_Normal;								// Normal of vertex

	Vertex() {}										// Generic constructor

	Vertex(float3 pos, float3 norm) {				// Constructor with given values
		m_Pos = pos;								// Set the position
		m_Color = pos;								// Set color to be the same as position
		m_Normal = norm;							// Set normal
	}
};

struct VertexPositionColor							// Vertex used for creating VBO (No Normals)
{
	float3 m_Pos;									// Position of vertex
    float3 m_Color;									// Color of vertex

	VertexPositionColor() {}							// Generic constructor

	VertexPositionColor(float3 pos, float3 color) {	// Constructor with given values
		m_Pos = pos;								// Set the position
		m_Color = color;							// Set color to be the same as position
	}
};

#endif
