#include "Inc.h"
#include "SystemManager.h"

#include <OpenGLBase/Include.h>
#include <OpenGLBase/src/GLCore/GraphicsIncludes.h>
#include <OpenGLBase/src/Example/GLExampleObject.h>
#include <OpenGLBase/src/CGMain/MeshGen.h>
#include <OpenGLBase/src/GLBase/Environment/Scene.h>

#include "Trace.h"

#include <iostream>
#include <stdio.h>
#include <string>

using namespace trace;

int N = 16;

int ID(int x, int y) {
	return x + (N+2) * y;
}

void SetBorder(int N, int b, float * x)
{
	int i;

	// sides
	for (i = 1; i <= N; i++) {
		x[ID(0, i)] = b == 1 ? -x[ID(1, i)] : x[ID(1, i)];
		x[ID(N + 1, i)] = b == 1 ? -x[ID(N, i)] : x[ID(N, i)];
		x[ID(i, 0)] = b == 2 ? -x[ID(i, 1)] : x[ID(i, 1)];
		x[ID(i, N + 1)] = b == 2 ? -x[ID(i, N)] : x[ID(i, N)];
	}

	// corners
	x[ID(0, 0)] = 0.5f*(x[ID(1, 0)] + x[ID(0, 1)]);
	x[ID(0, N + 1)] = 0.5f*(x[ID(1, N + 1)] + x[ID(0, N)]);
	x[ID(N + 1, 0)] = 0.5f*(x[ID(N, 0)] + x[ID(N + 1, 1)]);
	x[ID(N + 1, N + 1)] = 0.5f*(x[ID(N, N + 1)] + x[ID(N + 1, N)]);
}

void LinearSolve(int N, int b, float * x, float * x0, float a, float c)
{
	int i, j, k;

	for (k = 0; k < 6; k++) {
		for (i = 1; i <= N; i++) {
			for (j = 1; j <= N; j++) {
				x[ID(i, j)] = (x0[ID(i, j)] + a*(x[ID(i - 1, j)] + x[ID(i + 1, j)] + x[ID(i, j - 1)] + x[ID(i, j + 1)])) / c;
			}
		}
		SetBorder(N, b, x);
	}
}

void Test() {

	int b = 0;
	real dt = 0.1;
	std::vector<float> x;
	std::vector<float> x0;

	int s = (N + 2); s *= s;
	x.resize(s);
	x0.resize(s);

	x0[ID(4, 4)] = 1;
	
	float diff = 0.000;
	float a = dt*diff*N*N;

	LinearSolve(N, b, x.data(), x0.data(), a, 1 + 4 * a);

	for (int i = 0; i < N + 2; i++) {
		for (int j = 0; j < N + 2; j++) {
			std::cout.precision(2);
			std::cout << std::setw(7) << x[ID(i,j)] << ", ";
		}
		std::cout << "\n";
	}
	std::cout << "\n";
}

void main() {
	std::cout << "Press SPACE to start/stop the simulation, press \"t\" to do a single timestep.";
	SystemManager sm;
	sm.Run();
}