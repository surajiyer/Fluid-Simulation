#include "FluidBorder.h"
#include "FluidSystem.h"

void SquareBorder::WriteBorder(int N, FluidSystem* pFs, std::vector<byte>& cellInfo)
{
	real size = 0.3;
	int low = 1 + N / 2 - size * N / 2;
	int high = 1 + N / 2 + size * N / 2;

	for (int i = low; i <= high; i++) {
		// bottom side
		pFs->DisableEdge(i, low, i, low - 1);
		// top side
		pFs->DisableEdge(i, high, i, high + 1);
		// left side
		pFs->DisableEdge(low, i, low - 1, i);
		// right side
		pFs->DisableEdge(high, i, high + 1, i);
	}
}