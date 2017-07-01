#pragma once
#include "Inc.h"

class FluidBorder {
public:
	virtual void WriteBorder(int N, FluidSystem*, std::vector<byte>& cellInfo) = 0;
};

class SquareBorder
	: public FluidBorder
{
public:
	virtual void WriteBorder(int N, FluidSystem*, std::vector<byte>& cellInfo) override;

};