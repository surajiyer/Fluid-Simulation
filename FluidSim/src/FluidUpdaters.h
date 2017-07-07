#pragma once

#include "Inc.h"
#include "FluidSystem.h"

namespace FluidUpdaters {

class Blower
	: public Tools::UpdatableR<bool, FluidUpdate>
{
public:
	Blower(real x, real y, real dx, real dy, real sc, real v, real d, int fid);
	virtual bool Update(FluidUpdate) override;

	real px, py, dx, dy, scale, vel, dens;
	int fid;
};

class Gravity
	: public Tools::UpdatableR<bool, FluidUpdate>
{
public:
	virtual bool Update(FluidUpdate) override;
};


class Square 
	: public Tools::UpdatableR<bool, FluidUpdate>
{
public:
	virtual bool Update(FluidUpdate) override;
private:
	int id = 0;
};


//void HalfFull(FluidSystem*);

}