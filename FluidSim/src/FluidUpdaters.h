#pragma once

#include "Inc.h"
#include "FluidSystem.h"

namespace FluidUpdaters {

class Blower
	: public Tools::UpdatableR<bool, FluidUpdate>
{
public:
	Blower(int dn);
	virtual bool Update(FluidUpdate) override;

	int dn;
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