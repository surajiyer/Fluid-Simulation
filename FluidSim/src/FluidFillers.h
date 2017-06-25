#pragma once

#include "Inc.h"

namespace FluidFillers {


class Square 
	: public Tools::UpdatableR<bool, FluidSystem*>
{
public:
	virtual bool Update(FluidSystem*) override;
private:
	int id = 0;
};

class Blower
	: public Tools::UpdatableR<bool, FluidSystem*>
{
public:
	virtual bool Update(FluidSystem*) override;
private:
	int id = 0;
};


//void HalfFull(FluidSystem*);

}