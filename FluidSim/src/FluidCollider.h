#pragma once
#include "Inc.h"
#include "Trace.h"

class FluidCollider
{
public:
	FluidCollider();
	~FluidCollider();

	
	void Update(int N, FluidSystem*, std::vector<byte>& cellInfo, real dt);
	virtual void UpdateChild(int N, FluidSystem*, std::vector<byte>& cellInfo, real dt) = 0;
	void ApplyForces(int N, real dt, FluidSystem*);
	bool Contains(int i, int j);
	void AddVel(real dx, real dy);
	void Collide(int i, std::vector<FluidCollider*>& obj_rest);

protected:
	Vec2 vel;
	real angVel;

	Vec2 loc;
	Vec2 scale;
	real rot;

	real mass = 0;

	Eigen::Matrix3f rotMatrix;
	Eigen::Matrix3f locMatrix;
	Eigen::Matrix3f scMatrix;

	std::vector<Vec3> modelSpacePoints;
	std::vector<Vec3> modelSpaceNormals;
	std::vector<Vec2> normals;
	Vec2 ms_center;
	std::vector<trace::V2f> gTracePointsWS;
	std::set<trace::V2i> gCells;

	int cellOffsetX = 0;
	int cellOffsetY = 0;


	void ApplyImpulse(const Vec2& impulse, const Vec2& contactVector);
	Vec2 GetSupport(const Vec2& dir);
	real FindAxisLeastPenetration(uint32_t *faceIndex, FluidCollider* B);

	void AddVel(int N, real& torque, Vec2& force, int x, int y, std::vector<real>& vX, std::vector<real>& vY, FluidSystem* pFs);
};

class RectCollider
	: public FluidCollider
{
public:
	RectCollider(int ox, int oy, real w, real h, real r);

	virtual void UpdateChild(int N, FluidSystem*, std::vector<byte>& cellInfo, real dt) override;
};