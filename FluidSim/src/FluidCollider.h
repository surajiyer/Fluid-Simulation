#pragma once
#include "Inc.h"
#include "Trace.h"

// Represents a single point of contact during a collision
struct Contact
{
	Vec2 position;
	Vec2 normal;
	real penetration;
};

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
	static void Collide(FluidCollider* A, FluidCollider* B);
	inline Vec2 GetLoc() { return loc; }

protected:
	Vec2 vel = {0,0};
	real angVel = 0;

	Vec2 loc;
	Vec2 scale;
	real rot = 0;
	
	real coeff_restitution = 1;
	real mass = 0.1;
	real momentOfInertia = 10.0;

	Eigen::Matrix3f gRotMatrix;
	Eigen::Matrix3f gLocMatrix;
	Eigen::Matrix3f gScaleMatrix;
	Eigen::Matrix3f gModelMatrix;

	Vec2 w_center;
	std::vector<Vec3> m_vertices;
	std::vector<Vec3> m_normals;
	std::vector<Vec2> w_vertices;
	std::vector<Vec2> w_normals;
	std::vector<trace::V2f> gTracePointsWS;
	std::set<trace::V2i> gCells;

	int cellOffsetX = 0;
	int cellOffsetY = 0;


	static void ApplyImpulse(FluidCollider* A, FluidCollider* B, const Contact* cp);
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