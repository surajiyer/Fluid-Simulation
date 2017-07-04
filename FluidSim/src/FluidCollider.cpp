#include "FluidCollider.h"
#include "Trace.h"
#include "FluidCode.h"
#include "FluidSystem.h"

using namespace trace;


FluidCollider::FluidCollider()
{
	vel = Vec2(40,0);
}


FluidCollider::~FluidCollider()
{
}

void FluidCollider::Update(int N, FluidSystem* pFs, std::vector<byte>& cellInfo, real dt)
{
	loc += vel * dt;
	if (loc(0) > N + scale(0)) {
		loc(0) = -scale(0);
	}
	UpdateChild(N, pFs, cellInfo, dt);
}

RectCollider::RectCollider(int ox, int oy, real w, real h, real r)
{
	loc = Vec2(ox, oy);
	scale = Vec2(w, h);
	rot = r;
}

void RectCollider::UpdateChild(int N, FluidSystem* pFs, std::vector<byte>& cellInfo, real dt)
{
	Eigen::Matrix3f rotMatrix;
	Eigen::Matrix3f locMatrix;
	Eigen::Matrix3f scMatrix;
	rotMatrix << 
		cos(rot), -sin(rot), 0, 
		sin(rot), cos(rot), 0, 
		0, 0, 1;
	scMatrix <<
		scale(0), 0, 0,
		0, scale(1), 0,
		0, 0, 1;
	locMatrix <<
		1, 0, loc(0),
		0, 1, loc(1),
		0, 0, 1;
	Eigen::Matrix3f modelMatrix = locMatrix * scMatrix * rotMatrix;

	Vec3 p1 = modelMatrix * Vec3(-0.5, -0.5, 1);
	Vec3 p2 = modelMatrix * Vec3(-0.5, 0.5, 1);
	Vec3 p3 = modelMatrix * Vec3(0.5, 0.5, 1);
	Vec3 p4 = modelMatrix * Vec3(0.5, -0.5, 1);

	std::vector<V2d> points = { { p1(0), p1(1) }, { p2(0), p2(1) }, { p3(0), p3(1) }, { p4(0), p4(1) } };

	real subX = 0;
	real subY = 0;
	for (V2d& p : points) {
		if (p.x < subX) {
			subX = p.x;
		}
		if (p.y < subY) {
			subY = p.y;
		}
	}
	for (V2d& p : points) {
		p.x -= subX;
		p.y -= subY;
	}

	size_t cell_size = 1;
	auto cells = pick_cells(points, cell_size);

	for (auto c : cells) {
		int x = c.x + subX;
		int y = c.y + subY;
		if (x <= N && y <= N && x > 0 && y > 0) {
			cellInfo[x + (N + 2) * y] &= ~CellInfo::FLUID;
			pFs->SetVelocity(x, y, vel(0), vel(1));
		}
	}
}
