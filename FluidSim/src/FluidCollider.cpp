#include "FluidCollider.h"

#include "FluidCode.h"
#include "FluidSystem.h"


using namespace trace;


FluidCollider::FluidCollider()
{
	vel = Vec2(40, 0);
	//inertia << 1, 1;
}


FluidCollider::~FluidCollider()
{
}

void FluidCollider::Update(int N, FluidSystem* pFs, std::vector<byte>& cellInfo, real dt)
{
	loc += vel * dt;
	rot += angVel * dt;

	for (int i = 0; i < 2; i++) {
		if (loc(i) > N + scale(i)) {
			loc(i) = -scale(i);
		}
		if (loc(i) < -scale(i)) {
			loc(i) += N+2;
		}
	}

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

	auto center = modelMatrix * Vec3(0,0,1);
	ms_center = Vec2(center(0), center(1));

	gTracePointsWS.clear();
	for (auto& ms_point : modelSpacePoints) {
		auto ws_point = modelMatrix * ms_point;
		gTracePointsWS.push_back(V2f{ ws_point(0), ws_point(1) });
	}

	normals.clear();
	for (auto& ms_point : modelSpaceNormals) {
		auto ws_norm = modelMatrix * ms_point;
		normals.push_back(Vec2{ ws_norm(0), ws_norm(1) });
	}

	for (V2f& p : gTracePointsWS) {
		if (p.x < cellOffsetX) {
			cellOffsetX = p.x;
		}
		if (p.y < cellOffsetY) {
			cellOffsetY = p.y;
		}
	}

	for (V2f& p : gTracePointsWS) {
		p.x -= cellOffsetX;
		p.y -= cellOffsetY;
	}

	size_t cell_size = 1;
	gCells = pick_cells(gTracePointsWS, cell_size);
	mass = 10 * gCells.size() / (real)(N*N);

	for (auto c : gCells) {
		int x = c.x + cellOffsetX;
		int y = c.y + cellOffsetY;
		if (x <= N && y <= N && x > 0 && y > 0) {

			int id = x + (N + 2) * y;
			cellInfo[id] &= ~CellInfo::FLUID;
			cellInfo[id] |= CellInfo::OBJECT;

			auto effective_vel = vel * 0.05;
			pFs->SetVelocity(x, y, effective_vel(0), effective_vel(1));
		}
	}

	UpdateChild(N, pFs, cellInfo, dt);
}

void FluidCollider::AddVel(int N, real& torque, Vec2& force, int x, int y, std::vector<real>& vX, std::vector<real>& vY, FluidSystem* pFs) {
	real d = 0.1 + Tools::Min(pFs->CombinedDensity(x, y), 0.9f);
	int id = x + (N + 2) * y;
	auto vel = Vec2{ vX[id], vY[id] };
	force += vel;
	auto arm = Vec2(x, y) - ms_center;
	torque += d * ( (arm(0) * vel(1) - arm(1)*vel(0)) / (arm(0)*arm(0) + arm(1)*arm(1))); //(Vec2(x,y) - ms_center) * vel;
}

void FluidCollider::ApplyForces(int N, real dt, FluidSystem* pFs)
{
	// for all cells, find neighbouring fluid cells
	auto& infos = pFs->GetInfo();

	real torque{ 0 };
	Vec2 force { 0,0 };

	auto& vX = pFs->GetVelX();
	auto& vY = pFs->GetVelY();

	for (auto c : gCells) {
		int x = c.x + cellOffsetX;
		int y = c.y + cellOffsetY;
		if (x <= N && y <= N && x > 0 && y > 0) {
			auto info = infos[x + (N + 2) * y];
			
			if (!(info & LEFT)) {
				AddVel(N, torque, force, x - 1, y, vX, vY, pFs);
			}
			if (!(info & RIGHT)) {
				AddVel(N, torque, force, x + 1, y, vX, vY, pFs);
			}
			if (!(info & TOP)) {
				AddVel(N, torque, force, x, y + 1, vX, vY, pFs);
			}
			if (!(info & BOTTOM)) {
				AddVel(N, torque, force, x, y - 1, vX, vY, pFs);
			}
		}
	}

	this->vel += dt * force / mass;
	this->vel *= powf(0.9, dt);
	this->angVel += dt * torque;
	this->angVel *= powf(0.9, dt);
}

bool FluidCollider::Contains(int i, int j)
{
	return gCells.find(V2i{ i,j }) != gCells.end();
}

void FluidCollider::AddVel(real dx, real dy)
{
	vel += Vec2{dx, dy};
}

void FluidCollider::ApplyImpulse(const Vec2& impulse, const Vec2& contactVector)
{
	vel += 1.0f / mass * impulse;
	//rot += (inertia.inverse()) * contactVector.cross(impulse);
}

void FluidCollider::Collide(int i, std::vector<FluidCollider*>& objs)
{
	for (int j = i; j < objs.size(); j++) {
		uint32_t face_idx_1, face_idx_2;
		real depth_1 = FindAxisLeastPenetration(&face_idx_1, objs[j]);
		real depth_2 = objs[j]->FindAxisLeastPenetration(&face_idx_2, this);
		real depth = Tools::Max(depth_1, depth_2);
		if (depth < 0) {
			// handle collision

		}
	}
}

Vec2 FluidCollider::GetSupport(const Vec2& dir)
{
	real bestProjection = -FLT_MAX;
	Vec2 bestVertex;

	for (int i = 0; i < modelSpacePoints.size(); ++i)
	{
		auto v = gTracePointsWS[i].toVec2();
		real projection = v.dot(dir);

		if (projection > bestProjection)
		{
			bestVertex = v;
			bestProjection = projection;
		}
	}

	return bestVertex;
}

real FluidCollider::FindAxisLeastPenetration(uint32_t *faceIndex, FluidCollider* B)
{
	real bestDistance = -FLT_MAX;
	uint32_t bestIndex;

	auto A = this;

	for (uint32_t i = 0; i < A->modelSpacePoints.size(); ++i)
	{
		// Retrieve a face normal from A
		Vec2 n = A->normals[i];

		// Retrieve support point from B along -n
		Vec2 s = B->GetSupport(-n);

		// Retrieve vertex on face from A, transform into
		// B's model space
		Vec2 v = A->gTracePointsWS[i].toVec2();

		// Compute penetration distance (in B's model space)
		real d = n.dot(s - v);// Dot(n, s - v);

		// Store greatest distance
		if (d > bestDistance)
		{
			bestDistance = d;
			bestIndex = i;
		}
	}

	*faceIndex = bestIndex;
	return bestDistance;
}
RectCollider::RectCollider(int ox, int oy, real w, real h, real r)
{
	loc = Vec2(ox, oy);
	scale = Vec2(w, h);
	rot = r;

	modelSpacePoints = { Vec3(-0.5, -0.5, 1), Vec3(-0.5, 0.5, 1), Vec3(0.5, 0.5, 1), Vec3(0.5, -0.5, 1) };
	modelSpaceNormals = { Vec3(-1, 0, 1), Vec3(0, 1, 1), Vec3(1, 0, 1), Vec3(0, -1, 1) };
}

void RectCollider::UpdateChild(int N, FluidSystem* pFs, std::vector<byte>& cellInfo, real dt)
{


}
