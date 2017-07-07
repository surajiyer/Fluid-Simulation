#include "FluidCollider.h"

#include "FluidCode.h"
#include "FluidSystem.h"


using namespace trace;


FluidCollider::FluidCollider()
{
	
}


FluidCollider::~FluidCollider()
{
}

void FluidCollider::Update(int N, FluidSystem* pFs, std::vector<byte>& cellInfo, real dt)
{
	loc += vel * dt;
	rot += angVel * dt;

	real bounce_force = 50;

	for (int i = 0; i < 2; i++) {
		if (loc(i) > N + 2) {
			vel(i) -= (loc(i) - N-2) * bounce_force * dt;
		}
		if (loc(i) < 0) {
			vel(i) += (-loc(i)) * bounce_force * dt;
		}
	}

	gRotMatrix <<
		cos(rot), -sin(rot), 0,
		sin(rot), cos(rot), 0,
		0, 0, 1;
	gScaleMatrix <<
		scale(0), 0, 0,
		0, scale(1), 0,
		0, 0, 1;
	gLocMatrix <<
		1, 0, loc(0),
		0, 1, loc(1),
		0, 0, 1;
	gModelMatrix = gLocMatrix * gScaleMatrix * gRotMatrix;

	auto center = gModelMatrix * Vec3(0,0,1);
	w_center = Vec2(center(0), center(1));

	w_vertices.clear();
	gTracePointsWS.clear();
	for (auto& n : m_vertices) {
		auto v = gModelMatrix * n;
		w_vertices.push_back(Vec2{v(0), v(1)});
		gTracePointsWS.push_back(V2f{ v(0), v(1) });
	}

	w_normals.clear();
	for (auto& n : m_normals) {
		auto v = gModelMatrix * n;
		auto tmp = Vec2{ v(0), v(1) };
		w_normals.push_back(tmp.normalized());
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
	real d = 0.2f + Tools::Min(pFs->CombinedDensity(x, y), 0.8f);
	int id = x + (N + 2) * y;
	auto vel = Vec2{ vX[id], vY[id] };
	force += vel;
	auto arm = Vec2(x, y) - w_center;
	real div = arm(0)*arm(0) + arm(1)*arm(1);
	if (div > 0.00001) {
		torque += d * ((arm(0) * vel(1) - arm(1)*vel(0)) / div); //(Vec2(x,y) - ms_center) * vel;
	}
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

	// damp
	this->vel += dt * force / mass;
	this->vel *= powf(0.9, dt);
	this->angVel += dt * torque / momentOfInertia;
	this->angVel *= powf(0.9, dt);

	// drag
	real v2 = vel.norm(); v2 *= v2;
	vel /= (1 + 0.00001 * v2);
	real av2 = angVel * angVel;
	angVel /= (1 + 0.001 * av2);
}

bool FluidCollider::Contains(int i, int j)
{
	auto val = V2i(i - cellOffsetX, j - cellOffsetY);
	for (auto entry : gCells) {
		if (val == entry) {
			return true;
		}
	}
	return false;
	//gCells.find(V2i{ i,j }) != gCells.end();
}

void FluidCollider::AddVel(real dx, real dy)
{
	vel += Vec2{dx, dy};
}

void FluidCollider::Collide(FluidCollider* A, FluidCollider* B)
{
	uint32_t face_idx_1, face_idx_2;
	real distance1 = A->FindAxisLeastPenetration(&face_idx_1, B);
	real distance2 = B->FindAxisLeastPenetration(&face_idx_2, A);

	Contact cp;
	cp.penetration = Tools::Max(distance1, distance2);
	cp.normal = distance1 > distance2 ? A->w_normals[face_idx_1] : B->w_normals[face_idx_2];

	//std::cout << "\ndist1: " << distance1 << " dist2: " << distance2 << "\n";
	if (cp.penetration < 0) {
		// collision response
		FluidCollider::ApplyImpulse(A, B, &cp);
	}
}

void FluidCollider::ApplyImpulse(FluidCollider* A, FluidCollider* B, const Contact* cp)
{
	Vec2 relVel = A->vel - B->vel;
	real contactVel = relVel.dot(cp->normal);

	// Do not resolve if velocities are separating
	if (contactVel > 0 || A->mass < 0.000001 || B->mass < 0.000001)
		return;

	// Calculate restitution
	real e = std::min(A->coeff_restitution, B->coeff_restitution);

	// Calculate impulse scalar
	real j = -(1.0f + e) * contactVel;
	j /= 1 / A->mass + 1 / B->mass + pow(1, 2) / A->momentOfInertia + pow(1, 2) / B->momentOfInertia;

	// Apply impulse
	Vec2 impulse = j * cp->normal;
	Vec2 contactPoint = A->w_vertices[0] - A->w_center; // TODO: Need to find correct contact point
	Vec2 aContactVec = A->w_center - contactPoint;
	Vec2 bContactVec = B->w_center - contactPoint;
	A->vel += 1 / A->mass * impulse;
	B->vel -= 1 / B->mass * impulse;
	A->angVel += std::min(1 / A->momentOfInertia * (aContactVec(0)*impulse(1) - aContactVec(1)*impulse(0)), 0.1f);
	B->angVel -= std::min(1 / B->momentOfInertia * (bContactVec(0)*impulse(1) - bContactVec(1)*impulse(0)), 0.1f);
}

real FluidCollider::FindAxisLeastPenetration(uint32_t *faceIndex, FluidCollider* other)
{
	real bestDistance = -FLT_MAX;
	uint32_t bestIndex = 0;

	uint32_t size = w_vertices.size();
	for (uint32_t i = 0; i < size; ++i)
	{
		// Retrieve a face normal from A
		Vec2 n = this->w_normals[i];

		// Retrieve support point from B along -n
		Vec2 s = other->GetSupport(-n);

		// Retrieve vertex on face from A, transform into
		// B's model space
		Vec2 v = this->w_vertices[i];

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

Vec2 FluidCollider::GetSupport(const Vec2& dir)
{
	real bestProjection = -FLT_MAX;
	Vec2 bestVertex;

	for (int i = 0; i < w_vertices.size(); ++i)
	{
		auto v = w_vertices[i];
		real projection = v.dot(dir);

		if (projection > bestProjection)
		{
			bestVertex = v;
			bestProjection = projection;
		}
	}

	return bestVertex;
}


RectCollider::RectCollider(int ox, int oy, real w, real h, real r)
{
 	loc = Vec2(ox, oy);
	scale = Vec2(w, h);
	rot = r;

	m_vertices = { Vec3(-0.5, -0.5, 1), Vec3(-0.5, 0.5, 1), Vec3(0.5, 0.5, 1), Vec3(0.5, -0.5, 1) };
	m_normals = { Vec3(-1, 0, 0), Vec3(0, 1, 0), Vec3(1, 0, 0), Vec3(0, -1, 0) };
}

void RectCollider::UpdateChild(int N, FluidSystem* pFs, std::vector<byte>& cellInfo, real dt)
{


}
