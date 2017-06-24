#include "SystemManager.h"
#include "FluidSystem.h"
#include "FluidRenderer.h"
#include "FluidFillers.h"

#include <Tools/Include.h>
#include <WindowWin32/InputManager_get.h>

SystemManager::SystemManager()
	: gEnableRender(true)
{
	Setup();
}

SystemManager::~SystemManager()
{
	Tools::SafeDeletePtr(gInputListener);
}

void SystemManager::Run()
{
	Tools::SetHighPriority();
	auto t = Tools::TimeHelp::GetTime();
	real base_dt = 0.0015f;
	real dt = base_dt;

	while
		// (gpEngine ? gpEngine->Run_step() : true) {
		(gpEngine ? gpEngine->IsRunning() : true) {

		ReadInput();

		if (gInputListener->IsKeyDown(System::InputManager::KeyCodes::Esc)) {
			std::cout << "System aborted.\n";
			break;
		}

		if (gpFluidSystem == nullptr) {
			Tools::Sleep(5);
			continue;
		}

		// update
		if (gSimulationEnabled || gInputListener->IsKeyPressed('T')) {
			// sdt is the time step we send to the system
			float sdt = gTimeAnalyse.AvgDt(); // dt gTimeAnalise.AvgDt();
			if (sdt < base_dt) sdt = base_dt;

			Update(sdt);

			// update dt
			auto t2 = Tools::TimeHelp::GetTime();
			dt = Tools::TimeHelp::DeltaTime(t, t2, Tools::TimeHelp::SECONDS);
			//dt = 0.001f;
			t = t2;

			gTimeAnalyse.Step(dt, sdt);

			std::cout << "\rDT (ms): " << dt * 1000
				<< " \tAvg. " << 1000 * gTimeAnalyse.AvgDt()
				<< " \tMax. " << 1000 * gTimeAnalyse.dt_max;

			gRefreshVisual = true;
			//if (gEnableRender && gpParticleRenderer) gpParticleRenderer->DoBuffer();
		}
		else {
// 			if (gRefreshVisual && gEnableRender && gpParticleRenderer) {
// 				gpParticleSystem->RecalculateForces(); // for visualization
// 				if (gpParticleRenderer->DoBuffer()) {
// 					gRefreshVisual = false;
// 				}
// 			}

			t = Tools::TimeHelp::GetTime();
		}

		// sleep?
		//if (dt < timestep) Tools::Sleep(timestep - dt);
	}

	// if (gpEngine) gpEngine->Run_end();
}

void SystemManager::Setup()
{
	gInputListener = new System::InputManager_get();
	gpFluidSystem = new FluidSystem();
	gpFluidSystem->Setup({ 32, 32 });

	if (gEnableRender) {
		gpEngine = new GLCore::GLEngine();
		gpEngine->BootInfo().window_info.windowedPrefResolution = {600, 600};
		gpEngine->RunAsync();	
		gpFluidRenderer = new FluidRenderer(gpEngine);
		gpFluidRenderer->SetFluidSystem(gpFluidSystem);
	}
}

void SystemManager::Update(float dt)
{
	gpFluidSystem->Update(dt);
	// reset forces in particles
// 	gpParticleSystem->RecalculateForces();
// 	gpParticleSystem->DampVelocity(dt);
// 
// 	// use the solver to get new positions
// 	if (gfSolveFunc)
// 		gfSolveFunc(gpParticleSystem, dt);
// 
// 	// update positions
// 	gpParticleSystem->PostSolve();

}

void SystemManager::ReadInput()
{
	gInputListener->PrepForNextFrame();

	// press "SPACE" to play / pause
	if (gInputListener->IsKeyPressed(System::InputManager::KeyCodes::Space)) {
		gSimulationEnabled = !gSimulationEnabled;
	}

	// press "R" to reset the particle system
	if (gInputListener->IsKeyPressed('R')) {
		//gpParticleSystem->Reset();
		gTimeAnalyse.Reset();
		gRefreshVisual = true;
	}

	// 	static std::vector<std::tuple<int, SolveFunc, std::string>> solver_key_mapping = {
	// 	};

// 	for (auto entry : solver_key_mapping) {
// 		if (gInputListener->IsKeyPressed(System::InputManager::KeyCodes::Nr(std::get<0>(entry)))) {
// 			gfSolveFunc = std::get<1>(entry);
// 			std::cout << " > Solver set to: " << std::get<2>(entry) << " \t\t";
// 		}
// 	}

	System::InputManager::AState boardState;
	boardState.Shift = System::InputManager::AStateTypeLR::EITHER_DOWN;
 	static std::vector<std::tuple<int, FluidFiller, std::string>> scene_key_mapping = {
		{ 1, &FluidFillers::HalfFull, "Half full" },
 	};

 	for (auto entry : scene_key_mapping) {
 		if (gInputListener->IsKeyPressed(System::InputManager::KeyCodes::Nr(std::get<0>(entry)), boardState)) {
 			std::get<1>(entry)(gpFluidSystem);
 			std::cout << " > Scene set to: " << std::get<2>(entry) << " \t\t";
 		}
 	}
}

// void SystemManager::RenewRenderer()
// {
// 	if (!gpParticleRenderer) {
// 		gpParticleRenderer = new ParticleRenderer2(gpEngine);
// 	}
// 	gpParticleRenderer->SetParticleSystem(gpParticleSystem);
// 
// 	if (!gpUserInteraction) {
// 		gpUserInteraction = new UserInteraction(gpEngine->GetWindow()->GetInputManager(), gpEngine->GetWindow(), gpParticleRenderer);
// 	}
// 
// 	gpParticleSystem->Add(gpUserInteraction);
// }

void SystemManager::TimeAnalyse::Reset()
{
	temp_total[0] = temp_total[1] = 0;
	steps[0] = steps[1] = 0;
	dt_max = 0;
}

void SystemManager::TimeAnalyse::Step(double dt, double sdt)
{
	if (steps[current_bufferid] == stepRefresh) {
		current_bufferid = 1 - current_bufferid;
		steps[current_bufferid] = 0;
		temp_total[current_bufferid] = 0;
	}
	steps[current_bufferid] ++;
	temp_total[current_bufferid] += dt;
	if (sdt > dt_max) dt_max = sdt;
}

double SystemManager::TimeAnalyse::AvgDt()
{
	int cache_bufferid = current_bufferid ? 0 : 1;
	double avg = 0;
	int s_total = steps[cache_bufferid] + steps[current_bufferid];
	if (s_total > 0) {
		double t_total = temp_total[cache_bufferid] + temp_total[current_bufferid];
		return t_total / s_total;
	}
	else return 0;
}
