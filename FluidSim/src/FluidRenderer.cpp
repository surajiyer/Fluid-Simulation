#include "FluidRenderer.h"
#include <GLCore/glengine.h>
#include <GLBase/Environment/Scene.h>
#include <FluidSystem.h>

void GLCheck(int line) {
	GLenum err;
	while ((err = glGetError()) != GL_NO_ERROR) {
		std::cout << "OpenGL error @ " << line << " :" << err << std::endl;
	}
}

FluidRenderer::FluidRenderer(GLCore::GLEngine* pEngine)
{
	if (!gpImageShader) {
		gpImageShader = new ImageShader();
		pEngine->AddTask(gpImageShader);
	}
	pEngine->AddTask(this);
}

FluidRenderer::~FluidRenderer()
{
}

void FluidRenderer::SetFluidSystem(FluidSystem* pFluidSystem)
{
	gpFluidSystem = pFluidSystem;
}

void FluidRenderer::Execute(GLCore::RendererContext rc)
{
	GLuint texID;
	glGenTextures(1, &texID);
	rc.pState->BindTexture(texID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 64, 64, 0, GL_RGB, GL_FLOAT, 0);

	gFluidTexID = texID;

	rc.pState->SetClearColor(0.1, 0.1, 0.1, 1);
	rc.pScene->AddUpdatable(this);
	rc.pScene->AddRenderable(this);
}

void FluidRenderer::Update(GLCore::RendererContext rc)
{
	if (gpFluidSystem) {
		rc.pState->BindTexture(gFluidTexID);
		auto surface = gpFluidSystem->GetSize();
		uint32_t pixel_count = surface.Area();
		gPixels.resize(pixel_count * 3);
		auto& density = gpFluidSystem->GetDensities();
		auto& vel_x = gpFluidSystem->GetVelX();
		auto& vel_y = gpFluidSystem->GetVelY();
		for (int i = 0; i < pixel_count; i++) {
			real speed = sqrt(vel_x[i] * vel_x[i] + vel_y[i] * vel_y[i]);
			real dens = density[i];
			real value = speed;
			gPixels[i * 3 + 0] = speed;
			gPixels[i * 3 + 1] = dens;
			gPixels[i * 3 + 2] = dens;
		//std::cout << data[i] << ", ";
		} // TODO : normalize?
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, surface.width, surface.height, 0, GL_RGB, GL_FLOAT, gPixels.data());
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	}
}

void FluidRenderer::Render(GLCore::RendererContext rc)
{
	if (gpImageShader->IsReady()) {
		rc.pState->SetDepthTest(false);
		rc.pState->SetCulling(false);
		gpImageShader->Prepare(rc);
		gpImageShader->SetIMG(gFluidTexID);
		rc.pEngine->GetShaderManager()->Support().drawFSQ();
	}
}

ImageShader::ImageShader()
{

}

ImageShader::~ImageShader()
{
	glDeleteProgram(programID);
}

void ImageShader::Prepare(GLCore::RendererContext rc)
{
	if (gImgID != (GLuint) -1) {
		rc.pState->UseProgram(Shader::programID);
		GLCheck(__LINE__);
		rc.pState->ActivateTextureSlot(0);
		rc.pState->BindTexture(gImgID);
		GLCheck(__LINE__);
	}
}

void ImageShader::SetIMG(GLuint imgID)
{
	gImgID = imgID;
}

void ImageShader::Update(GLCore::RendererContext rc)
{
	// rc.pState->UseProgram(Shader::programID);
	// update frame based variables.. (camera position)
	// glUniformMatrix4fv(0, 1, FALSE, rc.pScene->GetActiveCamera()->getVPMatrix().data());
}

void ImageShader::Execute(GLCore::RendererContext rc)
{
	auto shaderManager = rc.pEngine->GetShaderManager();
	Shader::programID = shaderManager->LoadShader("img_shader", "img_shader", nullptr);
	shaderManager->Register(this);
}
