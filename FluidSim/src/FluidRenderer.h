#pragma once
#include "Inc.h"

#include <Tools/Include.h>
#include <OpenGLBase/Include.h>

class ImageShader
	: public GLBase::Shading::Shader
{
public:
	ImageShader();
	~ImageShader();
	void Prepare(GLCore::RendererContext);
	void SetIMG(GLuint imgID);

	virtual void Update(GLCore::RendererContext) override;
	virtual void Execute(GLCore::RendererContext) override;

private:
	GLuint gImgID = -1;
};

class FluidRenderer :
	public Tools::Task<GLCore::RendererContext>,
	public Tools::Updatable<GLCore::RendererContext>,
	public Tools::Renderable<GLCore::RendererContext>
{
public:
	FluidRenderer(GLCore::GLEngine*);
	~FluidRenderer();

	void SetFluidSystem(FluidSystem*);

	virtual void Execute(GLCore::RendererContext) override;
	virtual void Update(GLCore::RendererContext) override;
	virtual void Render(GLCore::RendererContext) override;

	GLuint				gFluidTexID = -1;
	FluidSystem*		gpFluidSystem = nullptr;
	std::vector<float>	gPixels;

	ImageShader*		gpImageShader = nullptr;
};

