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
	GLuint gImgID = 0;
};

class LineShader
	: public GLBase::Shading::Shader

{
public:
	void Prepare(GLCore::RendererContext);
	~LineShader();

	virtual void Update(GLCore::RendererContext) override;
	virtual void Execute(GLCore::RendererContext) override;
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
	bool DoBuffer();
	bool& RenderLines();
	bool& RenderImg();

	virtual void Execute(GLCore::RendererContext) override;
	virtual void Update(GLCore::RendererContext) override;
	virtual void Render(GLCore::RendererContext) override;

private:

	void SetupLines(GLCore::RendererContext);
	void SetupImages(GLCore::RendererContext);
	void RenderLines(GLCore::RendererContext);
	void RenderImage(GLCore::RendererContext);

	GLuint				gFluidTexID = 0;
	FluidSystem*		gpFluidSystem = nullptr;
	std::vector<float>	gPixels;

	ImageShader*		gpImageShader = nullptr;
	LineShader*			gpLineShader = nullptr;

	bool				gRenderImg = true;
	bool				gRenderLine = false;

	// lines:
	GLuint gLinePosBufferID;
	GLuint gLineColorBufferID;
	const int linepos_vsize = 2;
	const int linecol_vsize = 4;
	const int points_per_obj = 2;
	std::vector<float> gLinePositions;
	std::vector<float> gLineColors;

	const int pixel_vsize = 3;

};

