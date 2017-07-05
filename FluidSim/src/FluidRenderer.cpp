#include "FluidRenderer.h"
#include <GLCore/glengine.h>
#include <GLBase/Environment/Scene.h>
#include <FluidSystem.h>
#include <Tools/Math.h>

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

	if (!gpLineShader) {
		gpLineShader = new LineShader();
		pEngine->AddTask(gpLineShader);
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

bool FluidRenderer::DoBuffer()
{
	if (gpFluidSystem == nullptr) {
		return false;
	}

	int maxElements = gpFluidSystem->GetSize().Area();

	int position_el_count = points_per_obj * linepos_vsize * maxElements;
	gLinePositions.resize(position_el_count);

	// copy positions:
	//part_lock.lock();
	//line_lock.lock();
	//part_lock.unlock();

	// copy lines
	int N2 = gpFluidSystem->GetSize().width;

	for (int i = 0; i < maxElements; i++) {
		int start_linepos = points_per_obj * linepos_vsize * i;

		auto xy = gpFluidSystem->rID(i);
		auto velX = gpFluidSystem->VelX(xy.x, xy.y);
		auto velY = gpFluidSystem->VelY(xy.x, xy.y);
		
		float mag = sqrt(velX * velX + velY * velY); //Tools::Math::Abs(velX) + Tools::Math::Abs(velY);

		float vx = velX / mag / (N2*1.05f);
		float vy = velY / mag / (N2*1.05f);

		if (mag < 0.000000000001) {
			vx = vy = 0;
		}

		float lx, ly;
		lx = (xy.x + 0.5f) / (float) N2;
		ly = (xy.y + 0.5f) / (float) N2;

		gLinePositions[start_linepos + 0] = lx;
		gLinePositions[start_linepos + 1] = ly;
		gLinePositions[start_linepos + 2] = lx + vx;
		gLinePositions[start_linepos + 3] = ly + vy;
		/*gLinePositions[start_linepos + 0] = 0.05 + 0.9 * (lx);
		gLinePositions[start_linepos + 1] = 0.05 + 0.9 * (ly);
		gLinePositions[start_linepos + 2] = 0.05 + 0.9 * (lx + vx);
		gLinePositions[start_linepos + 3] = 0.05 + 0.9 * (ly + vy);*/
	}

	//line_lock.unlock();

	auto surface = gpFluidSystem->GetSize();
	int pixel_count = surface.Area();
	gPixels.resize(pixel_count * pixel_vsize);
	auto& density = gpFluidSystem->GetDensities();
	auto& props = gpFluidSystem->GetProperties();
	auto& info = gpFluidSystem->GetInfo();

	auto& vel_x = gpFluidSystem->GetVelX();
	auto& vel_y = gpFluidSystem->GetVelY();
	for (int i = 0; i < pixel_count; i++) {
		real speed = sqrt(vel_x[i] * vel_x[i] + vel_y[i] * vel_y[i]);
		real value = speed;

		real r, g, b;
		r = g = b = 0;

		int size = surface.width;

		if (info[i] & CellInfo::FLUID) {
			real d_total = 0;
			for (int n = 0; n < density.size(); n++) {
				real d = density[n].Curr()[i];
				d_total += d;
				r += props[n].color[0] * d;
				g += props[n].color[1] * d;
				b += props[n].color[2] * d;
			}
			if (d_total > 1) {
				r /= d_total;
				g /= d_total;
				b /= d_total;
			}
		}
		else if (info[i] & CellInfo::OBJECT) {
			b = r = 1;
		}
		else {
			g = 1;
		}

		gPixels[i * pixel_vsize + 0] = r;// abs(vel_x[i]); abs(vel_y[i]);
		gPixels[i * pixel_vsize + 1] = g;//dens * 0.1;
		gPixels[i * pixel_vsize + 2] = b;//std::min(dens * 1.1, 1.0) - std::min(dens * 0.1, 1.0);
								  //std::cout << data[i] << ", ";
	}

	return true;
}

void FluidRenderer::Execute(GLCore::RendererContext rc)
{
	SetupImages(rc);
	SetupLines(rc);

	rc.pState->SetClearColor(0.0, 0.0, 0.0, 1);
	rc.pScene->AddUpdatable(this);
	rc.pScene->AddRenderable(this);
}

void FluidRenderer::Update(GLCore::RendererContext rc)
{
	if (gpFluidSystem) {
		//rc.pState->BindTexture(gFluidTexID);
		glBindTexture(GL_TEXTURE_2D, gFluidTexID);
		auto surface = gpFluidSystem->GetSize();
		if (gPixels.size() == surface.Area() * pixel_vsize) {
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, surface.width, surface.height, 0, GL_RGB, GL_FLOAT, gPixels.data());
		}
	}
}

void FluidRenderer::Render(GLCore::RendererContext rc)
{
	if (gpImageShader->IsReady()) {

		glEnable(GL_BLEND);		
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
		if (gRenderImg) RenderImage(rc);
		if (gRenderLine) RenderLines(rc);
	}
}

void FluidRenderer::SetupLines(GLCore::RendererContext rc)
{
	uint32_t maxElements = gpFluidSystem->GetSize().Area();

	// single data

	glGenBuffers(1, &gLinePosBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, gLinePosBufferID);
	glBufferData(GL_ARRAY_BUFFER, points_per_obj * maxElements * linepos_vsize * sizeof(GLfloat), NULL, GL_STREAM_DRAW);

	// instance data:

	glGenBuffers(1, &gLineColorBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, gLineColorBufferID);
	glBufferData(GL_ARRAY_BUFFER, points_per_obj * maxElements * linecol_vsize * sizeof(GLfloat), NULL, GL_STREAM_DRAW);
}

void FluidRenderer::SetupImages(GLCore::RendererContext rc)
{
	GLuint texID;
	glGenTextures(1, &texID);
	//rc.pState->BindTexture(texID);
	glBindTexture(GL_TEXTURE_2D, texID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 64, 64, 0, GL_RGB, GL_FLOAT, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // GL_LINEAR GL_NEAREST
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	gFluidTexID = texID;
}

void Grad(float* color, float mag)
{
	mag = Tools::Clamp<float>(mag, 0, 1);
	color[0] = (mag);
	color[1] = 0;
	color[2] = (1.f - mag);
}

void FluidRenderer::RenderLines(GLCore::RendererContext rc)
{
	gpLineShader->Prepare(rc);

	int maxObjects = gpFluidSystem->GetSize().Area();

	int position_el_count = points_per_obj * linepos_vsize * maxObjects;

	int colors_el_count = points_per_obj * linecol_vsize * maxObjects;
	gLineColors.resize(colors_el_count);

	float color_l1[] = { 0, 1, 0, 1 };
	float color_l2[] = { 0, 1, 0, 1 };

	int N2 = gpFluidSystem->GetSize().width;
	int step = points_per_obj * linecol_vsize;
	for (int i = 0; i < maxObjects; i ++) {

		auto xy = gpFluidSystem->rID(i);
		auto velX = gpFluidSystem->VelX(xy.x, xy.y);
		auto velY = gpFluidSystem->VelY(xy.x, xy.y);

		float mag = sqrt(velX * velX + velY * velY); //Tools::Math::Abs(velX) + Tools::Math::Abs(velY);

		color_l1[3] = 1;// 0.5 + mag * 0.5;
		color_l2[3] = 1;// 0.5 + mag * 0.5;

		Grad(color_l1, mag);
		Grad(color_l2, mag);

		for (int j = 0; j < 4; j++) {
			gLineColors[i * step + j] = color_l1[j];
		}
		for (int j = 0; j < 4; j++) {
			gLineColors[i * step + j + 4] = j == 3 ? 0.5f : color_l2[j];
		}
	}

	//Tools::Fill(gLineColors, (float) 1);
	
	glBindBuffer(GL_ARRAY_BUFFER, gLinePosBufferID);
	glBufferData(GL_ARRAY_BUFFER, position_el_count * sizeof(GLfloat), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
	glBufferSubData(GL_ARRAY_BUFFER, 0, position_el_count * sizeof(GLfloat), gLinePositions.data());

	glBindBuffer(GL_ARRAY_BUFFER, gLineColorBufferID);
	glBufferData(GL_ARRAY_BUFFER, colors_el_count * sizeof(GLfloat), NULL, GL_STREAM_DRAW); // Buffer orphaning, a common way to improve streaming perf. See above link for details.
	glBufferSubData(GL_ARRAY_BUFFER, 0, colors_el_count * sizeof(GLfloat), gLineColors.data());

	// 1rst attribute buffer : locations
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, gLinePosBufferID);
	glVertexAttribPointer(
		0, // attribute. No particular reason for 0, but must match the layout in the shader.
		linepos_vsize, // size
		GL_FLOAT, // type
		GL_FALSE, // normalized?
		0, // stride
		(void*) 0 // array buffer offset
	);

	// 2nd attribute buffer : colors
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, gLineColorBufferID);
	glVertexAttribPointer(
		1, // attribute. No particular reason for 1, but must match the layout in the shader.
		linecol_vsize, // size : x + y + z + size => 4
		GL_FLOAT, // type
		GL_FALSE, // normalized?
		0, // stride
		(void*) 0 // array buffer offset
	);

	glDisableVertexAttribArray(2);

	glVertexAttribDivisor(0, 0); // line points
	glVertexAttribDivisor(1, 0); // colors

	glDrawArrays(GL_LINES, 0, maxObjects * points_per_obj);
}

bool& FluidRenderer::RenderLines()
{
	return gRenderLine;
}

bool& FluidRenderer::RenderImg()
{
	return gRenderImg;
}

void FluidRenderer::RenderImage(GLCore::RendererContext rc)
{
	rc.pState->SetDepthTest(false);
	rc.pState->SetCulling(false);
	gpImageShader->Prepare(rc);
	gpImageShader->SetIMG(gFluidTexID);
	rc.pEngine->GetShaderManager()->Support().drawFSQ();
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
	if (gImgID != (GLuint) 0) {
		rc.pState->UseProgram(Shader::programID);
		rc.pState->ActivateTextureSlot(0);
		//rc.pState->BindTexture(gImgID);
		glBindTexture(GL_TEXTURE_2D, gImgID);
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

void LineShader::Prepare(GLCore::RendererContext rc)
{
	rc.pState->UseProgram(Shader::programID);
}

void LineShader::Update(GLCore::RendererContext rc)
{
	rc.pState->UseProgram(Shader::programID);
	// update frame based variables.. (camera position)
	// glUniformMatrix4fv(0, 1, FALSE, rc.pScene->GetActiveCamera()->getVPMatrix().data());
}

void LineShader::Execute(GLCore::RendererContext rc)
{
	auto shaderManager = rc.pEngine->GetShaderManager();
	Shader::programID = shaderManager->LoadShader("ss_instanced_color_line", "ss_instanced_color_line", nullptr);
	shaderManager->Register(this);
}
