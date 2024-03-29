 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>
#include <limits>       // std::numeric_limits
#include "scene.h"
#include "maths.h"
#include "sampler.h"
#include "rayAccelerator.h"

#define CAPTION "Whitted Ray-Tracer"

#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define MAX_DEPTH 4
#define DISPLACE_BIAS 0.001

#define SPP 1
unsigned int FrameCount = 0;

// Accelerators
typedef enum {NONE, GRID_ACC, BVH_ACC} Accelerator;
Accelerator Accel_Struct = GRID_ACC;
Grid* grid_ptr;
BVH* bvh_ptr;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];

//Enable OpenGL drawing.  
bool drawModeEnabled = true;

//Enable antialiasing
bool withAntialiasing = true;

//Enable soft shadows
bool softShadows = false;

//Enable fuzzy reflection
bool fuzzyReflections = false;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene
float sppSquared = sqrt(SPP);

// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;
int RES_X, RES_Y;

int WindowHandle = 0;
float dofMod = 1.0;
int dofDir = -1;

Color rayTracing(Ray ray, int depth, float ior_1);
void RayTraversal(int objectsN, Object*& currentObj, Ray& ray, float& dist, float& minDist, Object*& nearestObj);
void antiAliasedSoftShadows(Light* currentLight, Vector& actualHitPoint, Vector& L, Vector& normal, Color& lightSum, Object* obj, Vector& shadingNormal, Ray& ray);
void notAntiAliasedSoftShadows(Light* currentLight, Vector& actualHitPoint, Vector& L, Vector& normal, Color& lightSum, Object* obj, Vector& shadingNormal, Ray& ray);
void hardShadows(Light* currentLight, Vector& actualHitPoint, Vector& L, Vector& normal, Color& lightSum, Object* obj, Vector& shadingNormal, Ray& ray);
void Reflection(Vector& normal, Ray& ray, Vector& actualHitPoint, Vector& hitPoint, Color& reflectionColor, int depth, float ior_1, Object* obj, float reflectionIndex);
bool rayTraverseShadows(int objectN, Object*& currentObj, Ray& ray, float& dist, float lineLength);


bool isPointObstructed(Vector& fromPoint, Vector& toPoint)
{
	int objectN = scene->getNumObjects();
	Vector line = toPoint - fromPoint;
	float lineLength = line.length();

	Ray ray = Ray(fromPoint, line);
	Object* currentObj;
	float dist;
	if (Accel_Struct == NONE)
	{
		return rayTraverseShadows(objectN, currentObj, ray, dist, lineLength);
	}
	else if (Accel_Struct == GRID_ACC)
	{
		return grid_ptr->Traverse(ray);
	}
	else if (Accel_Struct == BVH_ACC)
	{
		return bvh_ptr->Traverse(ray);
	}

}

bool rayTraverseShadows(int objectN, Object*& currentObj, Ray& ray, float& dist, float lineLength)
{
	for (int i = 0; i < objectN; i++)
	{
		currentObj = scene->getObject(i);


		if (currentObj->intercepts(ray, dist))
		{
			if (dist <= lineLength)
				return true;
		}
	}
	return false;
}

Color calculateColor(Light* light, Object* obj, Vector& L, Vector& normal, Vector& eyeDir)
{
	Vector v, h;
	v = eyeDir * (-1);
	h = (L + v).normalize();
	Vector myNormal = normal;

	Color specular = (light->color * obj->GetMaterial()->GetSpecColor()) * obj->GetMaterial()->GetSpecular() *pow(max(h * normal,0), obj->GetMaterial()->GetShine());
	Color diffuse = (light->color * obj->GetMaterial()->GetDiffColor()) *  obj->GetMaterial()->GetDiffuse() * max((L * normal), 0);
	return diffuse + specular;
}

Color trace(Object* obj, Vector& hitPoint, Vector& normal, Ray ray, float ior_1,int depth)
{
	
	int lightN = scene->getNumLights();
	Color lightSum  = Color(0,0,0);
	Light* currentLight;
	Color objectColor = Color(0,0,0);
	Color reflectionColor, refractionColor;
	float attenuation;
	Vector actualHitPoint;
	Vector L,v,h;
	float refractionIndex;
	bool outsideIn = normal * ray.direction <= 0;


	for (int i = 0; i < lightN; i++)
	{

		currentLight = scene->getLight(i);
		L = currentLight->position - hitPoint;
		L.normalize();
		if (L * normal > 0) //isInDirectView
		{
			
			actualHitPoint = hitPoint + normal * DISPLACE_BIAS;

			Vector shadingNormal = normal;
			if (!outsideIn)
			{
				shadingNormal = normal * -1;
			}

			if(!softShadows)
			hardShadows(currentLight, actualHitPoint, L, normal, lightSum, obj, shadingNormal, ray);
			else
			{
				if(withAntialiasing)
					antiAliasedSoftShadows(currentLight, actualHitPoint, L, normal, lightSum, obj, shadingNormal, ray);
				else
					notAntiAliasedSoftShadows(currentLight, actualHitPoint, L, normal, lightSum, obj, shadingNormal, ray);
			}

		}
	}


	objectColor = lightSum.clamp(); 


	if (depth > MAX_DEPTH)
		return objectColor;

	float reflectionIndex = obj->GetMaterial()->GetReflection();
	if ( reflectionIndex > 0)
	{
		Reflection(normal, ray, actualHitPoint, hitPoint, reflectionColor, depth, ior_1, obj, reflectionIndex);
	}

	refractionIndex = obj->GetMaterial()->GetRefrIndex(); 
	if (obj->GetMaterial()->GetTransmittance() > 0)
	{
		float fromIor;
		float toIor;
		
		Vector invertedNormal = normal * -1;
		actualHitPoint = hitPoint;
		Vector mynormal = normal;

		if(ray.direction * normal <=0)
		{
			fromIor = ior_1;  
			toIor = refractionIndex;
			actualHitPoint = hitPoint + invertedNormal * DISPLACE_BIAS;
		
		}else
		{
			fromIor = ior_1;
			toIor = 1;
			
			actualHitPoint = hitPoint + normal * DISPLACE_BIAS ;
			//flip normal if hitting object from the inside, without this light would just bounce arround inside the object
			mynormal = invertedNormal;
			invertedNormal = normal;

		}


		Vector invertedRayDirection = ray.direction * -1;
		
		float cosTetaI = max(min(invertedRayDirection * mynormal, 1), -1); 
		float sinTetaI = sqrt(-1 * cosTetaI * cosTetaI + 1);
		float sinTetaT = (fromIor/ toIor ) * sinTetaI;
		float cosTetaT = sqrt(1- sinTetaT*sinTetaT);


		Vector V = mynormal *(invertedRayDirection * mynormal)  - invertedRayDirection;
		
		V.normalize();
		Vector refractionDirection = V*sinTetaT +  invertedNormal*cosTetaT;
		refractionDirection.normalize();
		
		float aux = (fromIor - toIor) / (fromIor + toIor);
		float R0 = (aux * aux);

		if (sinTetaI >= 1)///aqui cancelar refracao, só reflecao
		{
			attenuation = 1;

		}else
		{
		 attenuation = R0 + (1.0f - R0) * pow((1.0f - cosTetaI), 5);
		
		}


		Ray refractionRay =  Ray(actualHitPoint, refractionDirection);
		refractionColor = rayTracing(refractionRay, depth+1, toIor);
		refractionColor.clamp();

	}
	else
	{
		return (objectColor + reflectionColor).clamp();
	}
	
	return (objectColor + (reflectionColor* attenuation) + (refractionColor *(1- attenuation))).clamp();
}

void Reflection(Vector& normal, Ray& ray, Vector& actualHitPoint, Vector& hitPoint, Color& reflectionColor, int depth, float ior_1, Object* obj, float reflectionIndex)
{
	Vector myNormal = normal;
	Vector S, Sdir;
	if (ray.direction * normal > 0)//inside to outside
	{
		myNormal = normal * -1;
		actualHitPoint = hitPoint - normal * DISPLACE_BIAS;
	}
	else
	{
		actualHitPoint = hitPoint + normal * DISPLACE_BIAS;
	}

	Vector inverseDirection = ray.direction * -1;
	float newDirFloat = 2 * (inverseDirection * myNormal);
	Vector newDir = myNormal * newDirFloat - inverseDirection;

	if (fuzzyReflections)
	{
		scene->GetFuzzyReflector()->calculateFuzzyRayDirection(actualHitPoint, newDir, myNormal);
	}

	Ray newRay = Ray(actualHitPoint, newDir);
	reflectionColor = rayTracing(newRay, depth + 1, ior_1);
	if (obj->GetMaterial()->GetTransmittance() == 0)
	{
		reflectionColor = reflectionColor * reflectionIndex * obj->GetMaterial()->GetSpecColor();
	}
	reflectionColor.clamp();
}

void hardShadows(Light* currentLight, Vector& actualHitPoint, Vector& L, Vector& normal, Color& lightSum, Object* obj, Vector& shadingNormal, Ray& ray)
{
	Vector sample = currentLight->position;

	if (!isPointObstructed(actualHitPoint, sample) && L * normal > 0)
	{
		lightSum += calculateColor(currentLight, obj, L, shadingNormal, ray.direction);
	}
}

void antiAliasedSoftShadows(Light* currentLight, Vector& actualHitPoint, Vector& L, Vector& normal, Color& lightSum, Object* obj, Vector& shadingNormal, Ray& ray)
{
	Vector sample = currentLight->position + Vector(0, 1, 0) * rand_float() + Vector(1, 0, 0) * rand_float();

	if (!isPointObstructed(actualHitPoint, sample) && L * normal > 0)
	{
		lightSum += calculateColor(currentLight, obj, L, shadingNormal, ray.direction);
	}
}

void notAntiAliasedSoftShadows(Light* currentLight, Vector& actualHitPoint, Vector& L, Vector& normal, Color& lightSum, Object* obj, Vector& shadingNormal, Ray& ray)
{
	Vector sample = currentLight->position + Vector(0, 1, 0) * rand_float() + Vector(1, 0, 0) * rand_float();
	Color originalColor = currentLight->color;
	currentLight->color = currentLight->color * (1.0f/(sppSquared * sppSquared));
	for(int x = 0; x < sppSquared; x++)
		for (int y = 0; y < sppSquared; y++)
		{
			sample = currentLight->position + Vector(0, 1, 0) * ((x + rand_float())/sppSquared) + Vector(1, 0, 0) * ((y + rand_float()) / sppSquared);
			if (!isPointObstructed(actualHitPoint, sample) && L * normal > 0)
			{
				lightSum += calculateColor(currentLight, obj, L, shadingNormal, ray.direction);
			}
		}

	currentLight->color = originalColor;
}

Color rayTracing( Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	
	float dist ;
	int objectsN = scene->getNumObjects();
	Object* currentObj;
	Object* nearestObj = NULL;
	Vector normal, hitPoint;
	float minDist = numeric_limits<float>::max();
	
	
	if (Accel_Struct == NONE)
	{
		RayTraversal(objectsN, currentObj, ray, dist, minDist, nearestObj);
		hitPoint = ray.origin + ray.direction * minDist;
	}
	else if (Accel_Struct == GRID_ACC)
	{
		grid_ptr->Traverse(ray, &nearestObj, hitPoint);
	}
	else if (Accel_Struct == BVH_ACC)
	{
		bvh_ptr->Traverse(ray, &nearestObj, hitPoint);
	}



	if(nearestObj != NULL)
	{
		normal = nearestObj->getNormal(hitPoint);
	
	    return trace(nearestObj, hitPoint, normal, ray,ior_1, depth);
	}
	if(!P3F_scene)
		return scene->GetBackgroundColor();
//	return scene->GetBackgroundColor();
	return scene->GetSkyboxColor(ray);
}

void RayTraversal(int objectsN, Object*& currentObj, Ray& ray, float& dist, float& minDist, Object*& nearestObj)
{
	for (int i = 0; i < objectsN; i++)
	{
		currentObj = scene->getObject(i);
		if (currentObj->intercepts(ray, dist))
		{

			if (dist < minDist)
			{

				minDist = dist;
				nearestObj = currentObj;
			}

		}
	}
}



/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs


void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos=0;
	int index_col=0;
	unsigned int counter = 0;

	dofMod += 1*dofDir;
	if (dofMod == 6)
		dofDir = -1;
	if (dofMod <= 1)
		dofDir = 1;

	

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}
	set_rand_seed(42);
	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color = Color(0,0,0); 

			Vector pixel;  //viewport coordinates
			pixel.x = x + 0.5f;
			pixel.y = y + 0.5f;

			/*YOUR 2 FUNTIONS:*/
			if (withAntialiasing) {
				for (int p = 0; p < sppSquared; p++)
					for (int q = 0; q < sppSquared; q++) {
						float epsilon = rand_float();
						Vector pixelSample;  //viewport coordinates
						pixelSample.x = x + (p + epsilon) / sppSquared;
						pixelSample.y = y + (q + epsilon) / sppSquared;
						
						if(scene->GetCamera()->GetAperture() > 0)
						{
							Ray ray = scene->GetCamera()->PrimaryRay(sample_unit_disk() * scene->GetCamera()->GetAperture()* dofMod, pixelSample);
							
							color = color + rayTracing(ray, 1, 1.0);
						}
						else
						{
							Ray ray = scene->GetCamera()->PrimaryRay(pixelSample);
							color = color + rayTracing(ray, 1, 1.0);
						}

					} 
				color = color * (1.f / SPP);
			} 

			else {
				Ray ray = scene->GetCamera()->PrimaryRay(pixel);
				color = color + rayTracing(ray, 1, 1.0).clamp();

			}

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}
		}
	
	}
	if(drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}

// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


/////////////////////////////////////////////////////////////////////// SETUP

void setupCallbacks() 
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}

void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
	
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50] = "balls_low.p3f";
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}
	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	//GRID ACCELERATOR
	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		std::vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	//BVH ACCELERATOR
	else if (Accel_Struct == BVH_ACC) {
		bvh_ptr = new BVH();
		std::vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3*RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);
}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int ch;
	if (!drawModeEnabled) {

		do {
			init_scene();
			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
	   
		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////