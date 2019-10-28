#include "common.h"
#include "helpers/ImageData.h"
#include "helpers/OpenGLTextureObject.h"
#include "helpers/Singleton.h"

#include <glm/glm.hpp>
#include <map>
#include <list>

struct TextureData
{
	std::string name;

	ImageData data;
	GLHelpers::TextureObject* gltex;
};

class TextureManagerInstance
{

public:

	// RGBtoRGBA means load RGB images as RGBA with 1 in the alpha channel
	int                loadTexture(std::string filename, bool generate_mipmap = true, bool RGBtoRGBA = true);
	TextureData&        getTexture(std::string name);
	const TextureData&  getTexture(std::string name) const;
	bool                hasTexture(std::string name) const;
	bool             deleteTexture(std::string name);
	bool                     clear();
	int                saveTexture(std::string filename, GLHelpers::TextureObject2D tex);

protected:
	TextureManagerInstance();
	~TextureManagerInstance();

private:

	std::map<std::string, TextureData> textures;
	std::list<GLHelpers::TextureObject1D> tex1d;
	std::list<GLHelpers::TextureObject2D> tex2d;
	std::list<GLHelpers::TextureObject3D> tex3d;

	friend Singleton<TextureManagerInstance>;
};

typedef Singleton<TextureManagerInstance> TextureManager;
