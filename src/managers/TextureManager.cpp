#include "managers/TextureManager.h"
#include "helpers/OpenGLProgramObject.h"

#include <FreeImage.h>
#include <algorithm>

TextureManagerInstance::TextureManagerInstance()
{
	
}

TextureManagerInstance::~TextureManagerInstance()
{
	clear();
}

bool TextureManagerInstance::clear()
{
	textures.clear();
	for (auto& tex : tex1d) tex.Release();
	for (auto& tex : tex2d) tex.Release();
	for (auto& tex : tex3d) tex.Release();

	return SUCCESS;
}

bool TextureManagerInstance::deleteTexture(std::string name)
{
	if (!hasTexture(name)) return ERROR_RESOURCE_NOT_FOUND;

	{
		TextureData& td = getTexture(name);
		td.gltex->Release();
		delete td.gltex;
	}

	textures.erase(name);

	return SUCCESS;
}


int TextureManagerInstance::loadTexture(std::string filename, bool generate_mipmap, bool RGBtoRGBA)
{
	if (hasTexture(filename)) return SUCCESS;

	if (filename.size() < 4) return ERROR_READING_FILE;

	std::string suffix_4 = filename.substr(filename.size() - 4, 4);
	std::transform(suffix_4.begin(), suffix_4.end(), suffix_4.begin(), ::tolower);

	std::string suffix_5 = filename.substr(filename.size() - 5, 5);
	std::transform(suffix_5.begin(), suffix_5.end(), suffix_5.begin(), ::tolower);

	bool loaded_success = false;
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;

	if (suffix_5 == ".jpeg" || suffix_4 == ".jpg") {
		fif = FIF_JPEG;
	} else if (suffix_4 == ".png") {
		fif = FIF_PNG;
	} else if (suffix_4 == ".tif" || suffix_5 == ".tiff") {
		fif = FIF_TIFF;
	} else if (suffix_4 == ".tga") {
		fif = FIF_TARGA;
	} else if (suffix_4 == ".bmp") {
		fif = FIF_BMP;
	} else if (suffix_4 == ".hdr") {
		fif = FIF_HDR;
	} else if (suffix_4 == ".exr") {
		fif = FIF_EXR;
	} 
	
	FIBITMAP* fib = FreeImage_Load(fif, filename.c_str(), 0);
		
	TextureData td;
	if (!fib) return ERROR_LOADING_RESOURCE;

	BITMAPINFOHEADER* infoheader = FreeImage_GetInfoHeader(fib);
	BITMAPINFO* info = FreeImage_GetInfo(fib);

	td.data.m_LayerSize.x = FreeImage_GetWidth(fib);
	td.data.m_LayerSize.y = FreeImage_GetHeight(fib);
	td.data.m_sNumberOfLayers = 1;
	td.data.m_sNumberOfMipMapLevels = 1;
	td.data.m_sNumberOfFaces = 1;
	td.data.m_sBytePerPixel = FreeImage_GetBPP(fib) / 8;

	FREE_IMAGE_TYPE type = FreeImage_GetImageType(fib);

	bool addAlpha = false;

	switch (type) {

	case FIT_BITMAP:	//! standard image			: 1-, 4-, 8-, 16-, 24-, 32-bit
		td.data.m_eType = ImageType::BYTE;
		if (td.data.m_sBytePerPixel == 1) {
			td.data.m_eChannelFormat = ImageChannelFormat::RED;
			td.data.m_eFormat = ImageFormat::R8_UNORM;
		} else if (td.data.m_sBytePerPixel == 2) {
			td.data.m_eChannelFormat = ImageChannelFormat::RG;
			td.data.m_eFormat = ImageFormat::RG8_UNORM;
		} else if (td.data.m_sBytePerPixel == 3) {
			if (RGBtoRGBA) {
				addAlpha = true;
				td.data.m_sBytePerPixel = 4;
				td.data.m_eChannelFormat = ImageChannelFormat::RGBA;
				td.data.m_eFormat = ImageFormat::RGBA8_UNORM;
			} else {
				td.data.m_eChannelFormat = ImageChannelFormat::RGB;
				td.data.m_eFormat = ImageFormat::RGB8_UNORM;
			}
		} else if (td.data.m_sBytePerPixel == 4 && !addAlpha) {
			td.data.m_eChannelFormat = ImageChannelFormat::RGBA;
			td.data.m_eFormat = ImageFormat::RGBA8_UNORM;
		} else {
			goto jpg_load_end;
		}

		break;
	case FIT_UINT16:	//! array of unsigned short	: unsigned 16-bit
		td.data.m_eChannelFormat = ImageChannelFormat::RED;
		td.data.m_eType = ImageType::UNSIGNED_SHORT;
		td.data.m_eFormat = ImageFormat::R16UI;

		break;
	case FIT_INT16:     //! array of short			: signed 16-bit
		td.data.m_eChannelFormat = ImageChannelFormat::RED;
		td.data.m_eType = ImageType::SHORT;
		td.data.m_eFormat = ImageFormat::R16I;

		break;
	case FIT_UINT32:	//! array of unsigned long	: unsigned 32-bit
		td.data.m_eChannelFormat = ImageChannelFormat::RED;
		td.data.m_eType = ImageType::UNSIGNED_INT;
		td.data.m_eFormat = ImageFormat::R32UI;

		break;
	case FIT_INT32:	    //! array of long			: signed 32-bit
		td.data.m_eChannelFormat = ImageChannelFormat::RED;
		td.data.m_eType = ImageType::INT;
		td.data.m_eFormat = ImageFormat::R32I;

		break;
	case FIT_FLOAT:	    //! array of float			: 32-bit IEEE floating point
		td.data.m_eChannelFormat = ImageChannelFormat::RED;
		td.data.m_eType = ImageType::FLOAT;
		td.data.m_eFormat = ImageFormat::R32F;

		break;
	case FIT_DOUBLE:	//! array of double			: 64-bit IEEE floating point
		td.data.m_eChannelFormat = ImageChannelFormat::RED;
		td.data.m_eType = ImageType::DOUBLE;
		td.data.m_eFormat = ImageFormat::R64F;

		break;
	case FIT_RGB16:	    //! 48-bit RGB image			: 3 x 16-bit
		if (RGBtoRGBA) {
			addAlpha = true;
			td.data.m_sBytePerPixel = 8;
			td.data.m_eChannelFormat = ImageChannelFormat::RGBA;
			td.data.m_eType = ImageType::UNSIGNED_SHORT;
			td.data.m_eFormat = ImageFormat::RGBA16UI;
		} else {
			td.data.m_eChannelFormat = ImageChannelFormat::RGB;
			td.data.m_eType = ImageType::UNSIGNED_SHORT;
			td.data.m_eFormat = ImageFormat::RGB16UI;
		}
		break;
	case FIT_RGBA16:	//! 64-bit RGBA image		: 4 x 16-bit
		td.data.m_eChannelFormat = ImageChannelFormat::RGBA;
		td.data.m_eType = ImageType::UNSIGNED_SHORT;
		td.data.m_eFormat = ImageFormat::RGBA16UI;

		break;
	case FIT_RGBF:	    //! 96-bit RGB float image	: 3 x 32-bit IEEE floating point
		if (RGBtoRGBA) {
			addAlpha = true;
			td.data.m_sBytePerPixel = 16;
			td.data.m_eChannelFormat = ImageChannelFormat::RGBA;
			td.data.m_eType = ImageType::FLOAT;
			td.data.m_eFormat = ImageFormat::RGBA32F;
		} else {
			td.data.m_eChannelFormat = ImageChannelFormat::RGB;
			td.data.m_eType = ImageType::FLOAT;
			td.data.m_eFormat = ImageFormat::RGB32F;
		}
		break;
	case FIT_RGBAF:	    //! 128-bit RGBA float image	: 4 x 32-bit IEEE floating point
		td.data.m_eChannelFormat = ImageChannelFormat::RGBA;
		td.data.m_eType = ImageType::FLOAT;
		td.data.m_eFormat = ImageFormat::RGBA32F;

		break;

	default:
	case FIT_COMPLEX:	//! array of FICOMPLEX		: 2 x 64-bit IEEE floating point
	case FIT_UNKNOWN:
		goto jpg_load_end;
	}
		
	if (td.data.m_LayerSize.x == 0 || td.data.m_LayerSize.y == 0) goto jpg_load_end;

	int channelcount = 0;
	switch (td.data.m_eChannelFormat) {
	case ImageChannelFormat::RED:
	case ImageChannelFormat::GREEN:
	case ImageChannelFormat::BLUE:
	case ImageChannelFormat::ALPHA:
		channelcount = 1; break;
	case ImageChannelFormat::RG:
		channelcount = 2; break;
	case ImageChannelFormat::BGR:
	case ImageChannelFormat::RGB:
		channelcount = 3; break;
	case ImageChannelFormat::BGRA:
	case ImageChannelFormat::RGBA:
		channelcount = 4; break;
	default: break;
	}


	size_t datasize = td.data.m_sBytePerPixel * td.data.m_LayerSize.x * td.data.m_LayerSize.y * td.data.m_sNumberOfLayers * td.data.m_sNumberOfFaces;
	td.data.m_Data.resize(datasize);

//	RGBQUAD color;

	size_t scanline_size = td.data.m_LayerSize.x * td.data.m_sBytePerPixel;

	for (unsigned int y = 0; y < td.data.m_LayerSize.y; ++y) {

		BYTE   *ibytes   = (byte   *)(FreeImage_GetScanLine(fib, y));
		short  *ishorts  = (short  *)(FreeImage_GetScanLine(fib, y));
		int    *iints    = (int    *)(FreeImage_GetScanLine(fib, y));
		float  *ifloats  = (float  *)(FreeImage_GetScanLine(fib, y));
		double *idoubles = (double *)(FreeImage_GetScanLine(fib, y));

		byte   *obytes   = (byte   *)(td.data.m_Data.data() + y * scanline_size);
		short  *oshorts  = (short  *)(td.data.m_Data.data() + y * scanline_size);
		int    *oints    = (int    *)(td.data.m_Data.data() + y * scanline_size);
		float  *ofloats  = (float  *)(td.data.m_Data.data() + y * scanline_size);
		double *odoubles = (double *)(td.data.m_Data.data() + y * scanline_size);;

		switch (td.data.m_eFormat) {
		case ImageFormat::R8_UNORM:
		case ImageFormat::RG8_UNORM:
		case ImageFormat::RGB8_UNORM:
		case ImageFormat::RGBA8_UNORM:
		{
			RGBQUAD c;
			for (unsigned int x = 0; x < td.data.m_LayerSize.x; ++x) {
				FreeImage_GetPixelColor(fib, x, y, &c);
				if (channelcount >= 1) obytes[channelcount * x + 0] = c.rgbRed;
				if (channelcount >= 2) obytes[channelcount * x + 1] = c.rgbGreen;
				if (channelcount >= 3) obytes[channelcount * x + 2] = c.rgbBlue;
				if (channelcount >= 4) obytes[channelcount * x + 3] = addAlpha ? std::numeric_limits<byte>::max() : c.rgbReserved;
			}
		} break;

		case ImageFormat::R16UI:
		case ImageFormat::R16I:
		case ImageFormat::RGB16UI:
		case ImageFormat::RGB16I:
		{
			if (td.data.m_eFormat == ImageFormat::RGBA16UI && addAlpha) {
				for (unsigned int x = 0; x < td.data.m_LayerSize.x; ++x) {
					oshorts[4 * x + 0] = ishorts[3 * x + 0];
					oshorts[4 * x + 1] = ishorts[3 * x + 1];
					oshorts[4 * x + 2] = ishorts[3 * x + 2];
					oshorts[4 * x + 3] = 1;
				}
			} else {
				std::memcpy(oshorts, ishorts, scanline_size);
			}
		} break;

		case ImageFormat::R32UI:
		case ImageFormat::R32I:
		{
			std::memcpy(oints, iints, scanline_size);
		} break;

		case ImageFormat::R32F:
		case ImageFormat::RGB32F:
		case ImageFormat::RGBA32F:
		{
			if (td.data.m_eFormat == ImageFormat::RGBA32F && addAlpha) {
				for (unsigned int x = 0; x < td.data.m_LayerSize.x; ++x) {
					ofloats[4 * x + 0] = ifloats[3 * x + 0];
					ofloats[4 * x + 1] = ifloats[3 * x + 1];
					ofloats[4 * x + 2] = ifloats[3 * x + 2];
					ofloats[4 * x + 3] = 1.f;
				}
			} else {
				std::memcpy(ofloats, ifloats, scanline_size);
			}
		} break;

		case ImageFormat::R64F:
		{
			std::memcpy(odoubles, idoubles, scanline_size);
		} break;

		default:
			goto jpg_load_end;
		}
	}

	loaded_success = true;
		
	jpg_load_end:
	FreeImage_Unload(fib);

	if (!loaded_success) return ERROR_LOADING_RESOURCE;

	tex2d.push_back(GLHelpers::TextureObject2D());
	GLHelpers::TextureObject2D& newtex = tex2d.back();

	newtex.loadFromImageData(td.data, false, true);
	if (generate_mipmap) newtex.generateMipmap();
	td.gltex = &newtex;
	td.name = filename;

	textures[filename] = td;


	return SUCCESS;

}
 
int TextureManagerInstance::saveTexture(std::string filename, GLHelpers::TextureObject2D tex)
{
	size_t texdatasize = tex.bpp * tex.width * tex.height;
	if (!tex.m_uiOpenGLID || !texdatasize) return ERROR_INVALID_PARAMETER;

	std::vector<uint8_t> texdata(tex.bpp * tex.width * tex.height);

	tex.downloadTextureData(texdata.data(), 0, texdatasize);
	
	size_t pitch = tex.bpp * tex.width;

	FREE_IMAGE_TYPE image_type;

	switch (tex.internalFormat) {
	case GL_R8:
	case GL_RG8:
	case GL_RGB8:
	case GL_RGBA8:
		image_type = image_type = FIT_BITMAP;  //! standard image			: 1-, 4-, 8-, 16-, 24-, 32-bit
		break;

	case GL_R16UI:
		image_type = FIT_UINT16;  //! array of unsigned short	: unsigned 16-bit
		break;

	case GL_R16I:
		image_type = FIT_INT16;   //! array of short			: signed 16-bit
		break;

	case GL_R32UI:
		image_type = FIT_UINT32;  //! array of unsigned long	: unsigned 32-bit
		break;

	case GL_R32I:
		image_type = FIT_INT32;   //! array of long			    : signed 32-bit
		break;

	case GL_R32F:
	case GL_DEPTH_COMPONENT32F:
		image_type = FIT_FLOAT;   //! array of float			: 32-bit IEEE floating point

	case GL_RGB16F:
		image_type = FIT_RGB16;   //! 48-bit RGB image			: 3 x 16-bit
		break;

	case GL_RGBA16F:
		image_type = FIT_RGBA16;  //! 64-bit RGBA image		    : 4 x 16-bit
		break;

	case GL_RGB32F:
		image_type = FIT_RGBF;    //! 96-bit RGB float image	: 3 x 32-bit IEEE floating point
		break;

	case GL_RGBA32F:
		image_type = FIT_RGBAF;   //! 128-bit RGBA float image	: 4 x 32-bit IEEE floating point
		break;

	default:
		return ERROR_INVALID_PARAMETER;

	}

	FIBITMAP* bitmap = FreeImage_ConvertFromRawBitsEx(true, texdata.data(), image_type, tex.width, tex.height, pitch, tex.bpp, 0xFF0000, 0x00FF00, 0x0000FF, false);

	FreeImage_Save(FREE_IMAGE_FORMAT::FIF_EXR, bitmap, filename.c_str());

	FreeImage_Unload(bitmap);
}


TextureData& TextureManagerInstance::getTexture(std::string name)
{
#ifdef _DEBUG
	if (!hasTexture(name)) throw(std::runtime_error("Texture manager error: requesting non-existant texture."));
#endif

	return textures.find(name)->second;
}

const TextureData& TextureManagerInstance::getTexture(std::string name) const
{
#ifdef _DEBUG
	if (!hasTexture(name)) throw(std::runtime_error("Texture manager error: requesting non-existant texture."));
#endif

	return textures.find(name)->second;
}

bool TextureManagerInstance::hasTexture(std::string name) const
{
	return textures.find(name) != textures.end();
}

