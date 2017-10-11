// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "../ITMLib/Utils/ITMImageTypes.h"

namespace InputSource {

	class ImageSourceEngine
	{
	public:
		virtual ~ImageSourceEngine() {}

		/**
		 * \brief Gets the calibration parameters associated with the next RGB-D image (if any).
		 *
		 * \pre     hasMoreImages()
		 * \return  The calibration parameters associated with the next RGB-D image (if any), or the default calibration parameters otherwise.
		 */
		virtual ITMLib::ITMRGBDCalib getCalib() const = 0;

		/**
		 * \brief Gets the size of the next depth image (if any).
		 *
		 * \pre     hasMoreImages()
		 * \return  The size of the next depth image (if any), or Vector2i(0,0) otherwise.
		 */
		virtual Vector2i getDepthImageSize(void) const = 0;

		/**
		 * \brief Gets the next RGB and depth images (if any).
		 *
		 * \pre             hasMoreImages()
		 * \param rgb       An image into which to store the next RGB image.
		 * \param rawDepth  An image into which to store the next depth image.
		 */
		virtual void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth) = 0;

		/**
		 * \brief Gets the size of the next RGB image (if any).
		 *
		 * \pre     hasMoreImages()
		 * \return  The size of the next RGB image (if any), or Vector2i(0,0) otherwise.
		 */
		virtual Vector2i getRGBImageSize(void) const = 0;

		/**
		 * \brief Determines whether or not the image source engine has RGB-D images currently available.
		 *
		 * \return  true, if the image source engine has RGB-D images currently available, or false otherwise.
		 */
		virtual bool hasImagesNow(void) const
		{
			// By default, this just returns the same as hasMoreImages(), but it can be overridden by image
			// source engines that need to support the case of being temporarily unable to yield images.
			return hasMoreImages();
		}

		/**
		 * \brief Determines whether or not the image source engine is able to yield more RGB-D images.
		 *
		 * \return  true, if the image source engine is able to yield more RGB-D images, or false otherwise.
		 */
		virtual bool hasMoreImages(void) const = 0;
	};

	class BaseImageSourceEngine : public ImageSourceEngine
	{
	protected:
		ITMLib::ITMRGBDCalib calib;

	public:
		explicit BaseImageSourceEngine(const char *calibFilename);

		/** Override */
		virtual ITMLib::ITMRGBDCalib getCalib() const;
	};

	class ImageMaskPathGenerator
	{
	private:
		static const int BUF_SIZE = 2048;
		char rgbImageMask[BUF_SIZE];
		char depthImageMask[BUF_SIZE];

	public:
		ImageMaskPathGenerator(const char *rgbImageMask, const char *depthImageMask);
		std::string getRgbImagePath(size_t currentFrameNo) const;
		std::string getDepthImagePath(size_t currentFrameNo) const;
	};

	class ImageListPathGenerator
	{
	private:
		std::vector<std::string> depthImagePaths;
		std::vector<std::string> rgbImagePaths;

		size_t imageCount() const;

	public:
		ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_, const std::vector<std::string>& depthImagePaths_);
		std::string getRgbImagePath(size_t currentFrameNo) const;
		std::string getDepthImagePath(size_t currentFrameNo) const;

	};

	template <typename PathGenerator>
	class ImageFileReader : public BaseImageSourceEngine
	{
	private:
		ITMUChar4Image *cached_rgb;
		ITMShortImage *cached_depth;

		void loadIntoCache() const;
		mutable size_t cachedFrameNo;
		size_t currentFrameNo;
		mutable bool cacheIsValid;

		PathGenerator pathGenerator;
	public:

		ImageFileReader(const char *calibFilename, const PathGenerator& pathGenerator_, size_t initialFrameNo = 0);
		~ImageFileReader();

		bool hasMoreImages(void) const;
		void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);
		Vector2i getDepthImageSize(void) const;
		Vector2i getRGBImageSize(void) const;
	};

	class CalibSource : public BaseImageSourceEngine
	{
	private:
		Vector2i imgSize;
		void ResizeIntrinsics(ITMLib::ITMIntrinsics &intrinsics, float ratio);

	public:
		CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio);
		~CalibSource() { }

		bool hasMoreImages(void) const { return true; }
		void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth) { }
		Vector2i getDepthImageSize(void) const { return imgSize; }
		Vector2i getRGBImageSize(void) const { return imgSize; }
	};

	class RawFileReader : public BaseImageSourceEngine
	{
	private:
		static const int BUF_SIZE = 2048;
		char rgbImageMask[BUF_SIZE];
		char depthImageMask[BUF_SIZE];

		mutable ITMUChar4Image *cached_rgb;
		mutable ITMShortImage *cached_depth;

		void loadIntoCache() const;
		mutable int cachedFrameNo;
		int currentFrameNo;

		Vector2i imgSize;
		void ResizeIntrinsics(ITMLib::ITMIntrinsics &intrinsics, float ratio);

	public:
		RawFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask, Vector2i setImageSize, float ratio);
		~RawFileReader() { }

		bool hasMoreImages(void) const;
		void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

		Vector2i getDepthImageSize(void) const { return imgSize; }
		Vector2i getRGBImageSize(void) const { return imgSize; }
	};

	class BlankImageGenerator : public BaseImageSourceEngine
	{
	private:
		Vector2i imgSize;
	
	public:
		BlankImageGenerator(const char *calibFilename, Vector2i setImageSize);
		~BlankImageGenerator() { }

		bool hasMoreImages(void) const;
		void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth);

		Vector2i getDepthImageSize(void) const { return imgSize; }
		Vector2i getRGBImageSize(void) const { return imgSize; }
	};
}
