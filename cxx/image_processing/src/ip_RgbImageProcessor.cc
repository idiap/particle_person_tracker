// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_RgbImageProcessor - class to extract colour channels from images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <image_processing/ip_RgbImageProcessor.h>    // declaration of this
#include <image_processing/ip_PartialImageProvider.h> // for colour components

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

class ip_RedChannelProvider : public ip_PartialImageProvider {
    public:
    ip_RedChannelProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_RED_CHANNEL;
    };
};

class ip_GreenChannelProvider : public ip_PartialImageProvider {
    public:
    ip_GreenChannelProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_GREEN_CHANNEL;
    };
};

class ip_BlueChannelProvider : public ip_PartialImageProvider {
    public:
    ip_BlueChannelProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_BLUE_CHANNEL;
    };

};

ip_RgbImageProcessor::ip_RgbImageProcessor(ip_ImageProvider * src_img_provider)
        : mSourceImageProvider(src_img_provider) {

    int width = src_img_provider->image_buffer()->width;
    int height = src_img_provider->image_buffer()->height;
    int depth = src_img_provider->image_buffer()->depth;

    IplImage * red_channel_image = cvCreateImage(cvSize(width, height),
            depth, 1);
    mRedChannelProvider = new ip_RedChannelProvider(this, red_channel_image);
    add_provider(mRedChannelProvider);

    IplImage * green_channel_image = cvCreateImage(cvSize(width, height),
            depth, 1);
    mGreenChannelProvider = new ip_GreenChannelProvider(this,
            green_channel_image);
    add_provider(mGreenChannelProvider);

    IplImage * blue_channel_image = cvCreateImage(cvSize(width, height),
            depth, 1);
    mBlueChannelProvider = new ip_BlueChannelProvider(this,
            blue_channel_image);
    add_provider(mBlueChannelProvider);

} // ip_RgbImageProcessor

/* virtual */ ip_RgbImageProcessor::~ip_RgbImageProcessor() {

    remove_provider(mRedChannelProvider);
    remove_provider(mGreenChannelProvider);
    remove_provider(mBlueChannelProvider);

    IplImage * red_channel_image = mRedChannelProvider->image_buffer();
    cvReleaseImage(&red_channel_image);
    IplImage * green_channel_image = mGreenChannelProvider->image_buffer();
    cvReleaseImage(&green_channel_image);
    IplImage * blue_channel_image = mBlueChannelProvider->image_buffer();
    cvReleaseImage(&blue_channel_image);

    delete mRedChannelProvider;
    delete mGreenChannelProvider;
    delete mBlueChannelProvider;
}

ip_ImageProvider* ip_RgbImageProcessor::getRedChannelProvider() {
    return mRedChannelProvider;
} // getRedChannelProvider

ip_ImageProvider* ip_RgbImageProcessor::getGreenChannelProvider() {
    return mGreenChannelProvider;
} // getGreenChannelProvider

ip_ImageProvider* ip_RgbImageProcessor::getBlueChannelProvider() {
    return mBlueChannelProvider;
} // getBlueChannelProvider

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */
void
ip_RgbImageProcessor::
recompute_image(IplImage* image,
                const ip_RoiWindow& roi)
{
  // obtain the most recent image from the supplier and convert it
  IplImage* source_image = mSourceImageProvider->image(roi);
  // cvCvtPixToPlane(source_image,
  //                 mBlueChannelProvider->image_buffer(),
  //                 mGreenChannelProvider->image_buffer(),
  //                 mRedChannelProvider->image_buffer(),
  //                 NULL);

  cvSplit(source_image,
          mBlueChannelProvider->image_buffer(),
          mGreenChannelProvider->image_buffer(),
          mRedChannelProvider->image_buffer(),
          NULL);



} // recompute_image

} // namespace ImageProcessing
