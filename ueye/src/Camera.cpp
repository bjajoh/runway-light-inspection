/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2016, Kevin Hallenbeck
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin Hallenbeck nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ueye/Camera.h>

// Check expected uEye SDK version in ueye.h for supported architectures
#if defined(__i386) || defined(__i386__) || defined(_M_IX86)
  #define EXPECTED_VERSION_MAJOR 4
  #define EXPECTED_VERSION_MINOR 60
  #define EXPECTED_VERSION_BUILD 5
  #if UEYE_VERSION_CODE != UEYE_VERSION(EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR, 0)
  #warning Expected ueye driver version 4.60.x. Different version found in ueye.h.
  #endif
#elif defined(__x86_64) || defined(__x86_64__) || defined(__amd64) || defined(_M_X64)
  #define EXPECTED_VERSION_MAJOR 4
  #define EXPECTED_VERSION_MINOR 60
  #define EXPECTED_VERSION_BUILD 5
  #if UEYE_VERSION_CODE != UEYE_VERSION(EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR, 0)
  #warning Expected ueye driver version 4.60.x. Different version found in ueye.h.
  #endif
#elif defined(__arm__) || defined(__TARGET_ARCH_ARM)
  #define EXPECTED_VERSION_MAJOR 4
  #define EXPECTED_VERSION_MINOR 60
  #define EXPECTED_VERSION_BUILD 0
  #if UEYE_VERSION_CODE != UEYE_VERSION(EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR, 0)
  #warning Expected ueye driver version 4.60.x. Different version found in ueye.h.
  #endif
#elif defined(__ia64) || defined(__ia64__) || defined(_M_IA64)
  #define EXPECTED_VERSION_MAJOR 0
  #define EXPECTED_VERSION_MINOR 0
  #define EXPECTED_VERSION_BUILD 0
  #warning Architecture ia64 not explicitly supported.
#elif defined(__ppc__) || defined(__ppc) || defined(__powerpc__) || defined(_ARCH_COM) || defined(_ARCH_PWR) || defined(_ARCH_PPC) || defined(_M_MPPC) || defined(_M_PPC)
  #define EXPECTED_VERSION_MAJOR 0
  #define EXPECTED_VERSION_MINOR 0
  #define EXPECTED_VERSION_BUILD 0
  #warning Architecture ppc not explicitly supported.
#else
  #define EXPECTED_VERSION_MAJOR 0
  #define EXPECTED_VERSION_MINOR 0
  #define EXPECTED_VERSION_BUILD 0
  #warning Architecture not explicitly supported. Supported: amd64, i386, arm.
#endif


namespace ueye
{

void Camera::initPrivateVariables()
{
  streaming_ = false;
  stop_capture_ = false;
  color_mode_ = MONO8;
  auto_exposure_ = false;
  exposure_time_ = 99.0;
  hardware_gamma_ = true;
  gain_boost_ = false;
  zoom_ = 1;
  pixel_clock_ = 20;
  auto_gain_ = false;
  hardware_gain_ = 100;
  frame_rate_ = 5.0;
  flash_global_params_ = false;
  serial_number_ = 0;
  cam_ = 0;
  memset(&cam_info_, 0x00, sizeof(cam_info_));
  stream_callback_ = NULL;
}

Camera::Camera()
{
  initPrivateVariables();
}

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
bool Camera::checkVersion(int &major, int &minor, int &build, const char *&expected)
{
  expected = STR(EXPECTED_VERSION_MAJOR) "." STR(EXPECTED_VERSION_MINOR) "." STR(EXPECTED_VERSION_BUILD);
  build = is_GetDLLVersion();
  major = (build >> 24) & 0x000000FF;
  minor = (build >> 16) & 0x000000FF;
  build &= 0x0000FFFF;
  if ((major == EXPECTED_VERSION_MAJOR) && (minor == EXPECTED_VERSION_MINOR) && (build == EXPECTED_VERSION_BUILD)) {
    return true;
  }
  return false;
}
#undef STR_HELPER
#undef STR

int Camera::getNumberOfCameras() const
{
  int num = 0;
  checkError(is_GetNumberOfCameras(&num));
  return num;
}

unsigned int Camera::getSerialNumberList(std::vector<unsigned int>& serial, std::vector<unsigned int>& dev_id)
{
  int num = getNumberOfCameras();
  if (num > 0) {
    UEYE_CAMERA_LIST *list = (UEYE_CAMERA_LIST *)malloc(sizeof(DWORD) + num * sizeof(UEYE_CAMERA_INFO));
    list->dwCount = num;
    if (is_GetCameraList(list) == IS_SUCCESS) {
      num = list->dwCount;
      serial.resize(num);
      dev_id.resize(num);
      for (int i = 0; i < num; i++) {
        serial[i] = atoll(list->uci[i].SerNo);
        dev_id[i] = list->uci[i].dwDeviceID;
      }
    } else {
      num = 0;
    }
    free(list);
    return num;
  }
  return 0;
}

bool Camera::openCameraCamId(unsigned int id)
{
  if (getNumberOfCameras() < 1) {
    return false;
  }

  cam_ = id;
  checkError(is_InitCamera(&cam_, 0));

  checkError(is_GetSensorInfo(cam_, &cam_info_));
  CAMINFO info;
  checkError(is_GetCameraInfo(cam_, &info));
  serial_number_ = atoll(info.SerNo);

  setColorMode(color_mode_);
  setAutoExposure(&auto_exposure_);
  if (!auto_exposure_) {
    setExposure(&exposure_time_);
  }
  setHardwareGamma(&hardware_gamma_);
  setGainBoost(&gain_boost_);
  setAutoGain(&auto_gain_);
  if (!auto_gain_) {
    setHardwareGain(&hardware_gain_);
  }
  setZoom(&zoom_);
  setPixelClock(&pixel_clock_);
  setFrameRate(&frame_rate_);
  return true;
}
bool Camera::openCameraDevId(unsigned int id)
{
  return openCameraCamId(id | IS_USE_DEVICE_ID);
}
bool Camera::openCameraSerNo(unsigned int serial_number)
{
  std::vector<unsigned int> serial;
  std::vector<unsigned int> dev_id;
  unsigned int num = getSerialNumberList(serial, dev_id);
  for (unsigned int i = 0; i < num; i++) {
    if (serial[i] == serial_number) {
      return openCameraDevId(dev_id[i]);
    }
  }
  return false;
}

const char* Camera::colorModeToString(uEyeColor mode)
{
  switch (mode) {
    case MONO8:
      return "mono8";
    case MONO16:
      return "mono16";
    case BGR5:
      return "bgr5";
    case BGR565:
      return "bgr565";
    case YUV:
      return "yuv422";
    case YCbCr:
      return "ycbcr422";
    case BGR8:
      return "bgr8";
    case RGB8:
      return "rgb8";
    case BGRA8:
    case BGRY8:
      return "bgra8";
    case RGBA8:
    case RGBY8:
      return "rgba8";
  }
  return "";
}

double Camera::getExposure()
{
  double time_ms;
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_GET_EXPOSURE, &time_ms, sizeof(double)));
  return time_ms;
}
unsigned int Camera::getHardwareGain()
{
  hardware_gain_ = is_SetHWGainFactor(cam_, IS_GET_MASTER_GAIN_FACTOR, 0);
  return hardware_gain_;
}
TriggerMode Camera::getTriggerMode()
{
  return (TriggerMode)is_SetExternalTrigger(cam_, IS_GET_EXTERNALTRIGGER);
}
TriggerMode Camera::getSupportedTriggers()
{
  return (TriggerMode)is_SetExternalTrigger(cam_, IS_GET_SUPPORTED_TRIGGER_MODE);
}

void Camera::setColorMode(uEyeColor mode)
{
  bool restart = streaming_ && (stream_callback_ != NULL);
  stopVideoCapture();
  if (is_SetColorMode(cam_, mode) != IS_SUCCESS) {
    mode = MONO8;
    is_SetColorMode(cam_, mode);
  }
  color_mode_ = mode;
  if (restart) {
    startVideoCapture(stream_callback_);
  }
}
void Camera::setAutoExposure(bool *enable)
{
  double param1 = *enable ? 1.0 : 0.0;
  double param2 = 0;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2);
    *enable = false;
  }
  auto_exposure_ = *enable;
}
void Camera::setExposure(double *time_ms)
{
  bool b = false;
  setAutoExposure(&b);
  checkError(is_Exposure(cam_, IS_EXPOSURE_CMD_SET_EXPOSURE, time_ms, sizeof(double)));
  flashUpdateGlobalParams();
  exposure_time_ = *time_ms;
}
void Camera::setHardwareGamma(bool *enable)
{
  if (*enable) {
    if (IS_SUCCESS != is_SetHardwareGamma(cam_, IS_SET_HW_GAMMA_ON)) {
      is_SetHardwareGamma(cam_, IS_SET_HW_GAMMA_OFF);
      *enable = false;
    }
  } else {
    is_SetHardwareGamma(cam_, IS_SET_HW_GAMMA_OFF);
  }
  hardware_gamma_ = *enable;
}
void Camera::setZoom(int *zoom)
{
  if (zoom_ != *zoom) {
    // Reset zoom
    is_SetSubSampling(cam_, 0);
    is_SetBinning(cam_, 0);

    // Try Subsampling then Binning
    if (IS_SUCCESS != is_SetSubSampling(cam_, getSubsampleParam(zoom))) {
      is_SetSubSampling(cam_, 0);
      if (IS_SUCCESS != is_SetBinning(cam_, getBinningParam(zoom))) {
        is_SetBinning(cam_, 0);
        *zoom = 1;
      }
    }

    // Zoom affects the frame-rate and needs a restart to change the buffer sizes
    is_HotPixel(cam_, IS_HOTPIXEL_ENABLE_CAMERA_CORRECTION, NULL, 0);
    setFrameRate(&frame_rate_);
    restartVideoCapture();
  }
  zoom_ = *zoom;
}
void Camera::setPixelClock(int *MHz)
{
  int pixelClockList[150];  // No camera has more than 150 different pixel clocks (uEye manual)
  int numberOfSupportedPixelClocks = 0;
  checkError(is_PixelClock(cam_, IS_PIXELCLOCK_CMD_GET_NUMBER, &numberOfSupportedPixelClocks, sizeof(numberOfSupportedPixelClocks)));
  if(numberOfSupportedPixelClocks > 0) {
    memset(pixelClockList, 0x00, sizeof(pixelClockList));
    checkError(is_PixelClock(cam_, IS_PIXELCLOCK_CMD_GET_LIST, pixelClockList, numberOfSupportedPixelClocks * sizeof(int)));
  }
  int minPixelClock = pixelClockList[0];
  int maxPixelClock = pixelClockList[numberOfSupportedPixelClocks-1];

  // As list is sorted smallest to largest...
  for(int i = 0; i < numberOfSupportedPixelClocks; i++) { 
    if(*MHz <= pixelClockList[i]) {
      *MHz = pixelClockList[i];  // ...get the closest-larger-or-equal from the list
      break;
    }  
  }

  if (*MHz < minPixelClock) {
    *MHz = minPixelClock;  // Clip to min
  }
  if (*MHz > maxPixelClock) {
    *MHz = maxPixelClock;  // Clip to max
  }

  checkError(is_PixelClock(cam_, IS_PIXELCLOCK_CMD_SET, MHz, sizeof(int)));
  setFrameRate(&frame_rate_);

  pixel_clock_ = *MHz;
}
void Camera::setFrameRate(double *rate)
{
  checkError(is_SetFrameRate(cam_, *rate, rate));
  flashUpdateGlobalParams();
  frame_rate_ = *rate;
}
void Camera::setGainBoost(bool *enable)
{
  if (is_SetGainBoost(cam_, IS_GET_SUPPORTED_GAINBOOST) == IS_SET_GAINBOOST_ON) {
    if (*enable)
      is_SetGainBoost(cam_, IS_SET_GAINBOOST_ON);
    else
      is_SetGainBoost(cam_, IS_SET_GAINBOOST_OFF);
    gain_boost_ = is_SetGainBoost(cam_, IS_GET_GAINBOOST) == IS_SET_GAINBOOST_ON;
  } else {
    gain_boost_ = false;
  }
  *enable = gain_boost_;
}
void Camera::setAutoGain(bool *enable)
{
  double param1 = *enable ? 1.0 : 0.0;
  double param2 = 0;
  if (IS_SUCCESS != is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_GAIN, &param1, &param2)) {
    param1 = 0;
    is_SetAutoParameter(cam_, IS_SET_ENABLE_AUTO_GAIN, &param1, &param2);
    *enable = false;
  }
  auto_gain_ = *enable;
}
void Camera::setHardwareGain(int *gain)
{
  bool b = false;
  setAutoGain(&b);
  if (*gain < 0)
    *gain = 0;
  if (*gain > 400)
    *gain = 400;
  hardware_gain_ = is_SetHWGainFactor(cam_, IS_SET_MASTER_GAIN_FACTOR, *gain);
  *gain = hardware_gain_;
}
bool Camera::setTriggerMode(TriggerMode mode)
{
  if ((mode == 0) || (mode & getSupportedTriggers())) {
    if (is_SetExternalTrigger(cam_, mode) == IS_SUCCESS) {
      return true;
    }
  }
  return false;
}
void Camera::setFlashWithGlobalParams(FlashMode mode)
{
  UINT m = mode;
  switch (mode) {
    case FLASH_FREERUN_ACTIVE_LO:
    case FLASH_FREERUN_ACTIVE_HI:
    case FLASH_TRIGGER_ACTIVE_LO:
    case FLASH_TRIGGER_ACTIVE_HI:
      flash_global_params_ = true;
      break;

    case FLASH_CONSTANT_HIGH:
    case FLASH_CONSTANT_LOW:
      flash_global_params_ = false;
      break;

    case FLASH_OFF:
    default:
      flash_global_params_ = false;
      m = FLASH_OFF;
      break;
  }
  checkError(is_IO(cam_, IS_IO_CMD_FLASH_SET_MODE, (void*)&m, sizeof(m)));
  flashUpdateGlobalParams();
}
void Camera::setFlash(FlashMode mode, int delay_usec, unsigned int duration_usec)
{
  int num_mode = int(mode);

  checkError(is_IO(cam_, IS_IO_CMD_FLASH_SET_MODE, (void*)&num_mode, sizeof(num_mode)));

  if (mode != FLASH_OFF) {
    IO_FLASH_PARAMS params;
    memset(&params, 0, sizeof(params));

    params.s32Delay = delay_usec;
    params.u32Duration = duration_usec;

    checkError(is_IO(cam_, IS_IO_CMD_FLASH_SET_PARAMS, &params, sizeof(params)));
  }

  flash_global_params_ = false;
}
void Camera::setFlash(FlashMode mode)
{
  // If flash delay = 0 and flash duration = 0, the flash signal is
  // automatically synchronized to the exposure time.
  setFlash(mode, 0, 0);
}
void Camera::flashUpdateGlobalParams()
{
  if (flash_global_params_) {
    IO_FLASH_PARAMS params;
    checkError(is_IO(cam_, IS_IO_CMD_FLASH_GET_GLOBAL_PARAMS, (void*)&params, sizeof(params)));
    checkError(is_IO(cam_, IS_IO_CMD_FLASH_APPLY_GLOBAL_PARAMS, NULL, 0));
  }
}
void Camera::setTriggerDelay(int delay_usec)
{
  checkError(is_SetTriggerDelay(cam_, delay_usec));
}

bool Camera::forceTrigger()
{
  if (streaming_)
    return is_ForceTrigger(cam_) == IS_SUCCESS;
  return false;
}

int Camera::getSubsampleParam(int *scale)
{
  int param;
  switch (*scale) {
    case 2:
      param = IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL;
      break;
    case 3:
      param = IS_SUBSAMPLING_3X_VERTICAL | IS_SUBSAMPLING_3X_HORIZONTAL;
      break;
    case 4:
      param = IS_SUBSAMPLING_4X_VERTICAL | IS_SUBSAMPLING_4X_HORIZONTAL;
      break;
    case 5:
      param = IS_SUBSAMPLING_5X_VERTICAL | IS_SUBSAMPLING_5X_HORIZONTAL;
      break;
    case 6:
      param = IS_SUBSAMPLING_6X_VERTICAL | IS_SUBSAMPLING_6X_HORIZONTAL;
      break;
    case 7:
    case 8:
      *scale = 8;
      param = IS_SUBSAMPLING_8X_VERTICAL | IS_SUBSAMPLING_8X_HORIZONTAL;
      break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
      *scale = 16;
      param = IS_SUBSAMPLING_16X_VERTICAL | IS_SUBSAMPLING_16X_HORIZONTAL;
      break;
    default:
      *scale = 1;
      param = IS_SUBSAMPLING_DISABLE;
      break;
  }
  return param;
}
int Camera::getBinningParam(int *scale)
{
  int param;
  switch (*scale) {
    case 2:
      param = IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL;
      break;
    case 3:
      param = IS_BINNING_3X_VERTICAL | IS_BINNING_3X_HORIZONTAL;
      break;
    case 4:
      param = IS_BINNING_4X_VERTICAL | IS_BINNING_4X_HORIZONTAL;
      break;
    case 5:
      param = IS_BINNING_5X_VERTICAL | IS_BINNING_5X_HORIZONTAL;
      break;
    case 6:
      param = IS_BINNING_6X_VERTICAL | IS_BINNING_6X_HORIZONTAL;
      break;
    case 7:
    case 8:
      *scale = 8;
      param = IS_BINNING_8X_VERTICAL | IS_BINNING_8X_HORIZONTAL;
      break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
      *scale = 16;
      param = IS_BINNING_16X_VERTICAL | IS_BINNING_16X_HORIZONTAL;
      break;
    default:
      *scale = 1;
      param = IS_BINNING_DISABLE;
      break;
  }
  return param;
}

void Camera::closeCamera()
{
  if (cam_ > 0) {
    // Release camera and all associated memory
    checkError(IS_SUCCESS != is_ExitCamera(cam_));
    initPrivateVariables();
  }
}

Camera::~Camera()
{
  closeCamera();
}

void Camera::initMemoryPool(int size)
{
  int bits = 32;
  switch (color_mode_) {
    case MONO8:
      bits = 8;
      break;
    case MONO16:
    case BGR5:
    case BGR565:
    case YUV:
    case YCbCr:
      bits = 16;
      break;
    case BGR8:
    case RGB8:
      bits = 24;
      break;
    case BGRA8:
    case BGRY8:
    case RGBA8:
    case RGBY8:
      bits = 32;
      break;
  }

  int width = getWidth();
  int height = getHeight();
  if (size < 2) {
    size = 2;
  }
  img_mem_.resize(size);
  img_mem_id_.resize(size);
  for (int i = 0; i < size; i++) {
    if (IS_SUCCESS != is_AllocImageMem(cam_, width, height, bits, &img_mem_[i], &img_mem_id_[i])) {
      throw uEyeException(-1, "Failed to initialize memory.");
    }
    //add memory to memory pool
    if (IS_SUCCESS != is_SetImageMem(cam_, img_mem_[i], img_mem_id_[i])) {
      throw uEyeException(-1, "Failed to initialize memory.");
    }
  }
}
void Camera::destroyMemoryPool()
{
  for (int i = 0; i < img_mem_.size(); i++) {
    checkError(is_FreeImageMem(cam_, img_mem_[i], img_mem_id_[i]));
  }
  img_mem_.clear();
  img_mem_id_.clear();
}

void Camera::captureThread(CamCaptureCB callback)
{
  streaming_ = true;
  stop_capture_ = false;

  initMemoryPool(4);

  // Setup video event
  checkError(is_EnableEvent(cam_, IS_SET_EVENT_FRAME));

  // There is a weird condition with the uEye SDK 4.30 where
  // this never returns when using IS_WAIT.
  // We are using IS_DONT_WAIT and retry every 0.1s for 2s instead
  bool capture = false;
  for (int i = 0; i < 20; ++i) {
    if (is_CaptureVideo(cam_, IS_DONT_WAIT) == IS_SUCCESS) {
      capture = true;
      break;
    }
    usleep(100000); // 100ms
  }
  if (!capture) {
    throw uEyeException(-1, "Capture could not be started.");
  }

  size_t depth = 0;
  switch (color_mode_) {
    case MONO8:
      depth = 1;
      break;
    case MONO16:
    case BGR5:
    case BGR565:
      depth = 1;
      break;
    case YUV:
    case YCbCr:
      depth = 2;
      break;
    case BGR8:
    case RGB8:
      depth = 3;
      break;
    case BGRA8:
    case BGRY8:
    case RGBA8:
    case RGBY8:
      depth = 4;
      break;
    default:
      throw uEyeException(-1, "Unsupported color mode when initializing image header.");
      return;
  }
  size_t size = (size_t)getWidth() * (size_t)getHeight() * depth;

  char *img_mem;
  while (!stop_capture_) {
    // Wait for image. Timeout after 2*FramePeriod = (2000ms/FrameRate)
    if (is_WaitEvent(cam_, IS_SET_EVENT_FRAME, (int)(2000 / frame_rate_)) == IS_SUCCESS) {
      if (is_GetImageMem(cam_, (void**)&img_mem) == IS_SUCCESS) {
        callback(img_mem, size);
      }
    }
  }

  // Stop video event
  checkError(is_DisableEvent(cam_, IS_SET_EVENT_FRAME));
  checkError(is_StopLiveVideo(cam_, IS_WAIT));

  destroyMemoryPool();
  streaming_ = false;
}

void Camera::startVideoCapture(CamCaptureCB callback)
{
  stream_callback_ = callback;
  thread_ = boost::thread(&Camera::captureThread, this, callback);
}
void Camera::stopVideoCapture()
{
  stop_capture_ = true;
  if (thread_.joinable()) {
    forceTrigger();
    thread_.join();
  }
}
void Camera::restartVideoCapture()
{
  if (streaming_) {
    if (stream_callback_ != NULL) {
      stopVideoCapture();
      startVideoCapture(stream_callback_);
    }
  }
}

} //namespace ueye
