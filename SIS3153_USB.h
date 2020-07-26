// Danula Godagama (2020/06/13)


#ifndef _Included_SIS3153
#define _Included_SIS3153

#include "libusb.h"
#include <jni.h>

#ifdef __WIN64__
	#define DLLEXPORT __declspec(dllexport)
	
#else
	#define DLLEXPORT
#endif



libusb_device_handle *dev_handle;
libusb_context *ctx = NULL;
/************************************Author Information**************************************
				
				     Danula Godagama 
			    	 danula.godagama@uky.edu 
			        University of Kentucky 
				       07/25/2020 

**************************************File Information***************************************
				       SIS3153_USB.h
Header of the native driver for STRUCK SIS3153 VME controller. Written to facilitate native 
functions of SIS3153 Kmax Java driver.
*********************************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#undef SIS3153_DRV_ERR_NO_ERROR
#define SIS3153_DRV_ERR_NO_ERROR 0L
#undef SIS3153_DRV_ERR_NULL_ARRAY
#define SIS3153_DRV_ERR_NULL_ARRAY 1001L
#undef SIS3153_DRV_ERR_BAD_ELEMENT_COUNT
#define SIS3153_DRV_ERR_BAD_ELEMENT_COUNT 1002L
#undef SIS3153_DRV_ERR_BAD_ELEMENT_OFFSET
#define SIS3153_DRV_ERR_BAD_ELEMENT_OFFSET 1003L
#undef SIS3153_DRV_ERR_NO_SUCH_ADDRESS
#define SIS3153_DRV_ERR_NO_SUCH_ADDRESS 1004L
#undef SIS3153_DRV_ERR_MEMORY_ERROR
#define SIS3153_DRV_ERR_MEMORY_ERROR 1005L
#undef SIS3153_DRV_ERR_METHOD_UNIMPLEMENTED
#define SIS3153_DRV_ERR_METHOD_UNIMPLEMENTED 1007L
#undef SIS3153_DRV_ERR_NO_SUCH_FUNCTION
#define SIS3153_DRV_ERR_NO_SUCH_FUNCTION 1008L
#undef SIS3153_DRV_ERR_BAD_BUFFER_SIZE
#define SIS3153_DRV_ERR_BAD_BUFFER_SIZE 1009L
#undef SIS3153_DRV_ERR_JNI_ERROR
#define SIS3153_DRV_ERR_JNI_ERROR 1010L

DLLEXPORT int SIS3153_OPEN_DEVICE();
DLLEXPORT int SIS3153_CLOSE_DEVICE();
DLLEXPORT int usbTransaction(libusb_device_handle* device,unsigned char *outpacket,unsigned int outbytes,unsigned char *inpacket,unsigned int inbytes);
DLLEXPORT int SIS3153_REGISTER_READ(libusb_device_handle* device, unsigned long addr, unsigned long *data);
DLLEXPORT int SIS3153_REGISTER_WRITE(libusb_device_handle* device, unsigned long addr, unsigned long *data);
DLLEXPORT int SIS3153_VME_READ(libusb_device_handle* device, unsigned long addr, unsigned long vme_am, unsigned long vme_size, unsigned long fifo_mode,unsigned long* data, unsigned long req_nof_bytes, unsigned long* got_nof_bytes);
DLLEXPORT int SIS3153_VME_WRITE(libusb_device_handle* device, unsigned long addr, unsigned long vme_am, unsigned long vme_size, unsigned long fifo_mode,unsigned long* data, unsigned long req_nof_bytes, unsigned long* put_nof_bytes);

//SIS3153 D32 functions 
DLLEXPORT int SIS3153_VME_D32_READ(libusb_device_handle* device, unsigned long addr, unsigned long vme_am,unsigned long* data, unsigned long req_nof_bytes);
DLLEXPORT int SIS3153_VME_D32_WRITE(libusb_device_handle* device, unsigned long addr, unsigned long vme_am,unsigned long* data, unsigned long req_nof_bytes);

//Kmax native driver functions 

DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativeInit(JNIEnv *, jobject);
DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativeClose(JNIEnv *, jobject);
DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativeReadInt(JNIEnv *, jobject, jlong, jint, jintArray, jint, jint);
DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativewriteInt(JNIEnv *, jobject, jlong, jint, jintArray, jint, jint);



#ifdef __cplusplus
}
#endif
#endif
