/************************************Author Information**************************************
				
				     Danula Godagama 
			    	 danula.godagama@uky.edu 
			        University of Kentucky 
				       07/25/2020 

**************************************File Information***************************************
				       SIS3153_USB.cpp
Source of the native driver for STRUCK SIS3153 VME controller. Written to facilitate native 
functions of SIS3153 Kmax Java driver.
*********************************************************************************************/


#include "libusb.h"
#include "SIS3153_USB.h"
#include <stdio.h>
#include <string.h>


#define USB_MAX_NOF_BYTES    0xf800
#define USB_MAX_NOF_LWORDS   USB_MAX_NOF_BYTES/4

#define sis3153usb_error_code_invalid_parameter  			0x110 
#define sis3153usb_error_code_usb_write_error    			0x111 
#define sis3153usb_error_code_usb_read_error     			0x112 
#define sis3153usb_error_code_usb_read_length_error     		0x113

static const int    USB_WRITE_ENDPOINT = 2;
static const int    USB_READ_ENDPOINT  = 0x86;
static const int    USB_TIMEOUT        = 30000;     /* This is in ms. */


libusb_device_handle *dev_handle=NULL;
libusb_context *ctx=NULL;
bool SIS3153_Device_open=false;

 DLLEXPORT int SIS3153_OPEN_DEVICE(){
	 
	libusb_device **devs;
	 
	if(libusb_init(&ctx)<0){
		return 1011;
	} 
		
	libusb_get_device_list(ctx, &devs);
		
	dev_handle = libusb_open_device_with_vid_pid(ctx,0x1657,0x3153);
	
	if(dev_handle == NULL){
        	return 1012;
	}
    	
	libusb_free_device_list(devs, 1);

	if(libusb_kernel_driver_active(dev_handle, 0) == 1) { 

       	
		if(libusb_detach_kernel_driver(dev_handle, 0) != 0) 

	           return 1013;
    	}
	    		
	if(libusb_claim_interface(dev_handle, 0)!=0){
		
		return 1014;
	}

	return 0;
 }



DLLEXPORT int SIS3153_CLOSE_DEVICE(){

	if(libusb_release_interface(dev_handle, 0)!=0){ //release the claimed interface
	        return -1;
    	}

	libusb_close(dev_handle); 
	libusb_exit(ctx); 

	return 0;

}


DLLEXPORT int usbTransaction(libusb_device_handle* device,
			  unsigned char*   outpacket,
			  unsigned int    outbytes,
			  unsigned char*   inpacket,
			  unsigned int    inbytes)
{
  int   status;
  int   nBytes;

  


  status = libusb_bulk_transfer(device, (USB_WRITE_ENDPOINT),outpacket, outbytes,&nBytes, USB_TIMEOUT);
  //cout<<"Write status:"<<status<<endl;
  if(status < 0) {
    return -1;
  }
  

  
  status = libusb_bulk_transfer(device, (USB_READ_ENDPOINT),inpacket,inbytes,&nBytes, USB_TIMEOUT);
  //cout<<"Read status:"<<status<<endl;
  if(status < 0) {
    return -2;
  }
  return nBytes;
}


DLLEXPORT int SIS3153_REGISTER_READ(libusb_device_handle* device, uint32_t addr, uint32_t *data){

  int return_code ;
  unsigned int nBytes  = 0;
  
  int req_nof_lwords=1;
  unsigned int usb_wlength;
  unsigned int usb_rlength;

  unsigned char cUsbBuf[0x100+USB_MAX_NOF_BYTES];
  unsigned char cInPacket[0x100 + USB_MAX_NOF_BYTES];

  if(req_nof_lwords > USB_MAX_NOF_LWORDS) {
     return_code = sis3153usb_error_code_invalid_parameter ;
     return  return_code ;
  }

	cUsbBuf[0]  =   (char)  0x00 ;	   // header 7:0 	  ; :
	cUsbBuf[1]  =   (char)  0x10 ;	   // header 15:8 	   Bit0 = 11 : not Write
	cUsbBuf[2]  =   (char)  0xaa ;	   // header 23:16
	cUsbBuf[3]  =   (char)  0xaa ;	   // header 31:24
	
	cUsbBuf[4]  =   (char)  req_nof_lwords   ;       //length 7:0
	cUsbBuf[5]  =   (char) (req_nof_lwords >> 8);    //length 15:8
	cUsbBuf[6]  =   (char)  0x0 ;   
	cUsbBuf[7]  =   (char)  0x0 ;
         
	cUsbBuf[8]  =   (char)  addr   ;       //addr 7:0
	cUsbBuf[9]  =   (char) (addr >> 8);    //addr 15:8
	cUsbBuf[10] =   (char) (addr >> 16) ;  //addr 23:16 
	cUsbBuf[11] =   (char) (addr >> 24);   //addr 31:24

	usb_wlength = 12;

	usb_rlength = (req_nof_lwords * 4) ; // data: (req_nof_lwords * 4) Bytes; 
	usb_rlength = (usb_rlength + 0x1ff) & 0xfffffe00 ; // 512er Boundary 

	return_code = usbTransaction(device,cUsbBuf, usb_wlength,cInPacket, sizeof(cInPacket));
	
	if (return_code == -1) {
	  return sis3153usb_error_code_usb_write_error;
	}
	if (return_code == -2) {
	  return sis3153usb_error_code_usb_read_error;
	}

	nBytes = return_code;	

		
	if(nBytes == 0) {
	  return_code = sis3153usb_error_code_usb_read_length_error;
	  return return_code;
	} 
	
	if (nBytes > 0) {
	  memcpy(data, cInPacket, nBytes);
	}

   	return_code  = 0 ;
	return return_code ;

}

DLLEXPORT int SIS3153_REGISTER_WRITE(libusb_device_handle* device, uint32_t addr, uint32_t *data){

  int return_code ;
  unsigned int nBytes  = 0;
  
  int req_nof_lwords=1;
  unsigned int usb_wlength;
  unsigned int usb_rlength;

  unsigned char cUsbBuf[0x100+USB_MAX_NOF_BYTES];
  unsigned char cInPacket[0x100 + USB_MAX_NOF_BYTES];

  if(req_nof_lwords > USB_MAX_NOF_LWORDS) {
     return_code = sis3153usb_error_code_invalid_parameter ;
     return  return_code ;
  }

	cUsbBuf[0]  =   (char)  0x00 ;	   // header 7:0 	  ; :
	cUsbBuf[1]  =   (char)  0x18 ;	   // header 15:8 	   Bit0 = 11 : not Write
	cUsbBuf[2]  =   (char)  0xaa ;	   // header 23:16
	cUsbBuf[3]  =   (char)  0xaa ;	   // header 31:24
	
	cUsbBuf[4]  =   (char)  req_nof_lwords   ;       //length 7:0
	cUsbBuf[5]  =   (char) (req_nof_lwords >> 8);    //length 15:8
	cUsbBuf[6]  =   (char)  0x0 ;   
	cUsbBuf[7]  =   (char)  0x0 ;
         
	cUsbBuf[8]  =   (char)  addr   ;       //addr 7:0
	cUsbBuf[9]  =   (char) (addr >> 8);    //addr 15:8
	cUsbBuf[10] =   (char) (addr >> 16) ;  //addr 23:16 
	cUsbBuf[11] =   (char) (addr >> 24);   //addr 31:24

	for(int i=0;i<req_nof_lwords;i++) {
		cUsbBuf[(i*4)+12] =   (char)  data[i]   ;       //data 7:0
		cUsbBuf[(i*4)+13] =   (char) (data[i] >> 8);    //data 15:8
		cUsbBuf[(i*4)+14] =   (char) (data[i] >> 16) ;  //data 23:16 
		cUsbBuf[(i*4)+15] =   (char) (data[i] >> 24);   //data 31:24
	}

	usb_wlength = 12 + (req_nof_lwords * 4);

	usb_rlength = (req_nof_lwords * 4) ; // data: (req_nof_lwords * 4) Bytes; 
	usb_rlength = (usb_rlength + 0x1ff) & 0xfffffe00 ; // 512er Boundary 

	return_code = usbTransaction(device,cUsbBuf, usb_wlength,cInPacket, sizeof(cInPacket));
	
	if (return_code == -1) {
	  return sis3153usb_error_code_usb_write_error;
	}
	if (return_code == -2) {
	  return sis3153usb_error_code_usb_read_error;
	}

	nBytes = return_code;	/* That's how usbTransaction works. */

	if(nBytes == 0) {
		return_code = 0;
	}else{
		return_code = sis3153usb_error_code_usb_read_length_error;
	}
	
	return return_code ;

}


DLLEXPORT int SIS3153_VME_READ(libusb_device_handle* device, uint32_t addr, uint32_t vme_am, uint32_t vme_size, uint32_t fifo_mode,
			       uint32_t* data, uint32_t req_nof_bytes, uint32_t* got_nof_bytes)
{
  int return_code ;
  unsigned int nBytes  = 0;
      
  uint32_t usb_wlength;
  uint32_t usb_rlength;
  
  unsigned char cUsbBuf[0x100 + USB_MAX_NOF_BYTES];
  unsigned char cInPacket[0x100 + USB_MAX_NOF_BYTES];
  
  char cSize, cFifoMode;
  char* cUsbBuf_ptr;
  
  cSize = 0x2 ; // Default 4 Byte
  switch (vme_size) {
	case 1:	   // 1 Byte
   		cSize = 0x0 ; //  1 Byte
		break;
	case 2:	   // 2 Bytes
   		cSize = 0x1 ; //  2 Bytes
		break;
	case 4:	   // 4 Bytes
   		cSize = 0x2 ; //  4 Bytes
		break;
	case 8:	   // 8 Bytes
   		cSize = 0x3 ; //  8 Bytes
		break;
  }

  if (fifo_mode == 0) {
	 cFifoMode  = 0x0 ;
  }
   else {
	 cFifoMode  = 0x4 ;
  } 

  cUsbBuf_ptr = (char*) data ;


  if(req_nof_bytes > USB_MAX_NOF_BYTES) {
     return_code = sis3153usb_error_code_invalid_parameter ;
     return return_code;
  }


	cUsbBuf[0]  =   (char)  0x00 ;	                 	// header 7:0 	  ; :
	cUsbBuf[1]  =   (char)  (0x40 + cSize + cFifoMode);	// header 15:8 	   Bit0 = 11 : not Write	/ D32
	cUsbBuf[2]  =   (char)  0xaa ;	           			// header 23:16
	cUsbBuf[3]  =   (char)  0xaa ;	           			// header 31:24
	
	cUsbBuf[4]  =   (char)  req_nof_bytes   ;       //length 7:0
	cUsbBuf[5]  =   (char) (req_nof_bytes >> 8);    //length 15:8
	cUsbBuf[6]  =   (char)  vme_am ;   
	cUsbBuf[7]  =   (char) (vme_am >> 8);
        
	cUsbBuf[8]  =   (char)  addr   ;       //addr 7:0
	cUsbBuf[9]  =   (char) (addr >> 8);    //addr 15:8
	cUsbBuf[10] =   (char) (addr >> 16) ;  //addr 23:16 
	cUsbBuf[11] =   (char) (addr >> 24);   //addr 31:24


	usb_wlength = 12;
	usb_rlength = (req_nof_bytes) ; // data: (req_nof_lwords * 4) Bytes; 
	usb_rlength = (usb_rlength + 0x1ff) & 0xffffe00; // 512 byte boundary.


	return_code = usbTransaction(device,
				     cUsbBuf, usb_wlength,
				     cInPacket, usb_rlength);
	if (return_code == -1) {
	  return sis3153usb_error_code_usb_write_error;
	}
	if (return_code == -2) {
	  return sis3153usb_error_code_usb_read_error;
	}

	nBytes = return_code;
	*got_nof_bytes = (uint32_t) (nBytes )  ;
			
	if(nBytes != req_nof_bytes) {
		 return_code = sis3153usb_error_code_usb_read_length_error; 
		 return return_code;
	}
	
	if(nBytes > 0) {
	  memcpy(cUsbBuf_ptr, cInPacket, nBytes);
	}

	return_code  = 0 ;
	return return_code ;
}






DLLEXPORT int SIS3153_VME_DMA_READ(libusb_device_handle* device, uint32_t addr, uint32_t vme_am, uint32_t vme_size, uint32_t fifo_mode,uint32_t* data, uint32_t req_nof_bytes, uint32_t* got_nof_bytes){

	int return_code ;
  	uint32_t reg_data;

	int error ;
	uint32_t new_addr ; 
	unsigned int index_num_data ; 
	uint32_t rest_req_nof_bytes ; 
	uint32_t new_req_nof_bytes ; 
	uint32_t new_got_nof_bytes ; 


	error = 0x0 ;
	*got_nof_bytes = 0x0 ;
	if (req_nof_bytes == 0) return error ;

	new_addr = addr ;
	rest_req_nof_bytes = req_nof_bytes ;
	index_num_data = 0x0 ;

	do {
		if (rest_req_nof_bytes >= USB_MAX_NOF_BYTES) {
			new_req_nof_bytes = USB_MAX_NOF_BYTES ;
		}
		else {
			new_req_nof_bytes = rest_req_nof_bytes ;
		}
		
	  	error = SIS3153_VME_READ(device, new_addr, vme_am, vme_size, fifo_mode,
	  	                            &data[index_num_data], new_req_nof_bytes, &new_got_nof_bytes);

		index_num_data = index_num_data + (new_got_nof_bytes >> 2) ;
		rest_req_nof_bytes = rest_req_nof_bytes - new_got_nof_bytes ; 

		if(!fifo_mode) new_addr = new_addr + (new_got_nof_bytes ) ; 	

	} while ((error == 0) && (rest_req_nof_bytes>0)) ;
	*got_nof_bytes = index_num_data*4 ;

  return_code = error ;
  if (return_code == 0x0) {
  	return return_code ;
  }
  if (return_code == sis3153usb_error_code_usb_read_length_error) {
	
	 SIS3153_REGISTER_READ(device, 0x11, &reg_data)  ;
	 return_code = ((reg_data >> 16) & 0xffff) ;
  
  	return return_code ;
  }	
	return error;

}


DLLEXPORT int SIS3153_VME_D32_READ(libusb_device_handle* device, uint32_t addr, uint32_t vme_am,uint32_t* data, uint32_t req_nof_words){
	
	uint32_t req_nof_bytes=req_nof_words*4;
	uint32_t got_nof_bytes=0;
	
	int ret=SIS3153_VME_DMA_READ(device,addr,vme_am,4,0,data,req_nof_bytes,&got_nof_bytes);
	
	if((ret==0)&&(got_nof_bytes==req_nof_bytes)){
		return 0;
	}
	
	return ret;
	
}


DLLEXPORT int SIS3153_VME_MLT64_READ(libusb_device_handle* device, uint32_t addr,uint32_t* data, uint32_t req_nof_words){

	uint32_t got_nof_bytes=0;
	uint32_t req_nof_bytes=(req_nof_words&0xfffffffe)*4;

	int ret=SIS3153_VME_DMA_READ(device,addr,0x8,8,0,data,req_nof_bytes,&got_nof_bytes);
	if((ret==0)&&(got_nof_bytes==req_nof_bytes)){
		return 0;
	}

	return ret;

}


DLLEXPORT int SIS3153_VME_2ESST_READ(libusb_device_handle* device, uint32_t addr,uint32_t vme_am,uint32_t* data, uint32_t req_nof_words){

	uint32_t got_nof_bytes=0;
	uint32_t req_nof_bytes=(req_nof_words&0xfffffffe)*4;

	int ret=SIS3153_VME_DMA_READ(device,addr,vme_am,8,0,data,req_nof_bytes,&got_nof_bytes);
	if((ret==0)&&(got_nof_bytes==req_nof_bytes)){
		return 0;
	}

	return ret;


}



DLLEXPORT int SIS3153_VME_WRITE(libusb_device_handle* device, uint32_t addr, uint32_t vme_am, uint32_t vme_size, uint32_t fifo_mode,uint32_t* data, uint32_t req_nof_bytes, uint32_t* put_nof_bytes){
	
int return_code ;
  unsigned int nBytes  = 0;
    
  unsigned int usb_wlength;
  unsigned int usb_rlength;
  unsigned char cUsbBuf[0x100 + USB_MAX_NOF_BYTES];
  unsigned char cInPacket[0x100 + USB_MAX_NOF_BYTES];
  uint32_t data_copy_nof_words =(req_nof_bytes>>2); 
  uint32_t data_byte3, data_byte2, data_byte1, data_byte0 ;
  
  char cSize, cFifoMode;
    
  cSize = 0x2 ; // Default 4 Byte
  switch (vme_size) {
	case 1:	   // 1 Byte
   		cSize = 0x0 ; //  1 Byte
		break;
	case 2:	   // 2 Bytes
   		cSize = 0x1 ; //  2 Bytes
		break;
	case 4:	   // 4 Bytes
   		cSize = 0x2 ; //  4 Bytes
		break;
	case 8:	   // 8 Bytes
   		cSize = 0x3 ; //  8 Bytes
		break;
  }

  if (fifo_mode == 0) {
	 cFifoMode  = 0x0 ;
  }
   else {
	 cFifoMode  = 0x4 ;
  } 
  
  if(req_nof_bytes > USB_MAX_NOF_BYTES) {
     return_code = sis3153usb_error_code_invalid_parameter ;
     return return_code;
  }


	cUsbBuf[0]  =   (char)  0x00 ;	                 	// header 7:0 	  ; :
	cUsbBuf[1]  =   (char)  (0x48 + cSize + cFifoMode);	// header 15:8 	   Bit0 = 11 : not Write	/ D32
	cUsbBuf[2]  =   (char)  0xaa ;	           			// header 23:16
	cUsbBuf[3]  =   (char)  0xaa ;	           			// header 31:24
	
	cUsbBuf[4]  =   (char)  req_nof_bytes   ;       //length 7:0
	cUsbBuf[5]  =   (char) (req_nof_bytes >> 8);    //length 15:8
	cUsbBuf[6]  =   (char)  vme_am ;   
	cUsbBuf[7]  =   (char) (vme_am >> 8);
        
	cUsbBuf[8]  =   (char)  addr   ;       //addr 7:0
	cUsbBuf[9]  =   (char) (addr >> 8);    //addr 15:8
	cUsbBuf[10] =   (char) (addr >> 16) ;  //addr 23:16 
	cUsbBuf[11] =   (char) (addr >> 24);   //addr 31:24

	uint32_t *long_cUsbBuf_ptr =  (uint32_t*) (cUsbBuf + 12);
	for(unsigned int i=0;i<data_copy_nof_words;i++) {
		*long_cUsbBuf_ptr++ =   data[i] ;
	}

	usb_wlength = 12+ req_nof_bytes;
	usb_rlength = 4 ;  
	usb_rlength = (usb_rlength + 0x1ff) & 0xffffe00; // 512 byte boundary.

	return_code = usbTransaction(device,
				     cUsbBuf, usb_wlength,
				     cInPacket, usb_rlength);
	if (return_code == -1) {
	  return sis3153usb_error_code_usb_write_error;
	}
	if (return_code == -2) {
	  return sis3153usb_error_code_usb_read_error;
	}

	nBytes = return_code;
	
	if(nBytes == 0) {
		*put_nof_bytes = req_nof_bytes  ;
		return_code = 0;
		return return_code;
	}
	
	if(nBytes == 4) { // VME Error
		data_byte0 =  (uint32_t) (cInPacket[0] & 0xff) ;
		data_byte1 =  (uint32_t) (cInPacket[1] & 0xff) ;
		data_byte2 =  (uint32_t) (cInPacket[2] & 0xff) ;
		data_byte3 =  (uint32_t) (cInPacket[3] & 0xff) ;
		*put_nof_bytes =   (data_byte1 << 8)  + (data_byte0 )   ;
		return_code    =   (data_byte3 << 8)  + (data_byte2 ) ;
		return return_code;
	}
	 
	return_code = sis3153usb_error_code_usb_read_length_error;
	return return_code;
	
	
	
}


DLLEXPORT int SIS3153_VME_D32_WRITE(libusb_device_handle* device, uint32_t addr, uint32_t vme_am,uint32_t* data, uint32_t req_nof_words){
	
	uint32_t req_nof_bytes=req_nof_words*4;
	uint32_t put_nof_bytes=0;
	int ret=SIS3153_VME_WRITE(device,addr,vme_am,4,0,data,req_nof_bytes, &put_nof_bytes);
	if((ret==0)&&(put_nof_bytes==req_nof_bytes)){
		return 0;
	}
	
	return ret;
	
	
}

//Kmax native driver functions 
 
 DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativeInit(JNIEnv *env, jobject ob){
	  
	uint32_t SIS3153_DATA=0x0;
	
	if(SIS3153_Device_open){
		 if(SIS3153_REGISTER_READ(dev_handle,0x1,&SIS3153_DATA)==0){ //reading from the module firmware register 
			return 0; //initialization successful
		 }

		SIS3153_CLOSE_DEVICE(); // Device handle no longer valid, close device & libusb !
			 
	}
	
	int ret=SIS3153_OPEN_DEVICE();
	if(ret!=0){
		return ret;
	}
			
	SIS3153_Device_open=true;
	return 0;

}
 
 
DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativeClose(JNIEnv *env, jobject ob){
	
	int ret=SIS3153_CLOSE_DEVICE();
	SIS3153_Device_open=false;
	return ret;
}

DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativeReadInt(JNIEnv *env, jobject ob, jlong addr, jint func, jintArray buf, jint offset, jint count){

	uint32_t VMEAddr=(addr&0xFFFFFFFF);
	uint32_t data[count]={0};
	int ret=-1;
			
	if(func==0x1000){
		ret=SIS3153_REGISTER_READ(dev_handle,VMEAddr,data);
		
	}else if(func==0x8){

		ret=SIS3153_VME_MLT64_READ(dev_handle,VMEAddr,data,count);
		

	}else if((func==0x260)||(func==0x160)||(func==0x60)){

		ret=SIS3153_VME_2ESST_READ(dev_handle,VMEAddr,func,data,count);

	}
	else{
		
		ret=SIS3153_VME_D32_READ(dev_handle,VMEAddr,func,data,count);
			
	}

	//Pointers to the java VM arrays
	jboolean isCopy = JNI_FALSE;
	jint *pAry = (jint *)env->GetPrimitiveArrayCritical(buf, &isCopy);
	if (pAry == NULL) {
		return -1001;
	}	
	jint *pData = pAry + offset; // ! pointer arithmetic
	
	for(int i=0;i<count;++i){
		pData[i]=(data[i]&0xFFFFFFFF);
	}
	
	env->ReleasePrimitiveArrayCritical(buf, pAry, 0);
	return ret;
	
}

DLLEXPORT JNIEXPORT jint JNICALL Java_SIS3153_nativewriteInt(JNIEnv *env, jobject ob, jlong addr, jint func, jintArray buf, jint offset, jint count){
	uint32_t VMEAddr=(addr&0xFFFFFFFF);
	uint32_t data[count]={0};
	int ret=-1;

	//Pointers to the java VM arrays
	jboolean isCopy = JNI_FALSE;
	jint *pAry = (jint *)env->GetPrimitiveArrayCritical(buf, &isCopy);
	if (pAry == NULL) {
		return -1001;
	}	
	jint *pData = pAry + offset; // ! pointer arithmetic
	
	if(func==0x1000){
		data[0]=pData[0];
		ret=SIS3153_REGISTER_WRITE(dev_handle,VMEAddr,data);
		
	}else{
		
		for(int i=0;i<count;i++){
			data[i]=pData[i];
		}	
		ret=SIS3153_VME_D32_WRITE(dev_handle,VMEAddr,func,data,count);
	}
	
	env->ReleasePrimitiveArrayCritical(buf, pAry, 0);
	return ret;
	
}


