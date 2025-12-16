#ifndef _ESP32API_H_
#define _ESP32API_H_

#include <stdint.h>
#include <stddef.h>

typedef void* ESP32File;

enum {
    ESP32_Seek_Set = 0,
    ESP32_Seek_Cur,
    ESP32_Seek_End
};

//void ESP32API_GetDisplayDimensions( int* OutWidthPtr, int* OutHeightPtr );
//void ESP32API_SetAddressWindow( int x0, int y0, int x1, int y1 );
//void ESP32API_WritePixels( const uint16_t* Pixels, size_t Count );

void init_vmacmini_esp32();

void ESP32API_GetMouseDelta( int* OutXDeltaPtr, int* OutYDeltaPtr );
void ESP32API_GiveEmulatedMouseToESP32( int* EmMouseX, int* EmMouseY );
int ESP32API_GetMouseButton( void );

uint64_t ESP32API_GetTimeMS( void );
void ESP32API_Yield( void );
void ESP32API_Delay( uint32_t MSToDelay );

ESP32File ESP32API_open( const char* Path, const char* Mode );
void ESP32API_close( ESP32File Handle );
size_t ESP32API_read( void* Buffer, size_t Size, size_t Nmemb, ESP32File Handle );
size_t ESP32API_write( const void* Buffer, size_t Size, size_t Nmemb, ESP32File Handle );
long ESP32API_tell( ESP32File Handle );
long ESP32API_seek( ESP32File Handle, long Offset, int Whence );
int ESP32API_eof( ESP32File Handle );

void* ESP32API_malloc( size_t Size );
void* ESP32API_calloc( size_t Nmemb, size_t Size );
void ESP32API_free( void* Memory );

void ESP32API_CheckForEvents( void );

void ESP32API_ScreenChanged( int Top, int Left, int Bottom, int Right );
void ESP32API_DrawScreen( const uint8_t* Screen );
void ESP32API_GiveScreenBufferToArduino( const uint8_t* ScreenPtr );

int minivmac_main(int argc, char** argv);

#endif