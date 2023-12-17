/*
  ******************************************************************************
  * @file 			( фаил ):   			GC9A01.h
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	 		author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
 */
 
 
#ifndef _GC9A01_H
#define _GC9A01_H


/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

// SPI включаем только передача ( MOSI SCK )
// настройка SPI, скорость максимальная, 1 Line (TX only), Polarity High, Phase 2Edge:

#include "main.h"
#include "fonts.h"

#include "stdlib.h"
#include "string.h"
#include "math.h"
	
	


//#######  SETUP  ##############################################################################################
		
		//==== выбераем через что будем отправлять через HAL или CMSIS(быстрее) ==================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
			// указываем порт SPI для CMSIS ( быстро )-------
			// так как у разных МК разные регистры то в функциях корректируем под свой МК
			// на данный момент есть реализация на серию F1 F4 H7 для выбора серии в функциях
			//	void GC9A01_SendCmd(uint8_t Cmd);
			//	void GC9A01_SendData(uint8_t Data );
			//	void GC9A01_SendDataMASS(uint8_t* buff, size_t buff_size);	
			// комментируем и раскомментируем то что нам нужно, также там же редактируем под свой МК если не работает
			#define 	GC9A01_SPI_CMSIS 	SPI1
			//-----------------------------------------------
			
			// указываем порт SPI для HAL ( медлено )--------
			//#define 	GC9A01_SPI_HAL 		hspi1
			//-----------------------------------------------
			
			// выбираем как выводить информацию через буфер кадра или попиксельно ( 1-буфер кадра, 0-попиксельный вывод ) -----
			// через буфер быстре если много информации обнавлять за один раз ( требует много оперативки для массива )
			// по пиксельно рисует онлайн буз буферра если информация обновляеться немного то выгодно испотзовать данный режим
			#define FRAME_BUFFER				0
			//-----------------------------------------------------------------------------------------------------------------
			
			
		//============================================================================
		
		//=== указываем порты ( если в кубе назвали их DC RES CS то тогда нечего указывать не нужно )
		#if defined (DC_GPIO_Port)
		#else
			#define DC_GPIO_Port	GPIOC
			#define DC_Pin			GPIO_PIN_5
		#endif
		
		#if defined (RST_GPIO_Port)
		#else
			#define RST_GPIO_Port   GPIOB
			#define RST_Pin			GPIO_PIN_14
		#endif
		
		//--  Cесли используем порт CS для выбора устройства тогда раскомментировать ------------
		// если у нас одно устройство лучше пин CS притянуть к земле( или на порту подать GND )
		
		#define CS_PORT
		
		//----------------------------------------------------------------------------------------
		#ifdef CS_PORT
			#if defined (CS_GPIO_Port)
			#else
				#define CS_GPIO_Port    GPIOA
				#define CS_Pin			GPIO_PIN_4
			#endif
		#endif
		
		//=============================================================================
		
		//==  выбираем дисплей: =======================================================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
		#define	GC9A01_IS_240X240		// 240 x 240 GC9A01 		
		//=============================================================================
		
		
//##############################################################################################################

#ifdef GC9A01_SPI_HAL
	extern SPI_HandleTypeDef GC9A01_SPI_HAL;
#endif

extern uint16_t GC9A01_Width, GC9A01_Height;

extern uint16_t GC9A01_X_Start;
extern uint16_t GC9A01_Y_Start;

#define RGB565(r, g, b)         (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#define PI 	3.14159265

	//--- готовые цвета ------------------------------
	#define GC9A01_BLACK   										0x0000
	#define GC9A01_BLUE    										0x001F
	#define GC9A01_RED     										0xF800
	#define GC9A01_GREEN   										0x07E0
	#define	GC9A01_CYAN    										0x07FF
	#define GC9A01_MAGENTA 										0xF81F
	#define GC9A01_YELLOW  										0xFFE0
	#define GC9A01_WHITE   										0xFFFF
	//------------------------------------------------

	// Команды контроллера дисплея
	#define GC9A01_SLPIN       								0x10
	#define GC9A01_SLPOUT      								0x11
	#define GC9A01_INVOFF      								0x20
	#define GC9A01_INVON       								0x21
	#define GC9A01_DISPOFF     								0x28
	#define GC9A01_DISPON      								0x29
	#define GC9A01_CASET       								0x2A
	#define GC9A01_RASET       								0x2B
	#define GC9A01_RAMWR       								0x2C
	#define GC9A01_TEON      									0x35    // Tearing effect line ON
	#define GC9A01_MADCTL      								0x36    // Memory data access control
	#define GC9A01_COLMOD      								0x3A   	// Pixel format set
	#define GC9A01_RAMWR_CONT       					0x3C

	#define GC9A01_DisplayFunctionControl    	0xB6
	#define GC9A01_PWCTR1       							0xC1    // Power control 1
	#define GC9A01_PWCTR2       							0xC3    // Power control 2
	#define GC9A01_PWCTR3       							0xC4    // Power control 3
	#define GC9A01_PWCTR4       							0xC9    // Power control 4
	#define GC9A01_PWCTR7       							0xA7    // Power control 7
	
	#define GC9A01_FRAMERATE      						0xE8
	#define GC9A01_InnerReg1Enable   					0xFE
	#define GC9A01_InnerReg2Enable   					0xEF

	#define GC9A01_GAMMA1       							0xF0    // Set gamma 1
	#define GC9A01_GAMMA2       							0xF1    // Set gamma 2
	#define GC9A01_GAMMA3       							0xF2    // Set gamma 3
	#define GC9A01_GAMMA4       							0xF3    // Set gamma 4

	//==============================================================================
	// Значения, передаваемые аргументом с командой GC9A01_COLMOD
	#define ColorMode_RGB_16bit  							0x50
	#define ColorMode_RGB_18bit  							0x60
	#define ColorMode_MCU_12bit  							0x03
	#define ColorMode_MCU_16bit  							0x05
	#define ColorMode_MCU_18bit  							0x06

	#define GC9A01_MADCTL_MY        					0x80
	#define GC9A01_MADCTL_MX        					0x40
	#define GC9A01_MADCTL_MV        					0x20
	#define GC9A01_MADCTL_ML        					0x10
	#define GC9A01_MADCTL_BGR       					0x08
	#define GC9A01_MADCTL_MH        					0x04

//-----------------------------------------------

#define DELAY 0x80


//###  параметры дисплея 240 x 240 GC9A01 ###################################

	// 240 x 240 GC9A01  display, default orientation

#ifdef GC9A01_IS_240X240
	
	#define GC9A01_WIDTH  					240
	#define GC9A01_HEIGHT 					240
	#define GC9A01_XSTART 					0												// смещение начала отсчета области
	#define GC9A01_YSTART 					0												// смещение начала отсчета области
	#define GC9A01_DEF_ROTATION			(GC9A01_MADCTL_BGR)			// GC9A01_rotation( 0, 0, 0 )
	
#endif
	
	
//##############################################################################


void GC9A01_Init(void);
void GC9A01_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);	
void GC9A01_HardReset(void);
void GC9A01_SleepModeEnter( void );
void GC9A01_SleepModeExit( void );
void GC9A01_InversionMode(uint8_t Mode);
void GC9A01_FillScreen(uint16_t color);
void GC9A01_Clear(void);
void GC9A01_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void GC9A01_SetBL(uint8_t Value);
void GC9A01_DisplayPower(uint8_t On);
void GC9A01_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void GC9A01_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t fillcolor);
void GC9A01_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void GC9A01_DrawLineWithAngle(int16_t x, int16_t y, uint16_t length, double angle_degrees, uint16_t color);
void GC9A01_DrawLineThick(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t thick);
void GC9A01_DrawLineThickWithAngle(int16_t x, int16_t y, int16_t length, double angle_degrees, uint16_t color, uint8_t thick);
void GC9A01_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void GC9A01_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void GC9A01_DrawPixel(int16_t x, int16_t y, uint16_t color);
void GC9A01_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius, uint16_t fillcolor);
void GC9A01_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
void GC9A01_DrawEllipse(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void GC9A01_DrawEllipseFilled(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void GC9A01_DrawEllipseFilledWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void GC9A01_DrawEllipseWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void GC9A01_DrawChar(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, unsigned char ch);
void GC9A01_DrawCharWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, unsigned char ch);
void GC9A01_print(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, char *str);
void GC9A01_printWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, char *str);
void GC9A01_rotation(uint8_t Rotation, uint8_t VertMirror, uint8_t HorizMirror);
void GC9A01_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);
void GC9A01_DrawBitmapWithAngle(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color, double angle_degrees);
void GC9A01_DrawCircleHelper(int16_t x0, int16_t y0, int16_t radius, int8_t quadrantMask, uint16_t color);
void GC9A01_DrawFillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void GC9A01_DrawFillRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void GC9A01_DrawRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void GC9A01_DrawArc(int16_t x0, int16_t y0, int16_t radius, int16_t startAngle, int16_t endAngle, uint16_t color, uint8_t thick);

#if FRAME_BUFFER
	void GC9A01_Update(void);
	void GC9A01_ClearFrameBuffer(void);
#endif

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif	/*	_GC9A01_H */

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
