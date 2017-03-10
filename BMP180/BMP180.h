///////////////////////////////////////////////////////////////////
// BMP180.h 
//
// Header file for the BMP180 Barometric Pressure Sensor Arduino Library.
// Copyright (C) 2013  Suusi Malcolm-Brown M0SUU 
//
// Important this class is derived from the I2C class which handles
// Reads(), Writes() and IsConnected()
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the version 3 GNU General Public License as
// published by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// Datasheet for BMP180:
// http://www.bosch-sensortec.com/content/language1/downloads/BST-BMP180-DS000-07.pdf
//
///////////////////////////////////////////////////////////////////

#ifndef BMP180_h
#define BMP180_h

#include <inttypes.h>
#include <Wire.h>
#include <I2C.h>


#define BMP180_ADDRESS                      (0xEE>>1)
#define BMP180_ID_REG						0xD0
#define BMP180_ID							0x55
#define BMP180_SOFT_RESET					0xE0
#define BMP180_SOFT_RESET_INSTRUCTION       0xB6		// power on reset
#define BMP180_MEASURE_TEMPERATURE          0x2E
#define BMP180_MEASURE_PRESSURE             0x34
#define BMP180_CSR							0xAA
#define BMP180_CER							0xBF
#define BMP180_MEASUREMENT_CONTROL_REGISTER 0xF4
#define BMP180_OutMSB                       0xF6		// a/d converter out MSD
#define BMP180_OutLSB                       0xF7		// a/d converter out LSD
#define BMP180_OutXLSB                      0xF8		// a/d converter out XLSD

enum BMP180_TEMPERATURE_t {	
	BMP180_CENTIGRADE = 0,
	BMP180_FAHRENHEIT 
};

enum BMP180_HEIGHT_t	{	
	BMP180_METERS = 0,
	BMP180_FEET 
};

enum BMP180_SAMPLES_t	{	
	BMP180_SAMPLES_1 = 1,
	BMP180_SAMPLES_2 = 2,
	BMP180_SAMPLES_4 = 4,
	BMP180_SAMPLES_8 = 8,
	BMP180_SAMPLES_16 
} ;

enum BMP180_WAIT_t	{	
	BMP180_WAIT_5MS  =  5,
	BMP180_WAIT_8MS  =  8,
	BMP180_WAIT_14MS = 14,
	BMP180_WAIT_26MS = 26,
	BMP180_WAIT_77MS = 77 
} ;

enum BMP180_MODE_t	{	
	BMP180_MODE_ULTRA_LOW_POWER = 0, 
	BMP180_MODE_STANDARD, 
	BMP180_MODE_HIGH_RESOLUTION, 
	BMP180_MODE_ULTRA_HIGH_RESOLUTION,
	BMP180_MODE_ADVANCED_RESOLUTION_MODE 
};

enum BMP180_ERROR_t	{	
	BMP180_ERROR_NONE = 0, 
	BMP180_ERROR_READ_ERROR,	
	BMP180_ERROR_WRITE_ERROR,
	BMP180_ERROR_NOT_CONNECTED, 
	BMP180_ERROR_SAMPLE_MODE_NOT_VALID
} ;


class BMP180 : public I2C {
	public:
							BMP180() : I2C(BMP180_ADDRESS, BMP180_ID_REG, BMP180_ID){				// constructor
			 					m_iSampleMode			= BMP180_MODE_ULTRA_LOW_POWER;				// sample mode (OSS from data sheet)
								m_iSamples				= BMP180_SAMPLES_1;							// number of samples taken depending on iSampleMode
								m_iWaitTime				= BMP180_WAIT_5MS;							// conversion time taken depending on iSampleMode
								m_LastTemperatureTime	= -1000;
								m_B5					= 0;
								m_LatencyForPressure	= 1000;
								m_lSeaLevelPressure		= 101325;									// set to 1013.25 milli Barr as default
							}

		bool				IsConnected(void);
		bool				Initialize(void);

		float				GetAltitude( BMP180_HEIGHT_t iUnits = BMP180_METERS );
		float				GetAltitude( long lCurrentSeaLevelPressure, BMP180_HEIGHT_t iUnits = BMP180_METERS );
		char*				GetErrorText( int iErrorCode );
		long				GetPressure(void);
		long				GetSeaLevelPressure(void) { return m_lSeaLevelPressure; }
		float				GetTemperature( BMP180_TEMPERATURE_t iUnits = BMP180_CENTIGRADE );

		BMP180_ERROR_t		SetResolution( BMP180_MODE_t iNewOverSample );
		void				SetSeaLevelPressure( long lNewSeaLevelPressure );
		long				SetSeaLevelPressure( float fAltitude, BMP180_HEIGHT_t bUnits = BMP180_METERS ) ;
		long				SetSeaLevelPressure( float fAltitude, long lPressure, BMP180_HEIGHT_t bUnits = BMP180_METERS ) ;

	protected:
		float				CompensateTemperature(int iUT);
		long				CompensatePressure(long iUP);
		int					GetUCTemperature(void);
		long				GetUCPressure(void);
		void				SoftReset(void);

	private:
		long  				m_lSeaLevelPressure ;			// stores the current sea level pressure in Pa (default 101325 pa)
		BMP180_MODE_t 		m_iSampleMode;					// sample mode (OSS from data sheet)
		BMP180_SAMPLES_t	m_iSamples;						// number of samples taken depending on iSampleMode
		BMP180_WAIT_t		m_iWaitTime;					// conversion time taken depending on iSampleMode
		long				m_B5;							// shared data for both temperature and pressure
		int					m_LastTemperatureTime;
		int					m_LatencyForPressure;
		BMP180_ERROR_t		m_iError;						// last error		

		int					m_AC1;
        int					m_AC2;
        int					m_AC3;
        uint16_t			m_AC4;
        uint16_t			m_AC5;
        uint16_t			m_AC6;
        int					m_B1;
        int					m_B2;
        int					m_MB;
        int					m_MC;
        int					m_MD;

};
#endif