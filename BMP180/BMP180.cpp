///////////////////////////////////////////////////////////////////
// BMP180.cpp
//
// Class file for the BMP180 Barometric Pressure Sensor Arduino Library.
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

#include <Arduino.h>
#include <BMP180.h>


///////////////////////////////////////////////////////////////////
//
// IsConnected()
//
// NOTE not compatible with error codes
//
// returns: true if BMP180/BMP085 is connected otherwise false if not
//
///////////////////////////////////////////////////////////////////

bool BMP180::IsConnected(void) {
	return I2C::IsConnected();
}

///////////////////////////////////////////////////////////////////
//
//	Initialize(void)
//
//	copies calibration data from BMP180 device to the BMP180 class
//
//	Note this is ripe for reading directly into a structure.
//
//	returns: true is successful, false otherwise
//
///////////////////////////////////////////////////////////////////

bool BMP180::Initialize(void)
{
	bool bResult = false;
	if( IsConnected()) {

		SoftReset();

		uint8_t* pBuffer = new uint8_t[(BMP180_CER - BMP180_CSR) + 1];

		if( pBuffer ) {
			bResult = Read(BMP180_CSR, BMP180_CER - BMP180_CSR + 1, pBuffer);
			if(bResult) {
				// This data is in Big Endian format from the BMP180.
				m_AC1 = (pBuffer[0]  << 8) | pBuffer[1];
				m_AC2 = (pBuffer[2]  << 8) | pBuffer[3];
				m_AC3 = (pBuffer[4]  << 8) | pBuffer[5];
				m_AC4 = (pBuffer[6]  << 8) | pBuffer[7];
				m_AC5 = (pBuffer[8]  << 8) | pBuffer[9];
				m_AC6 = (pBuffer[10] << 8) | pBuffer[11];
				m_B1  = (pBuffer[12] << 8) | pBuffer[13];
				m_B2  = (pBuffer[14] << 8) | pBuffer[15];
				m_MB  = (pBuffer[16] << 8) | pBuffer[17];
				m_MC  = (pBuffer[18] << 8) | pBuffer[19];
				m_MD  = (pBuffer[20] << 8) | pBuffer[21];
			} else {
				Serial.println("\nBMP180::Initialize() Read error" );
			}
			if (pBuffer) {
				delete[] pBuffer ;
			}
		} else {
			Serial.println("\nBMP180::Initialize() Memory allocation error" );
		}
	} 
	return bResult;
}

///////////////////////////////////////////////////////////////////
//
// GetAltitude(BMP180_HEIGHT_t bUnits)
//
// calculates the altitude from fPressure and lSeaLevelPressure in meters
// or feet depending on iUnits.
//
// returns: if iUnits == BMP180_METERS the altitude in Meters
//          if iUnits == BMP180_FEET the altitude in feet
//
///////////////////////////////////////////////////////////////////

float BMP180::GetAltitude(BMP180_HEIGHT_t bUnits)
{
	// Get pressure in Pascals (Pa).
	float fPressure = GetPressure();
	// Calculate altitude from sea level.
	float fAltitude = 44330.0 * (1.0 - pow( fPressure / m_lSeaLevelPressure, 0.1902949571836346));
	return (bUnits) ? ( fAltitude * 3.2808399 ) : fAltitude ;
}

///////////////////////////////////////////////////////////////////
//
// GetAltitude(long lCurrentSeaLevelPressureInPa, BMP180_HEIGHT_t bUnits)
//
// calculates the altitude from fPressure and lCurrentSeaLevelPressureInPa 
// in meters or feet depending on iUnits.
//
// returns: if iUnits == BMP180_METERS the altitude in Meters
//          if iUnits == BMP180_FEET the altitude in feet
//
///////////////////////////////////////////////////////////////////

float BMP180::GetAltitude(long lCurrentSeaLevelPressureInPa, BMP180_HEIGHT_t bUnits)
{
	// Get pressure in Pascals (Pa).
	float fPressure = GetPressure();
	// Calculate altitude from sea level.
	float fAltitude = 44330.0 * (1.0 - pow( fPressure / lCurrentSeaLevelPressureInPa, 0.1902949571836346));
	return (bUnits) ? ( fAltitude * 3.2808399 ) : fAltitude ;
}


///////////////////////////////////////////////////////////////////
//
// GetErrorText(BMP180_ERRORS_t iErrorCode )
//
// displays error messages depending on iErrorCode 
//
// returns: char* to error message
//
///////////////////////////////////////////////////////////////////

char* BMP180::GetErrorText(int iErrorCode )
{
	switch(iErrorCode  ) {
		case BMP180_ERROR_NONE:
			return "No Error found" ;
			break;
		case BMP180_ERROR_READ_ERROR:
			return "BMP180 Read Error" ;
			break;
		case BMP180_ERROR_WRITE_ERROR:
			return "BMP180 Write Error" ;
			break;
		case BMP180_ERROR_NOT_CONNECTED:
			return "BMP180 Not Connected" ;
			break;
		case BMP180_ERROR_SAMPLE_MODE_NOT_VALID:
			return "BMP180 Sample Mode was not valid, valid values are: ultra low power = 0, standard = 1, high resolution = 2, ultra high resolution = 3" ;
			break;
		default:
			return "Error not defined.";
			break;
	}
}

///////////////////////////////////////////////////////////////////
//
// GetPressure(void)
//
// gets the actual Pressure.
//
// returns: the actual Pressure value in pascals
//
///////////////////////////////////////////////////////////////////

long BMP180::GetPressure(void)
{
	return CompensatePressure(GetUCPressure());
}


///////////////////////////////////////////////////////////////////
//
// GetTemperature(BMP180_TEMPERATURE_t iUnits)
//
// gets the actual temperature.
//
// returns: if iUnits == BMP180_CENTIGRADE the temperature in degrees C
//          if iUnits == BMP180_FAHRENHEIT the temperature in degrees F
//
///////////////////////////////////////////////////////////////////

float BMP180::GetTemperature(BMP180_TEMPERATURE_t bUnits)
{
	return (bUnits) ? (((CompensateTemperature(GetUCTemperature())*9)/5)+32) : CompensateTemperature(GetUCTemperature());
}

///////////////////////////////////////////////////////////////////
//
// SetResolution(BMP180_MODES_t iNewSampleMode )
//
// Sets the wait time and the number of samples depending on the
// sample mode. Note advanced mode is not supported. 
//
// returns: BMP180_ERROR_NONE if sample mode is OK otherwise 
//          BMP180_ERROR_SAMPLE_MODE_NOT_VALID if not
//
///////////////////////////////////////////////////////////////////

BMP180_ERROR_t BMP180::SetResolution(BMP180_MODE_t iNewSampleMode )
{
	switch (iNewSampleMode ) {
		case BMP180_MODE_ULTRA_LOW_POWER:
			m_iWaitTime = BMP180_WAIT_5MS ;
			m_iSamples = BMP180_SAMPLES_1 ;
			break;
        case BMP180_MODE_STANDARD:
			m_iWaitTime = BMP180_WAIT_8MS ;
			m_iSamples = BMP180_SAMPLES_2 ;
			break;
        case BMP180_MODE_HIGH_RESOLUTION:
			m_iWaitTime = BMP180_WAIT_14MS ;
			m_iSamples = BMP180_SAMPLES_4 ;
			break;
		case BMP180_MODE_ULTRA_HIGH_RESOLUTION:
			m_iWaitTime = BMP180_WAIT_26MS ;
			m_iSamples = BMP180_SAMPLES_8 ;
			break;
		case BMP180_MODE_ADVANCED_RESOLUTION_MODE:
			m_iWaitTime = BMP180_WAIT_77MS ;
			m_iSamples = BMP180_SAMPLES_16 ;
			break;
		default:
			return BMP180_ERROR_SAMPLE_MODE_NOT_VALID ;
	}
	// if we get here sampleResolution is OK

    	m_iSampleMode = iNewSampleMode;			// save iNewSampleMode
	return BMP180_ERROR_NONE;			// success
}

///////////////////////////////////////////////////////////////////
//
// SetSeaLevelPressure( long lSeaLevelPressure )
//
// Sets m_lSeaLevelPressure to lNewSeaLevelPressure
//
// returns nothing
//
///////////////////////////////////////////////////////////////////

void BMP180::SetSeaLevelPressure( long lSeaLevelPressure )
{
	m_lSeaLevelPressure = lSeaLevelPressure ;
}

///////////////////////////////////////////////////////////////////
//
// SetSeaLevelPressure( float fAltitude, BMP180_HEIGHT_t bUnits)
//
// Sets lSeaLevelPressure to the value calculated from fAltitude
// in meters and current atmospheric pressure in pascals
//
// returns lSeaLevelPressure 
//
///////////////////////////////////////////////////////////////////

long BMP180::SetSeaLevelPressure( float fAltitude, BMP180_HEIGHT_t bUnits )
{
	long lPressure = GetPressure();					// get the current pressure
	fAltitude = (bUnits) ? ( fAltitude / 3.2808399 ) : fAltitude ;		// if in feet convert to meters
	m_lSeaLevelPressure = lPressure / pow( ( 1.0 - ( fAltitude / 44330.0 )), 5.25) ;
	return m_lSeaLevelPressure ;
}

///////////////////////////////////////////////////////////////////
//
// SetSeaLevelPressure( float fAltitude, long lPressure, BMP180_HEIGHT_t bUnits )
//
// Sets lSeaLevelPressure to the value calculated from fAltitude
// in meters and lPressure in pascals
//
// returns lSeaLevelPressure 
//
///////////////////////////////////////////////////////////////////

long BMP180::SetSeaLevelPressure( float fAltitude, long lPressure, BMP180_HEIGHT_t bUnits )
{
	fAltitude = (bUnits) ? ( fAltitude / 3.2808399 ) : fAltitude ;		// if in feet convert to meters
	m_lSeaLevelPressure = lPressure / pow( ( 1 - ( fAltitude / 44330 )), 5.25) ;
	return m_lSeaLevelPressure ;
}

///////////////////////////////////////////////////////////////////
//
// protected functions
//
///////////////////////////////////////////////////////////////////
//
// CompensateTemperature(int iUT)
//
// gets the actual temperature from the uncompensated temperature lUT.
//
// returns: the actual temperature value
//
///////////////////////////////////////////////////////////////////

float BMP180::CompensateTemperature(int iUT)
{
	long x1 = (((long)iUT - (long)m_AC6) * (long)m_AC5) >> 15;
	long x2 = ((long)m_MC << 11) / (x1 + m_MD);
	m_B5 = x1 + x2;
	int iTemperature = (int)((m_B5 + 8) >> 4);  /* temperature in 0.1 deg C*/
	float fTemperature = iTemperature;
	fTemperature /= 10.0;

	// Record this data because it is required by the pressure algorithem.
	m_LastTemperatureTime = millis();

	return fTemperature;
}

///////////////////////////////////////////////////////////////////
//
// CompensatePressure(long lUP)
//
// gets the actual Pressure from the uncompensated pressure lUP.
//
// returns: the actual Pressure value
//
///////////////////////////////////////////////////////////////////

long BMP180::CompensatePressure(long lUP)
{
	int msSinceLastReading = millis() - m_LastTemperatureTime;
	// Check to see if we have old temperature data.

	if (msSinceLastReading > m_LatencyForPressure) {
		GetTemperature(BMP180_CENTIGRADE); // Refresh the temperature.
	}
	// Algorithm taken from BMP180 datasheet.
	long B6 = m_B5 - 4000;
	long X1 = (m_B2 * (B6 * B6 >> 12)) >> 11;
	long X2 = m_AC2 * B6 >> 11;
	long X3 = X1 + X2;
	long B3 = ((m_AC1 * 4 + X3) << m_iSampleMode) + 2;
	B3 = B3 >> 2;
	X1 = m_AC3 * B6 >> 13;
	X2 = (m_B1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	unsigned long B4 = m_AC4 * (X3 + 32768) >> 15;
	unsigned long B7 = (((lUP - B3)) * (50000 >> m_iSampleMode));
	long lPressure = ( B7 < 0x80000000 ) ? ((B7 * 2) / B4) : ((B7 / B4) * 2) ;
	X1 = (lPressure >> 8) * (lPressure >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * lPressure) >> 16;
	lPressure = lPressure + ((X1 + X2 + 3791) >> 4);

	return lPressure;
}

///////////////////////////////////////////////////////////////////
//
//	GetUCTemperature(void)
//
//	gets the raw temperature data from the registers.
//
//	returns: the raw temperature value
//
///////////////////////////////////////////////////////////////////

int BMP180::GetUCTemperature(void)
{
	// Instruct device to perform a conversion.
	Write(BMP180_MEASUREMENT_CONTROL_REGISTER, BMP180_MEASURE_TEMPERATURE);
	// Wait for the conversion to complete.
	delay(5);
	uint16_t uiData ;
	ReadWORD(BMP180_OutMSB, uiData );
	return (int) uiData;
}

///////////////////////////////////////////////////////////////////
//
//	GetUCPressure(void)
//
//	gets the raw pressure data from the registers.
//
//	returns: the raw Pressure value
//
///////////////////////////////////////////////////////////////////

long BMP180::GetUCPressure(void)
{
	long pressure = 0;

	// Instruct device to perform a conversion, including the oversampling data.
	uint8_t CtrlByte = BMP180_MEASURE_PRESSURE + (m_iSampleMode << 6);

	Write(BMP180_MEASUREMENT_CONTROL_REGISTER, CtrlByte);
		
	// Wait for the conversion
	delay(m_iWaitTime);		

	uint8_t *pBuffer = new uint8_t[3];
	// Read the conversion data into buffer.
	Read(BMP180_OutMSB, 3, pBuffer);

	// Collect the data (and push back the LSB if we are not sampling them).
	pressure = ((((long)pBuffer[0] <<16) | ((long)pBuffer[1] <<8) | ((long)pBuffer[2])) >> (8-m_iSampleMode));
	if(pBuffer) {
		delete[] pBuffer ;
	}
	return pressure ;
}

///////////////////////////////////////////////////////////////////
//
// SoftReset(void)
//
// forces the BMP180 device to do a soft reset
//
// Returns Nothing
//
///////////////////////////////////////////////////////////////////

void BMP180::SoftReset(void) {
	Write(BMP180_SOFT_RESET, BMP180_SOFT_RESET_INSTRUCTION);
	delay(100);
}

///////////////////////////////////////////// end of BMP180.cpp ////////////////////