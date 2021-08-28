//	Sample Creation routines (encode and filter) for ARDOP Modem

#include "ARDOPC.h"

extern int intSessionBW;	// Negotiated speed

#pragma warning(disable : 4244)		// Code does lots of  float to int

FILE * fp1;

float dblQAMCarRatio = 1.0f / 1.765f;   //Optimum for 8,8 circular constellation with no phase offset: (Dmin inner to inner = Dmin inner to outer) 

extern int intAmp;		// Selected to have some margin in calculations with 16 bit values (< 32767) this must apply to all filters as well. 

extern unsigned char encstate;

unsigned char encstates[10] = {0};

#define MAX(x, y) ((x) > (y) ? (x) : (y))

// Function to generate the Two-tone leader and Frame Sync (used in all frame types) 

extern short Dummy;

int intSoftClipCnt = 0;
float PeakValue = 0.0f;
extern float dblEx;

BOOL SendingHeader200 = 0;		// Set when sending header in 200 Hz Modes

void Flush();
int Get4PSKSym(UCHAR * bytSymbols28_228);
int Get16APSKSym_8_8(UCHAR * Sym, float * dblMag);
void DrawTXFrame(const char * Frame);

// Square Law compressor used on modulated samples to reduce PAPR
int SqLawCompressor(float Sample)
{
	// Square Law compressor to minimize PAPR 
	// Applied after modulation but before transmission
	// Will require a companion SqLawExpander at the Receiving end before demodulation 
	// If it proves effective can be implemented with look up tables to minimize calculations

	float dblScaleFactor;
	float x;

	if (Sample == 0)
		return 0;

	dblScaleFactor = 30000 / powf(PeakValue, dblEx); // ' Improvement 3/5/2019 to use Factor 1.2 to improves PAPR by 1-2 dB
 
	if (Sample > 0)
		x = dblScaleFactor * powf(Sample, dblEx);
	else 
		x = -dblScaleFactor * powf(-Sample,dblEx);

	if (x > 30000)
		return x;

	return x;
}



void GetTwoToneLeaderWithSync(int intSymLen)
{
	// Generate a 50 baud (20 ms symbol time) 2 tone leader 
    // leader tones used are 1475 and 1525 Hz.  
  
	int intSign = 1;
	int i, j;
	short intSample;

    if ((intSymLen & 1) == 1) 
		intSign = -1;

	for (i = 0; i < intSymLen; i++)   //for the number of symbols needed (two symbols less than total leader length) 
	{
		for (j = 0; j < 240; j++)	// for 240 samples per symbol (50 baud) 
		{
           if (i != (intSymLen - 1)) 
			   intSample = intSign * int50BaudTwoToneLeaderTemplate[j];
		   else
			   intSample = -intSign * int50BaudTwoToneLeaderTemplate[j];	// This reverses phase of last symbol
		   SampleSink(intSample);
		}
		intSign = -intSign;
	}
}

void SendLeaderAndSYNC(UCHAR * bytEncodedBytes, int intLeaderLen)
{
	int intLeaderLenMS;
	if (intLeaderLen == 0)
		intLeaderLenMS = LeaderLength;
	else
		intLeaderLenMS = intLeaderLen;

 	// Create the leader

	GetTwoToneLeaderWithSync(intLeaderLenMS / 20);
}

// Function to extract an 8PSK symbol from an encoded data array

UCHAR GetSym8PSK(int intDataPtr, int k, int intCar, UCHAR * bytEncodedBytes, int intDataBytesPerCar)
{
	int int3Bytes = bytEncodedBytes[intDataPtr + intCar * intDataBytesPerCar];
//	int intMask  = 7;
	int intSym;
	UCHAR bytSym;

	int3Bytes = int3Bytes << 8;
	int3Bytes += bytEncodedBytes[intDataPtr + intCar * intDataBytesPerCar + 1];
	int3Bytes = int3Bytes << 8;
	int3Bytes += bytEncodedBytes[intDataPtr + intCar * intDataBytesPerCar + 2];  // now have 3 bytes, 24 bits or 8 8PSK symbols 
//	intMask = intMask << (3 * (7 - k));
	intSym = int3Bytes >> (3 * (7 - k));
	bytSym = intSym & 7;	//(intMask && int3Bytes) >> (3 * (7 - k));

	return bytSym;
}

//  Sub to Mod 1 Car 4PSK data (no Viterbi) used in 4PSK 50 baud control frames

VOID Mod1Car50Bd4PSK(UCHAR * bytEncodedData, int Len, int LeaderLen)
{
	// Includes Two Tone leader 
	// intCarFreq assumed to be 1500 Hz (will work for all bandwidths [200,500, 2500Hz]
	// Data is byte data No Viterbi Encoding
	// sync is the first symbol that does not change phase from the prior two-tone symbol
	// Reference symbol (0) follows Sync and the next 8 symbols whih are the frame type + parity  and the frame type Xor ID + Parity
	// The frame type should be decoded with a true minimal phase distance decoder
	// This would have to change if a 2 symbol 4PSK sync were used. e.g.  0, 180, 0, 180.... ,.... 180, 270, 90 etc. 
	// Initially checked on basic case 1/28/2019

	int intDataPtr;
	int Type = bytEncodedData[0] >> 2;

	int intSample;
	UCHAR bytLastSym, bytMask = 0xC0, bytSymToSend;
	int intMask = 0;
	int j, k, n;
	int intPeakAmp = 0;
	int intCarIndex = 5;		 // 1500 Hz
	BOOL QAM = 0;
	
	WriteDebugLog(LOGDEBUG, "Sending Frame Type %s", Name(Type));
	DrawTXFrame(Name(Type));

//	initFilter(200,1500);
	initFilter(500,1500);
	
	intSoftClipCnt = 0;
	
	// Create the leader

	SendLeaderAndSYNC(bytEncodedData, LeaderLength);
	SendingHeader200 = FALSE;

	// Experimental code to add option to send frame type
	// in FSK.

	bytLastSym = 0;			// Always start with zero reference byte

	if (UseFSKFrameType)
	{
		//Create the 8 symbols (16 bit) 50 baud 4FSK frame type with Implied SessionID

		for(j = 0; j < 2; j++)		 // for the 2 bytes of the frame type
		{  
			UCHAR symbol = bytEncodedData[j];
		
			for(k = 0; k < 4; k++)	 // for 4 symbols per byte (3 data + 1 parity)
			{
				bytSymToSend = (symbol & 0xc0) >> 6;		// for compatibility send top 2 bits first
				
				for(n = 0; n < 240; n++)
				{
					intSample = intFSK50bdCarTemplate[bytSymToSend + 4][n];
					SampleSink(intSample);	
				}
				symbol <<= 2;
			}
		}

		// Need to create reference symbol for psk message that follows
	
		// Always send zero for reference
	
		for (n = 0; n < 240; n++)  // all the samples of a symbols 
		{		
			intSample =  int16APSK_8_8_50bdCarTemplate[intCarIndex][0][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
	
			if (intSample > intPeakAmp)
				intPeakAmp = intSample;

			SampleSink(intSample);
		}
	}
	else
	{
		// Send Reference

		for (n = 0; n < 240; n++)  // all the samples of a symbols 
		{		
			intSample =  int16APSK_8_8_50bdCarTemplate[intCarIndex][0][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
	
			if (intSample > intPeakAmp)
				intPeakAmp = intSample;

			SampleSink(intSample);
		}
	
		// Send frame type

		for (j = 0; j < 2; j++)		 // for the 2 bytes of the frame type
		{  
			UCHAR symbol = bytEncodedData[j];
		
			for(k = 0; k < 4; k++)	 // for 4 symbols per byte (3 data + 1 parity)
			{		
				// Differential phase encocoding (add to last symbol sent Mod 4)  statement below confirmed 1/29/19

				bytSymToSend = (bytLastSym + ((symbol & 0xc0) >> 6)) & 3;		// High order bits first

				for (n = 0; n < 240; n++)  // all the samples of a symbols 
				{		
					if (bytSymToSend < 2)   // This uses only 90 degree values in the 16APSK table and the symmetry of the symbols to reduce the table size by a factor of 2
						intSample =  int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * bytSymToSend][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
					else	// for Sym values 2, 3
						intSample = -int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * (bytSymToSend - 2)][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
	
					if (intSample > intPeakAmp)
						intPeakAmp = intSample;

					SampleSink(intSample);
				}
		
				bytLastSym = bytSymToSend;
				symbol <<= 2;
			}
		}
	}

	// Now send the actual frame (Len - 2 Bytes
			
	for (j = 2; j < Len; j++)		 // for the 2 bytes of the frame type
	{  
		UCHAR symbol = bytEncodedData[j];
		
		for(k = 0; k < 4; k++)	 // for 4 symbols per byte (3 data + 1 parity)
		{
			// Differential phase encocoding (add to last symbol sent Mod 4)  statement below confirmed 1/29/19

			bytSymToSend = (bytLastSym + ((symbol & 0xc0) >> 6)) & 3;		// High order bits first

			for (n = 0; n < 240; n++)  // all the samples of a symbols 
			{		
				if (bytSymToSend < 2)   // This uses only 90 degree values in the 16APSK table and the symmetry of the symbols to reduce the table size by a factor of 2
					intSample =  int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * bytSymToSend][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
				else	// for Sym values 2, 3
					intSample = -int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * (bytSymToSend - 2)][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
	
				if (intSample > intPeakAmp)
					intPeakAmp = intSample;

				SampleSink(intSample);
			}
		
			bytLastSym = bytSymToSend;
			symbol <<= 2;
		}
	}
	Flush();
}

//Encode_Mod_PSK_DataFrame


// Function to Modulate data encoded for PSK. Viterbi encode, create
// the 16 bit samples and send to sound interface
   

void ModPSKDataAndPlay(unsigned char * bytEncodedBytes, int Len, int intLeaderLen)
{
	/* Only Used by packet code - needs to be changed

	int intNumCar, intBaud, intDataLen, intRSLen, intDataPtr, intSampPerSym, intDataBytesPerCar, totSymbols;
	BOOL blnOdd;
	int Type = bytEncodedBytes[0];

	int intSample;
	int maxSample = 0;

    char strType[18] = "";
    char strMod[16] = "";
	UCHAR bytSym, bytMask = 0xC0, bytSymToSend;
	float dblCarScalingFactor;
	int intMask = 0;
	int intLeaderLenMS;
	int i, j, k, l = 4, n;
	int intCarStartIndex;
	int intPeakAmp;
	float dblMag;
	int intCarIndex = 5;
	UCHAR bytViterbiSymbols[10][16];	// Viterbi encoded symbols for one byte. I think we need to generate all carriers

	UCHAR bytLastSym[11]; // = {0}; // Holds the last symbol sent (per carrier). bytLastSym(4) is 1500 Hz carrier (only used on 1 carrier modes) 
 
	if (!FrameInfo(Type >> 2, &blnOdd, &intNumCar, strMod, &intBaud, &intDataLen, &intRSLen,  &totSymbols))
		return;

	intDataBytesPerCar = (Len - 2) / intNumCar;		// We queue the samples here, so dont copy below
	intDataBytesPerCar = intDataLen + intRSLen + 3;

	memset(encstates, 0, sizeof(encstates));

	if (strchr(strMod, 'R'))
		SetCodeRate("R=3/4");
	else
		SetCodeRate("R=1/2");

	dblEx = 0.5f;

	if (strcmp(strMod, "16APSK") == 0)
	{  
		// Change compression on 16APSK

		if (intNumCar == 1)
			dblEx = 1.0f;
		if (intNumCar == 1)
			dblEx = 0.707f;  // Change compression on 10 Car 16APSK
		if (intNumCar == 4)
			dblEx = 0.6f;  // Change compression on 10 Car 16APSK
		if (intNumCar == 10)
			dblEx = 0.6f;  // Change compression on 10 Car 16APSK
	}

	switch(intNumCar)
	{
	case 1:
		intCarStartIndex = 5;
		PeakValue = 26000;
		break;
	case 2:
		intCarStartIndex = 4;
		PeakValue = 26000 * 2;
		break;
	case 4:
		intCarStartIndex = 3;
		PeakValue = 26000 * 4;
		break;
	case 10:
		PeakValue = 26000 * 10;
		intCarStartIndex = 0;
	} 
	
	if (intBaud == 50)
		intSampPerSym = 240;
	else if (intBaud == 100)
		intSampPerSym = 120;
	else if (intBaud == 200)
	{
		if (memcmp(strMod, "4PSKC", 5 ) == 0)  // 4PSKC or 4PSKCR
			intSampPerSym = 72;		//200 baud with 10% Cyclic Prefix
		else
			intSampPerSym = 60;
	}

	if (Type == PktFrameData)
	{
		intDataBytesPerCar = pktDataLen + pktRSLen + 3;
		intDataPtr = 11;		// Over Header
		goto PktLoopBack;
	}
	
	WriteDebugLog(LOGDEBUG, "Sending Frame Type %s", Name(Type >> 2));
	DrawTXFrame(Name(Type >> 2));


	if (intNumCar == 1)
		initFilter(200,1500);
	else if (intNumCar == 2)
		initFilter(500,1500);
	else
		initFilter(2500,1500);

	if (intLeaderLen == 0)
		intLeaderLenMS = LeaderLength;
	else
		intLeaderLenMS = intLeaderLen;

	intSoftClipCnt = 0;
	
	// Create the leader

	SendLeaderAndSYNC(bytEncodedBytes, intLeaderLen);
	SendingHeader200 = FALSE;

	bytLastSym[0] = 0;

	for (j = -1; j <  4 * 2; j++) // 4PSK so 4 symbols per byte + reference
	{
		if (j == -1)
			bytSymToSend = 0;	// Reference symbol 
		else
		{
			if (j % 4 == 0)
				bytMask = 0xC0;

			// Differential phase encocoding (add to last symbol sent Mod 4)  statement below confirmed 1/29/19

			bytSymToSend = (bytLastSym[0] + ((bytEncodedBytes[j / 4] & bytMask) >> (2 * (3 - (j % 4))))) % 4; // Horrible!!
		}
		for (n = 0; n < 240; n++)  // all the samples of a symbols 
		{		
			if (bytSymToSend < 2)   // This uses only 90 degree values in the 16APSK table and the symmetry of the symbols to reduce the table size by a factor of 2
				intSample = int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * bytSymToSend][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
			else	// for Sym values 2, 3
				intSample = -int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * (bytSymToSend - 2)][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
	
//			intSample = SoftClip(intSample * 0.5 * dblCarScalingFactor);
			SampleSink(intSample);
		}
		
		bytLastSym[0] = bytSymToSend;
		bytMask = bytMask >> 2;
	}

	intPeakAmp = 0;
	maxSample = 0;

	intDataPtr = 2;  // initialize pointer to start of data.

PktLoopBack:		// Reenter here to send rest of variable length packet frame
 
	// Now create a reference symbol for each carrier
      
	//	We have to do each carrier for each sample, as we write
	//	the sample immediately 

	intCarIndex = intCarStartIndex; // initialize the carrrier index

	for (n = 0; n < intSampPerSym; n++)  // Sum for all the samples of a symbols 
	{
		intSample = 0;
		intCarIndex = intCarStartIndex;  // initialize to correct starting carrier

		for (i = 0; i < intNumCar; i++)	// across all carriers
		{
			bytSymToSend = 0;  //  using non 0 causes error on first data byte 12/8/2014   ...Values 0-3  not important (carries no data).   (Possible chance for Crest Factor reduction?)       
			bytLastSym[intCarIndex] = 0;

			intSample += int16APSK_8_8_50bdCarTemplate[intCarIndex][0][n % 120];
	
			intCarIndex ++;
			if (intCarIndex == 5)
				intCarIndex = 6;	// skip over 1500 Hz for multi carrier modes (multi carrier modes all use even hundred Hz tones)
		}

//		intSample = SqLawCompressor(intSample);

		if (intSample > maxSample)
			maxSample = intSample;

		SampleSink(intSample);
	}
      
	// End of reference phase generation

	// Unlike ARDOP_WIN we send samples as they are created,
	// so we loop through carriers, then data bytes

	if (strcmp(strMod, "4PSK") == 0 || strcmp(strMod, "4PSKR") == 0)
	{
		for (j = 0; j < intDataBytesPerCar; j++)	//  for each data symbol 
		{ 		
			// Loop through each symbol of byte (4 for PSK 2 for QAM
 
			// Viterbi encode data for each carrier

			for (i = 0; i < intNumCar; i++)
			{
				encstate = encstates[i];
				ViterbiEncode(bytEncodedBytes[intDataPtr + (i * intDataBytesPerCar)], &bytViterbiSymbols[i][0]);
				encstates[i] = encstate;
			}
			for (k = 0; k < 16; k += 2)
			{
				for (n = 0; n < intSampPerSym; n++)
				{
					intSample = 0;
					intCarIndex = intCarStartIndex; // initialize the carrrier index
	
					for (i = 0; i < intNumCar; i++) // across all active carriers
					{					
						bytSym = Get4PSKSym(&bytViterbiSymbols[i][k]);
 						bytSymToSend = ((bytLastSym[intCarIndex] + bytSym) & 3);  // Values 0-3
						if (bytSymToSend < 2) // This uses the symmetry of the symbols to reduce the table size by a factor of 2
							intSample += int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * bytSymToSend][n % 120]; //  double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
						else if (bytSymToSend < 4)
							intSample -= int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * (bytSymToSend - 2)][n % 120]; // subtract 2 from the symbol value before doubling and subtract value of table 
		
	
						if (n == intSampPerSym - 1)		// Last sample?
							bytLastSym[intCarIndex] = bytSymToSend;	
	
						intCarIndex += 1;
						if (intCarIndex == 5)
							intCarIndex = 6;  // skip over 1500 Hz carrier for multicarrier modes
				
					}	// Carriers
      	
					// Done all carriers - send sample

					intSample = SqLawCompressor(intSample);

					if (intSample > maxSample)
						maxSample = intSample;

					SampleSink(intSample);

				}	// Samples

			}  // 8 Viterbi symbols per byte

			// Done all samples for this symbol
					
			intDataPtr++;
		}
	}
	else if (strcmp(strMod, "4PSKC") == 0 || strcmp(strMod, "4PSKCR") == 0)
	{
		for (j = 0; j < intDataBytesPerCar; j++)	//  for each data symbol 
		{ 		
			// Loop through each symbol of byte (4 for PSK 2 for QAM
 
			// Viterbi encode data for each carrier

			for (i = 0; i < intNumCar; i++)
			{
				encstate = encstates[i];
				ViterbiEncode(bytEncodedBytes[intDataPtr + (i * intDataBytesPerCar)], &bytViterbiSymbols[i][0]);
				encstates[i] = encstate;
			}
			for (k = 0; k < 16; k += 2)
			{
				for (n = 0; n < intSampPerSym; n++)
				{
					intSample = 0;
					intCarIndex = intCarStartIndex; // initialize the carrrier index
	
					for (i = 0; i < intNumCar; i++) // across all active carriers
					{					
						bytSym = Get4PSKSym(&bytViterbiSymbols[i][k]);
 						bytSymToSend = ((bytLastSym[intCarIndex] + bytSym) & 3);  // Values 0-3

						// The (n + 48) below implements the 10% cyclic prefix

						if (bytSymToSend < 2) // This uses the symmetry of the symbols to reduce the table size by a factor of 2
							intSample += int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * bytSymToSend][(n + 48) % 120]; //  double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
						else if (bytSymToSend < 4)
							intSample -= int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * (bytSymToSend - 2)][(n + 48) % 120]; // subtract 2 from the symbol value before doubling and subtract value of table 
		
	
						if (n == intSampPerSym - 1)		// Last sample?
							bytLastSym[intCarIndex] = bytSymToSend;	
	
						intCarIndex += 1;
						if (intCarIndex == 5)
							intCarIndex = 6;  // skip over 1500 Hz carrier for multicarrier modes
				
					}	// Carriers
      	
					// Done all carriers - send sample

					intSample = SqLawCompressor(intSample);

					if (intSample > maxSample)
						maxSample = intSample;

					SampleSink(intSample);

				}	// Samples

			}  // 8 Viterbi symbols per byte

			// Done all samples for this symbol
					
			intDataPtr++;
		}
	}
	else if (memcmp(strMod, "16APSK", 5) == 0)
	{
		for (j = 0; j < intDataBytesPerCar; j++)	//  for each data symbol 
		{ 		
			// Viterbi encode data for each carrier

			for (i = 0; i < intNumCar; i++)
			{
				encstate = encstates[i];
				ViterbiEncode(bytEncodedBytes[intDataPtr + (i * intDataBytesPerCar)], &bytViterbiSymbols[i][0]);
				encstates[i] = encstate;
			}

			for (k = 0; k < 16; k += 4)
			{
				for (n = 0; n < intSampPerSym; n++)
				{
					intSample = 0;
					intCarIndex = intCarStartIndex; // initialize the carrrier index
	
					for (i = 0; i < intNumCar; i++) // across all active carriers
					{					
						bytSym = Get16APSKSym_8_8(&bytViterbiSymbols[i][k], &dblMag);

						bytSymToSend = ((bytLastSym[intCarIndex] + bytSym) & 15);  // Values 0-15
	
						if (bytSymToSend < 4) //   for Sym values 0,1,2,3 (outer Ring, phase = 0 to 3Pi/4) use posive values from table 
							intSample += dblMag * int16APSK_8_8_50bdCarTemplate[intCarIndex][bytSymToSend][n % 120]; //  use all the table  symbols
						else if (bytSymToSend < 8) // for Sym values 4,5,6,7 (outer ring, Phase -Pi to -Pi/4) convert to - values from table 
							intSample -= dblMag * int16APSK_8_8_50bdCarTemplate[intCarIndex][bytSymToSend - 4][n % 120]; // ' subtract 4 from the symbol value and subtract value in table 
						else if (bytSymToSend < 12) //  for Sym values 8,9,10,11 (inner Ring, phase = 0 to 3Pi/4) use scaled posive values from table 
							intSample += dblMag * int16APSK_8_8_50bdCarTemplate[intCarIndex][bytSymToSend - 8][n % 120]; // subtract 8 from the symbol, scale and add value in table 
						else  // for Sym values 12,13,14,15 (inner ring, Phase -Pi to -Pi/4) convert to - values from table 
							intSample -= dblMag * int16APSK_8_8_50bdCarTemplate[intCarIndex][bytSymToSend - 12][n % 120]; // subtract 12 from the symbol, scale and subtract value in table 
	
						if (n == intSampPerSym - 1)		// Last sample?
							bytLastSym[intCarIndex] = bytSymToSend;	
	
						intCarIndex += 1;
						if (intCarIndex == 5)
							intCarIndex = 6;  // skip over 1500 Hz carrier for multicarrier modes
				
					}	// Carriers
      	
					// Done all carriers - send sample

					intSample = SqLawCompressor(intSample);

					if (intSample > maxSample)
						maxSample = intSample;

					SampleSink(intSample);

				}	// Samples

			}  // 8 Viterbi symbols per byte

			// Done all samples for this symbol
					
			intDataPtr++;
		}
	}
			
   	if (Type == PktFrameHeader)
	{
		// just sent packet header. Send rest in current mode

		Type = 0;			// Prevent reentry

		strcpy(strMod, &pktMod[pktMode][0]);
		intDataBytesPerCar = pktDataLen + pktRSLen + 3;
		intDataPtr = 11;		// Over Header
		intNumCar = pktCarriers[pktMode];

		switch(intNumCar)
		{		
		case 1:
			intCarStartIndex = 4;
//			dblCarScalingFactor = 1.0f; // Starting at 1500 Hz  (scaling factors determined emperically to minimize crest factor)  TODO:  needs verification
			dblCarScalingFactor = 1.2f; // Starting at 1500 Hz  Selected to give < 13% clipped values yielding a PAPR = 1.6 Constellation Quality >98
		case 2:
			intCarStartIndex = 3;
//			dblCarScalingFactor = 0.53f;
			if (strcmp(strMod, "16QAM") == 0)
				dblCarScalingFactor = 0.67f; // Carriers at 1400 and 1600 Selected to give < 2.5% clipped values yielding a PAPR = 2.17, Constellation Quality >92
			else
				dblCarScalingFactor = 0.65f; // Carriers at 1400 and 1600 Selected to give < 4% clipped values yielding a PAPR = 2.0, Constellation Quality >95
			break;
		case 4:
			intCarStartIndex = 2;
//			dblCarScalingFactor = 0.29f; // Starting at 1200 Hz
			dblCarScalingFactor = 0.4f;  // Starting at 1200 Hz  Selected to give < 3% clipped values yielding a PAPR = 2.26, Constellation Quality >95
			break;
		case 8:
			intCarStartIndex = 0;
//			dblCarScalingFactor = 0.17f; // Starting at 800 Hz
			if (strcmp(strMod, "16QAM") == 0)
				dblCarScalingFactor = 0.27f; // Starting at 800 Hz  Selected to give < 1% clipped values yielding a PAPR = 2.64, Constellation Quality >94
			else
				dblCarScalingFactor = 0.25f; // Starting at 800 Hz  Selected to give < 2% clipped values yielding a PAPR = 2.5, Constellation Quality >95
		} 
		goto PktLoopBack;		// Reenter to send rest of variable length packet frame
	}
	Flush();

	WriteDebugLog(LOGDEBUG, "Max Sample %d ", maxSample);
	*/
}


// Subroutine to add trailer before filtering

void AddTrailer()
{
	int intAddedSymbols = 1 + TrailerLength / 10; // add 1 symbol + 1 per each 10 ms of MCB.Trailer
	int i, k;

	return;		// testing

	for (i = 1; i <= intAddedSymbols; i++)
	{
		for (k = 0; k < 120; k++)
		{
			SampleSink(int16APSK_8_8_50bdCarTemplate[5][0][k % 60]);
		}
	}
}

//	Resends the last frame

void RemodulateLastFrame()
{	
	if (IsDataFrame(LastSentFrameType))
	{
		ModCarrierSet(intCalcLeader);
		return;
	}

	Mod1Car50Bd4PSK(bytEncodedBytes, EncLen, intCalcLeader);  // Modulate Data frame 
}

// Filter State Variables

static float dblR = (float)0.9995f;	// insures stability (must be < 1.0) (Value .9995 7/8/2013 gives good results)
static int intN = 120;				//Length of filter 12000/100
static float dblRn;

static float dblR2;
static float dblCoef[34] = {0.0f};			// the coefficients
float dblZin = 0, dblZin_1 = 0, dblZin_2 = 0, dblZComb= 0;  // Used in the comb generator

// The resonators 
      
float dblZout_0[34] = {0.0f};	// resonator outputs
float dblZout_1[34] = {0.0f};	// resonator outputs delayed one sample
float dblZout_2[34] = {0.0f};	// resonator outputs delayed two samples

int fWidth;				// Filter BandWidth
int SampleNo;
int outCount = 0;
int first, last;		// Filter slots
int centreSlot;

float largest = 0;
float smallest = 0;

short Last120[256];		// Now need 240 for 200 Hz filter

int Last120Get = 0;
int Last120Put = 120;

int Number = 0;				// Number waiting to be sent


extern unsigned short buffer[2][1200];

unsigned short * DMABuffer;

unsigned short * SendtoCard(unsigned short * buf, int n);
unsigned short * SoundInit();

// initFilter is called to set up each packet. It selects filter width

int modmaxSample;

float ScaleFactor;

void initFilter(int Width, int Centre)
{
	int i, j;
	fWidth = Width;
	centreSlot = Centre / 100;
	largest = smallest = 0;
	SampleNo = 0;
	Number = 0;
	outCount = 0;
	memset(Last120, 0, 256);

	modmaxSample = 0;

	DMABuffer = SoundInit();

	KeyPTT(TRUE);
	SoundIsPlaying = TRUE;
	StopCapture();

	Last120Get = 0;
	Last120Put = 120;

	dblRn = powf(dblR, intN);
	dblR2 = powf(dblR, 2);

	dblZin_2 = dblZin_1 = 0;

	switch (fWidth)
	{
	case 200:

		// Used for PSK 200 Hz modulation XMIT filter  
		// implements 5 100 Hz wide sections centered on 1500 Hz  (~200 Hz wide @ - 30dB centered on 1500 Hz)
 
//		SendingHeader200 = TRUE;
		intN = 120;
//		Last120Put = 240;
//		centreSlot = Centre / 50;
		first = centreSlot - 2;
		last = centreSlot + 2;		// 7 filter sections
		ScaleFactor = 0.00833333333f;
		break;

	case 500:

		// implements 7 100 Hz wide sections centered on 1500 Hz  (~500 Hz wide @ - 30dB centered on 1500 Hz)

		intN = 120;
		first = centreSlot - 3;
		last = centreSlot + 3;		// 7 filter sections
		ScaleFactor = 0.00833333333f;
		break;

	case 1000:

		// implements 11 100 Hz wide sections centered on 1500 Hz  (~500 Hz wide @ - 30dB centered on 1500 Hz)

		intN = 120;
		first = centreSlot - 5;
		last = centreSlot + 5;		// 7 filter sections
		ScaleFactor = 0.00833333333f;
		break;

	case 2500:
		
		// implements 14 200 Hz wide sections centered on 1500 Hz  (~2000 Hz wide @ - 30dB centered on 1500 Hz)

		intN = 60;
		Last120Put = 60;

		// Can't really move centre of 2500 filter

		first = 1;
		last = 14;		// 14 filter sections

		ScaleFactor = 0.015f;	// Not sure why, but Rick's value gives very low mod	
		break;
	
	default:

		WriteDebugLog(LOGCRIT, "Invalid Filter Width %d", fWidth);
	}


	for (j = first; j <= last; j++)
	{
		dblZout_0[j] = 0;
		dblZout_1[j] = 0;
		dblZout_2[j] = 0;
	}

	// Initialise the coefficients

	for (i = first; i <= last; i++)
	{
		dblCoef[i] = 2 * dblR * cosf(2 * M_PI * i / intN); // For Frequency = bin i
	}
 }


void SampleSink(short Sample)
{
	//	Filter and send to sound interface

	// This version is passed samples one at a time, as we don't have
	//	enough RAM in embedded systems to hold a full audio frame

	int intFilLen = intN / 2;
	int j;
	float intFilteredSample = 0;			//  Filtered sample

	//	We save the previous intN samples
	//	The samples are held in a cyclic buffer

	if (SampleNo < intN)
		dblZin = Sample;
	else 
		dblZin = Sample - dblRn * Last120[Last120Get];

	if (++Last120Get == (intN + 1))
		Last120Get = 0;

	//Compute the Comb

	dblZComb = dblZin - dblZin_2 * dblR2;
	dblZin_2 = dblZin_1;
	dblZin_1 = dblZin;

	// Now the resonators
		
	for (j = first; j <= last; j++)
	{
		dblZout_0[j] = dblZComb + dblCoef[j] * dblZout_1[j] - dblR2 * dblZout_2[j];
		dblZout_2[j] = dblZout_1[j];
		dblZout_1[j] = dblZout_0[j];

		switch (fWidth)
		{
		case 200:

			// scale each by transition coeff and + (Even) or - (Odd) 

			if (SampleNo >= intFilLen)
			{
				if (j == first || j == last)
					intFilteredSample -= 0.10335703f * dblZout_0[j]; 
				else if (j == first + 1 || j == last - 1)
					intFilteredSample += 0.59357535f * dblZout_0[j]; 
				else if ((j & 1) == 0)
					intFilteredSample += (int)dblZout_0[j];
				else
					intFilteredSample -= (int)dblZout_0[j];
			}
  
			break;

		case 500:

			// scale each by transition coeff and + (Even) or - (Odd) 
			// Resonators 12 and 18 scaled to get best shape and side lobe supression to - 45 dB while keeping BW at 500 Hz @ -26 dB
			// practical range of scaling .05 to .25
			// Scaling also accomodates for the filter "gain" of approx 60. 

			if (SampleNo >= intFilLen)
			{
				if (j == first || j == last)
					intFilteredSample += 0.10542034f * dblZout_0[j]; 
				else if (j == first + 1 || j == last - 1)
					intFilteredSample -= 0.5868017f * dblZout_0[j]; 
				else if ((j & 1) == 0)
					intFilteredSample += (int)dblZout_0[j];
				else
					intFilteredSample -= (int)dblZout_0[j];
			}
        
			break;

		case 1000:

			// scale each by transition coeff and + (Even) or - (Odd) 
			// Resonators 10, 11, 19, 20 scaled to get best shape and side lobe supression to - 45 dB while keeping BW at 500 Hz @ -26 dB
			// practical range of scaling .05 to .25
			// Scaling also accomodates for the filter "gain" of approx 60. 

			if (SampleNo >= intFilLen)
			{
				if (j == first || j == last)
					intFilteredSample += 0.10542034f * dblZout_0[j]; 
				else if (j == first + 1 || j == last - 1)
					intFilteredSample -= 0.5868017f * dblZout_0[j]; 
				else if ((j & 1) == 0)
					intFilteredSample += (int)dblZout_0[j];
				else
					intFilteredSample -= (int)dblZout_0[j];
			}
        
			break;

		case 2500:

			// scale each by transition coeff and + (Even) or - (Odd) 
			// Resonators 2 and 28 scaled to get best shape and side lobe supression to - 45 dB while keeping BW at 500 Hz @ -26 dB
			// practical range of scaling .05 to .25
			// Scaling also accomodates for the filter "gain" of approx 60. 
			// This uses 14 x 200 Hz slices
          
			if (SampleNo >= intFilLen)
			{
				if (j == 1)
					intFilteredSample -= 0.098108f * dblZout_0[j]; 
				else if (j == 14)
					intFilteredSample += 0.098108f * dblZout_0[j]; 
				else if (j == 2)
					intFilteredSample += 0.572724f * dblZout_0[j]; 
				else if (j == 13)
					intFilteredSample -= 0.572724f * dblZout_0[j]; 
				else if ((j & 1) == 0)	// Even
					intFilteredSample += (int)dblZout_0[j];
				else
					intFilteredSample -= (int)dblZout_0[j];
			}
		}
	}

	if (SampleNo >= intFilLen)
	{
		intFilteredSample = intFilteredSample * 0.00833333333f; //  rescales for gain of filter
//		intFilteredSample = intFilteredSample * ScaleFactor; //  rescales for gain of filter
		largest = max(largest, intFilteredSample);	
		smallest = min(smallest, intFilteredSample);
		
		if (intFilteredSample > 32700)  // Hard clip above 32700
			intFilteredSample = 32700;
		else if (intFilteredSample < -32700)
			intFilteredSample = -32700;

#ifdef TEENSY	
		int work = (short)(intFilteredSample);
		DMABuffer[Number++] = (work + 32768) >> 4; // 12 bit left justify
#else
		DMABuffer[Number++] = (short)intFilteredSample;
#endif

		if (intFilteredSample > modmaxSample)
			modmaxSample = intFilteredSample;


		if (Number == SendSize)
		{
			// send this buffer to sound interface

			DMABuffer = SendtoCard(DMABuffer, SendSize);
			Number = 0;
		}
	}
		
	Last120[Last120Put++] = Sample;

	if (Last120Put == (intN + 1))
		Last120Put = 0;

	SampleNo++;
}

extern int dttTimeoutTrip;

extern UCHAR bytSessionID;


void Flush()
{
	SoundFlush(Number);
}



// Subroutine to make a CW ID Wave File

void sendCWID(char * strID, BOOL blnPlay)
{
	// This generates a phase synchronous FSK MORSE keying of strID
	// FSK used to maintain VOX on some sound cards
	// Sent at 90% of  max ampllitude

	char strAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/"; 

	//Look up table for strAlphabet...each bit represents one dot time, 3 adjacent dots = 1 dash
	// one dot spacing between dots or dashes

	int intCW[] = {0x17, 0x1D5, 0x75D, 0x75, 0x1, 0x15D, 
           0x1DD, 0x55, 0x5, 0x1777, 0x1D7, 0x175,
           0x77, 0x1D, 0x777, 0x5DD, 0x1DD7, 0x5D, 
           0x15, 0x7, 0x57, 0x157, 0x177, 0x757, 
           0x1D77, 0x775, 0x77777, 0x17777, 0x5777, 0x1577,
           0x557, 0x155, 0x755, 0x1DD5, 0x7775, 0x1DDDD, 0x1D57, 0x1D57};


	float dblHiPhaseInc = 2 * M_PI * 1609.375f / 12000; // 1609.375 Hz High tone
	float dblLoPhaseInc = 2 * M_PI * 1390.625f / 12000; // 1390.625  low tone
	float dblHiPhase = 0;
 	float dblLoPhase = 0;
 	int  intDotSampCnt = 768;  // about 12 WPM or so (should be a multiple of 256
	short intDot[768];
	short intSpace[768];
	int i, j, k;
	char * index;
	int intMask;
	int idoffset;

    strlop(strID, '-');		// Remove any SSID    

	// Generate the dot samples (high tone) and space samples (low tone) 

	for (i = 0; i < intDotSampCnt; i++)
	{
		if (CWOnOff)
			intSpace[i] = 0;
		else

			intSpace[i] = sin(dblLoPhase) * 0.9 * intAmp;

		intDot[i] = sin(dblHiPhase) * 0.9 * intAmp;
		dblHiPhase += dblHiPhaseInc;
		if (dblHiPhase > 2 * M_PI)
			dblHiPhase -= 2 * M_PI;
		dblLoPhase += dblLoPhaseInc;
		if (dblLoPhase > 2 * M_PI)
			dblLoPhase -= 2 * M_PI;
	}
	
	initFilter(500,1500);
   
	//Generate leader for VOX 6 dots long

	for (k = 6; k >0; k--)
		for (i = 0; i < intDotSampCnt; i++)
			SampleSink(intSpace[i]);

	for (j = 0; j < strlen(strID); j++)
	{
		index = strchr(strAlphabet, strID[j]);
		if (index)
			idoffset = index - &strAlphabet[0];
		else
			idoffset = 0;

		intMask = 0x40000000;

		if (index == NULL)
		{
			// process this as a space adding 6 dots worth of space to the wave file

			for (k = 6; k >0; k--)
				for (i = 0; i < intDotSampCnt; i++)
					SampleSink(intSpace[i]);
		}
		else
		{
		while (intMask > 0) //  search for the first non 0 bit
			if (intMask & intCW[idoffset])
				break;	// intMask is pointing to the first non 0 entry
			else
				intMask >>= 1;	//  Right shift mask
				
		while (intMask > 0) //  search for the first non 0 bit
		{
			if (intMask & intCW[idoffset])
				for (i = 0; i < intDotSampCnt; i++)
					SampleSink(intDot[i]);
			else
				for (i = 0; i < intDotSampCnt; i++)
					SampleSink(intSpace[i]);
	
			intMask >>= 1;	//  Right shift mask
		}
		}
			// add 3 dot spaces for inter letter spacing
			for (k = 6; k >0; k--)
				for (i = 0; i < intDotSampCnt; i++)
					SampleSink(intSpace[i]);
	}
	
	//add 3 spaces for the end tail
	
	for (k = 6; k >0; k--)
		for (i = 0; i < intDotSampCnt; i++)
			SampleSink(intSpace[i]);

	SoundFlush();
}
