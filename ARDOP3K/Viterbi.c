//
//  Viterbi Stuff

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#define _CRT_SECURE_NO_DEPRECATE
#define _USE_32BIT_TIME_T

#include <windows.h>
#include <winioctl.h>
#else
#define HANDLE int
#endif

#include "ARDOPC.h"

int Puncture(UCHAR * bytViterbiSymbols, int count);


// I think max symbols is 1000, but code adds 16 on end for flush. I and Q for each, so * 2
// ot sure at the moment if we need to accumulate all, but so harm for now

// Looks like APSK modes use 4 bits per symbol

UCHAR bytSoftIQ[10][1016 * 4];	//The I and Q symbols for the Viterbi decoder (no I and Q for reference symbol 

int POLYA = 0x6D;	// The two generator polynomials for the NASA Standard  r=1/2,  K=7 (Voyager) 
int POLYB = 0x4F;

extern int intOffset; // the offset input  128 = erasure, 28 = perfect 0, 228 = perfect 1
extern int intAmplitude;
extern float dbl16APSK_8_8_CarRatio;

unsigned char encstate = 0;

 
//   ' 8-bit parity lookup table
static UCHAR Partab[] =
{
 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0
};

//  Function to generate Viterbi encoded I and Q symbol bits from binary input bytes
//    Public Function EncodeBytesToSymbolBits(ByRef bytData() As Byte, Optional ByVal blnMod As Boolean = True) As Byte()
/*
        ' if blnMod is set will set bit values to modulation levels (default 128 +/- amp) 
        ' Data is the input data in bytes
        'Generates an output byte array that is 16 x the input in size  
        ' Each output symbol is two sequential bytes:
        '     First byte is the I value 0 or 1
        '     Second is the Q value 0 or 1

        Dim bytSymbols(16 * bytData.Length - 1) As Byte ' 16 times data length 
        Dim intEncState As Int32 = 0
        Dim intSymbolPtr As Integer = 0
        Dim bytH1 As Byte = &H1
        For j As Integer = 0 To bytData.Length - 1
            For i As Integer = 7 To 0 Step -1
                intEncState = intEncState << 1 Or (bytData(j) >> i) And bytH1
                bytSymbols(intSymbolPtr) = bytPartab(intEncState And bytPolyA)
                bytSymbols(intSymbolPtr + 1) = bytPartab(intEncState And bytPolyB)
                intSymbolPtr += 2
            Next
        Next
        If blnMod Then
            For i As Integer = 0 To bytSymbols.Length - 1
                bytSymbols(i) = CByte((intOffset - intAmplitude) + 2 * intAmplitude * bytSymbols(i))
            Next
        End If
        Return bytSymbols
  */
  
// ******************  Original Phil Karn C code ********************************
/* Convolutionally encode data into binary symbols */

int ViterbiEncode(unsigned char * data, unsigned char *symbols, int count)
{
	int i;
	int j;
	unsigned char * save = symbols;

	for (j = 0; j < count; j++)
	{
		for(i = 7; i >= 0; i--)
		{
			encstate = (encstate << 1) | (((*data) >> i) & 1);

			*symbols++ = Partab[encstate & POLYA];	
			*symbols++ = Partab[encstate & POLYB];
		}

		symbols -= 16;

		for (i = 0; i < 16; i++)
			symbols[i] = (intOffset - intAmplitude) + 2 * intAmplitude * symbols[i];

		symbols += 16;
		data++;
	}

	// We now have 16 * count symbols. Puncture them.

	i = Puncture(save, (count * 16));

	return i;
}


int Get4PSKSym(UCHAR * bytSymbols28_228)
{
	// This is the confirmed (grayscale) coding 12/12/2018 
	// Returns -1 of incorrect values of bytSymbols28_228
	// Note Gray coding
	
	if (bytSymbols28_228[0] == 28 && bytSymbols28_228[1] == 28)
		return 0;		// 0 deg

	if (bytSymbols28_228[0] == 28 && bytSymbols28_228[1] == 228)
		return 1;		// 90 deg
	
	if (bytSymbols28_228[0] == 228 && bytSymbols28_228[1] == 228)
		return 2;		// 180 deg

	if (bytSymbols28_228[0] == 228 && bytSymbols28_228[1] == 28)
		return 3;		// 270 deg

	return 0;			// Return 0 which handles the all 0 decoder flush symbols
}

//  This Sub translates 2 IQ symbols from the Viterbi encoder to one 16APSK(8,8) symbol for transmission yielding a Carrier magnitude (28 OR 228) and Phase (0 to 8 steps of 45 degrees) 

int Get16APSKSym_8_8(UCHAR * Sym, float * dblMag)
{
	// (future) Faster implementation could be done with a 16 byte lookup table since the input (bytSymbols28_228) have only fixed byte values of 28 or 228.

	*dblMag = 1.0f;   // these symbols will be sent at the full magnitude of one of the 8 phases (45 deg or Pi/4 radians) 

	if (Sym[0] == 228 && Sym[1] == 28 && Sym[2] == 28 && Sym[3] == 28)	
		return 0;		// 0 deg

	if (Sym[0] == 228 && Sym[1] == 28 && Sym[2] == 28 && Sym[3] == 228)
		return 1;		//45 deg

	if (Sym[0] == 228 && Sym[1] == 28 && Sym[2] == 228 && Sym[3] == 228)	
		return 2;		//90 deg

	if (Sym[0] == 228 && Sym[1] == 28 && Sym[2] == 228 && Sym[3] == 28)
		return 3;		//135 deg

	if (Sym[0] == 228 && Sym[1] == 228 && Sym[2] == 228 && Sym[3] == 28)
		return 4;		//180 deg

	if (Sym[0] == 228 && Sym[1] == 228 && Sym[2] == 228 && Sym[3] == 228)	
		return 5;		//225 deg

	if (Sym[0] == 228 && Sym[1] == 228 && Sym[2] == 28 && Sym[3] == 228)	
		return 6;		//270 deg

	if (Sym[0] == 228 && Sym[1] == 228 && Sym[2] == 28 && Sym[3] == 28)	
		return 7;		//315 deg

	*dblMag = dbl16APSK_8_8_CarRatio; //  these symbols will be sent at the reduced magnitude of one of the 8 phases (45 deg or Pi/4 radians) 

	if (Sym[0] == 28 && Sym[1] == 28 && Sym[2] == 28 && Sym[3] == 28)	
		return 8;		//0 deg

	if (Sym[0] == 28 && Sym[1] == 28 && Sym[2] == 28 && Sym[3] == 228)	
		return 9;		//45 deg

	if (Sym[0] == 28 && Sym[1] == 28 && Sym[2] == 228 && Sym[3] == 228)	
		return 10;		//90 deg

	if (Sym[0] == 28 && Sym[1] == 28 && Sym[2] == 228 && Sym[3] == 28)	
		return 11;		//135 deg

	if (Sym[0] == 28 && Sym[1] == 228 && Sym[2] == 228 && Sym[3] == 28)	
		return 12;		//180 deg

	if (Sym[0] == 28 && Sym[1] == 228 && Sym[2] == 228 && Sym[3] == 228)	
		return 13;		//225 deg

	if (Sym[0] == 28 && Sym[1] == 228 && Sym[2] == 28 && Sym[3] == 228)	
		return 14;		//270 deg

	if (Sym[0] == 28 && Sym[1] == 228 && Sym[2] == 28 && Sym[3] == 28)	
		return 15;		//315 deg

	*dblMag = 0.0f;

	return 0;			// Return 0 which handles the all 0 decoder flush symbols

}
// Viterbi Decode

float dblSQRT2 = 0;
float dblErf2 = 0;
int intAmplitude= 0;
int intOffset = 128; //  the offset input  128 = erasure, 28 = perfect 0, 228 = perfect 1


struct stcState
{
	unsigned long path;	/* Decoded path to this state */
	long metric;		/* Cumulative metric to this state */
};

int mettab[2][256];

// Butterfly Index Table ' Used instead of MACRO generated call

UCHAR bytBtfIndx[] =
{
   0, 1, 3, 2, 3, 2, 0, 1, 0, 1, 3, 2, 3, 2, 0, 1, 2, 3, 1, 0, 1, 0, 2, 3, 2, 3, 1, 0, 1, 0, 2, 3
};

// Taylor series approximation for the Error function (VB doesn't have the native Erf Function)

float Erf(float dblZ)
{
	float Erf = dblZ;
	float dblZSq = dblZ * dblZ;
	float dblZPwr = dblZ;
	float dblNfact = 1;
	int i;

	if (dblZ > 2)
		return (dblErf2 + ((dblZ - 2) / dblZ) * (1 - dblErf2));  //an approximation for the tail where the series doesn't converger well
	else if (dblZ < -2)
		return -(dblErf2 + ((dblZ + 2) / dblZ) * (1 - dblErf2)); //an approximation for the tail where the series doesn't converger well

	//Use Taylor series for 2<= dblZ <= 2

	for (i = 1; i < 21; i++ )		// 21 total terms (??)
	{
		dblNfact = dblNfact * i;
		dblZPwr = dblZPwr * dblZSq;
	
		if ((i % 2) == 0)
			Erf += dblZPwr / (dblNfact * (2 * i + 1));
		else
			Erf -= dblZPwr / (dblNfact * (2 * i + 1));
	}
	
	Erf = Erf * 2 / sqrtf(M_PI);

	return Erf;
}

// Function to compute the normal distribution
float Normal(float dblX)
{
	return (0.5f + 0.5f * Erf(dblX / dblSQRT2));
}

float Log2(float n)  
{  
    float x = logf(n);
	x = x / logf(2);  
	return x;
}  


// Generate aryMettab based on amplitude, S/N, bias and scale

void GenerateMetrics(int intAmp, float dblSNdb, float dblBias, int intScale)
{
	int s, i;

	float dblNoise, dblP0, dblP1, dblEsn0;
	float dblMetrics[2][255];

	intAmplitude = intAmp;

	dblSQRT2 = sqrtf(2);

	dblErf2 = Erf(2);
	
	dblEsn0 = powf(10.0f, (dblSNdb / 10));
	dblNoise = 0.5f / dblEsn0;
	dblNoise = sqrtf(dblNoise);
	dblP1 = Normal(((0 - intOffset + 0.5f) / intAmp - 1) / dblNoise);
	dblP0 = Normal(((0 - intOffset + 0.5f) / intAmp + 1) / dblNoise);
	dblMetrics[0][0] = Log2(2 * dblP0 / (dblP0 + dblP1)) - dblBias;
	dblMetrics[1][0] = Log2(2 * dblP1 / (dblP0 + dblP1)) - dblBias;

	for (s = 1; s < 255; s++)
	{
		if (s == 28)
			s = s + 0;
   
		dblP1 = Normal(((s - intOffset + 0.5f) / intAmp - 1) / dblNoise) - Normal(((s - intOffset - 0.5f) / intAmp - 1) / dblNoise);
		dblP0 = Normal(((s - intOffset + 0.5f) / intAmp + 1) / dblNoise) - Normal(((s - intOffset - 0.5f) / intAmp + 1) / dblNoise);
		dblMetrics[0][s] = Log2(2 * dblP0 / (dblP0 + dblP1)) - dblBias;
		dblMetrics[1][s] = Log2(2 * dblP1 / (dblP0 + dblP1)) - dblBias;
	}
	
	for (i = 0; i < 255; i++)
	{
		mettab[0][i] = floorf(dblMetrics[0][i] * intScale + 0.5);
		mettab[1][i] = floorf(dblMetrics[1][i] * intScale + 0.5);
	}
}

extern UCHAR bytFrameData[MAXCAR][MAXCARRIERLEN];		// Received chars

extern char CarrierOk[];
extern int totSymbols;

BOOL DecodeFromSymbolBits(int Carrier, UCHAR * symbols)
{
	// Normally called with bytSymboBits padded with 16 0 symbols tagged on to "flush" decoder
	// Number of Output bytes = (bytSymbolsBits.length - 16)/16

	int intMetric = 0;

	int ptrOutput = 0;
	int intBitcnt = 0;
	int mets[4];
	int intBestmetric;
	int intBestState, i;
	struct stcState stcState[2][64] = {0};
	int ptrCurrent = 0;
	int ptrNext = 1;
	int intSymbolsPtr = 0;
	UCHAR * RawData = &bytFrameData[Carrier][0];

	if (CarrierOk[Carrier])
		return TRUE;						// don't do it again

	// Initialize starting metrics to prefer 0 state 
	
	stcState[ptrCurrent][0].metric = 0;
	for (i = 1; i < 64; i++)
         stcState[ptrCurrent][i].metric = -999999;
	
	stcState[ptrCurrent][0].path = 0;

	for (intBitcnt = 0; intBitcnt < totSymbols; intBitcnt++)
	{
		 mets[0] = mettab[0][symbols[intSymbolsPtr]] + mettab[0][symbols[intSymbolsPtr + 1]];
		 mets[1] = mettab[0][symbols[intSymbolsPtr]] + mettab[1][symbols[intSymbolsPtr + 1]];
		 mets[2] = mettab[1][symbols[intSymbolsPtr]] + mettab[0][symbols[intSymbolsPtr + 1]];
		 mets[3] = mettab[1][symbols[intSymbolsPtr]] + mettab[1][symbols[intSymbolsPtr + 1]];
         intSymbolsPtr += 2;

	
		//Do the Butterfly calcs here Implemented as a loop vs Macro 


		for (i = 0; i < 32; i++)
		{
			int intM0;
			int intM1;
			int sym  = bytBtfIndx[i];

			// ACS for 0 branch 

			intM0 = stcState[ptrCurrent][i].metric + mets[sym];
			intM1 = stcState[ptrCurrent][i + 32].metric + mets[3 ^ sym];

			if (intM0 > intM1)
			{
				stcState[ptrNext][2 * i].metric = intM0;
				stcState[ptrNext][2 * i].path = stcState[ptrCurrent][i].path << 1;
			}
			else
			{
				stcState[ptrNext][2 * i].metric = intM1;
				stcState[ptrNext][2 * i].path = (stcState[ptrCurrent][i + 32].path << 1) | 1;
			}
		
			//ACS for 1 branch 

 			intM0 = stcState[ptrCurrent][i].metric + mets[3 ^ sym];
			intM1 = stcState[ptrCurrent][i + 32].metric + mets[sym];

			if (intM0 > intM1)
			{
 				stcState[ptrNext][2 * i + 1].metric = intM0;
				stcState[ptrNext][2 * i + 1].path = stcState[ptrCurrent][i].path << 1;
			}	
			else
			{
				stcState[ptrNext][2 * i + 1].metric = intM1;
				stcState[ptrNext][2 * i + 1].path = (stcState[ptrCurrent][i + 32].path << 1) | 1;
			}
		}
		
		// Swap current and next states 
		
		if ((intBitcnt & 1)!= 0)
		{
			ptrCurrent = 0;
			ptrNext = 1;
		}
		else
		{
			ptrCurrent = 1;
			ptrNext = 0;
		}
		
		if (intBitcnt > totSymbols - 7)
		{
			// In tail, poison non-zero nodes 

			for (i = 1; i < 64; i+=2)
				stcState[ptrCurrent][i].metric = -999999;
		}

		// Produce output every 8 bits once path memory is full

		if (((intBitcnt % 8) == 5) && intBitcnt > 32)
		{
			// Find current best path
		
			intBestmetric = stcState[ptrCurrent][0].metric;
			intBestState = 0;
			for (i = 1; i < 64; i++)
			{
				if (stcState[ptrCurrent][i].metric > intBestmetric)
				{
					intBestmetric = stcState[ptrCurrent][i].metric;
					intBestState = i;
				}
			}

			// Debug.WriteLine("Beststate:" & intBestState.ToString & " metric=" & stcState(ptrCurrent, intBestState).metric.ToString & "  path=" & stcState(ptrCurrent, intBestState).path.ToString)
		
			RawData[ptrOutput] = stcState[ptrCurrent][intBestState].path >> 24;
			ptrOutput++;
		}
	}

	//	Output remaining bits from 0 state 

	if (intBitcnt % 8 != 6)
	{
		stcState[ptrCurrent][0].path = stcState[ptrCurrent][0].path << (6 - (intBitcnt % 8));
	}
	
	RawData[ptrOutput++] = stcState[ptrCurrent][0].path >> 24;
	RawData[ptrOutput++] = stcState[ptrCurrent][0].path >> 15;
	RawData[ptrOutput++] = stcState[ptrCurrent][0].path >> 8;
	RawData[ptrOutput++] = stcState[ptrCurrent][0].path;

	return TRUE;
}

	
UCHAR bytPuncVect[16] = {1, 1, 0, 1, 1, 0};  // default for no puncturing (This is a vecorized version of the puncturing matrix)

int intRptInterval = 2;			// Default. First 2 for R=1/2
int punctureIndex = 0;			// index into Pucture Vector;

BOOL SetCodeRate(char * strRate)
{
	punctureIndex = 0;

    if (strcmp(strRate, "R=1/2") == 0)
		intRptInterval = 2;  // no puncturing Rate = 1/2, free distance = 10
  
	else if (strcmp(strRate, "R=3/4") == 0)
		intRptInterval = 6;  // R= 3/4, free distance = 5

	else
		return FALSE;

	return TRUE;
}
	
// Case "R=2/3" : bytPuncVect = {1, 1, 0, 1} : Return True ' R= 2/3,  free distance = 6
// Case "R=5/6" : bytPuncVect = {1, 1, 0, 1, 1, 0, 0, 1, 1, 0} : Return True 'R=5/6, free distance = 4
// Case "R=7/8" : bytPuncVect = {1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0} : Return True 'R=7/8, free distance = 3
		
// Puncturing function added by Rick Muething, KN6KB 12/2/2018 for R= 1/2, 2/3, 3/4, 5/6, 7/8
    
int Puncture(UCHAR * bytViterbiSymbols, int count)
{
	// bytPV is the appropriate Puncture Vector for the desired Rate (R= 1/2, 2/3, 3/4, 5/6, or 7/8)

	// Basically we remove specific bytes from stream. In our case of 3/4 vector is 1 1 0 1 1 0
	// so we drop every 3rd byte. 

	int n  = count;			// We get 48 symbols
	int len = 0;

	UCHAR * ptr; 

	if (intRptInterval == 2)	// No puncturing
		return n;

	ptr = bytViterbiSymbols;	// Overwrite input 

	while (n--)
	{
		if (bytPuncVect[punctureIndex++] == 1)	// if vector = 0 drop byte
		{
			*(ptr++) = *(bytViterbiSymbols);
			len ++;
		}

		bytViterbiSymbols++;

		if (punctureIndex >= intRptInterval)
			punctureIndex = 0;

	}
	return len;
}

//	Erasure function added by Rick Muething, KN6KB 12/2/2018 for R= 1/2, 2/3, 3/4, 5/6, 7/8


VOID DoErasureAndDecode(int Carrier)
{
	// inserts bytErasure values (default 128) to replace punctured symbols in bytPunctured before decoding
	// bytPuncturedSymbols must of course have the same rate and bytPuncVect as was used for puncturing! 

	UCHAR bytErasure = 128;
	int save;
	UCHAR * PuncturedSymbols = &bytSoftIQ[Carrier][0];
	int intErasureSymPtr = 0, intPVPtr = 0, intPuncturedSymPtr = 0;

	UCHAR bytWithErasures[4096];			// 1.5 * max symbols for 3/4
 
	if (intRptInterval == 2)
	{
		 // no change in bytPuncturedSymbols, no erasures inserted. Free Distance = 10
		
		DecodeFromSymbolBits(Carrier, &bytSoftIQ[Carrier][0]);
		return;	
	}

	while (intPuncturedSymPtr < (totSymbols * 2))
	{
		intPVPtr = intErasureSymPtr % intRptInterval;

		if (bytPuncVect[intPVPtr] == 1)
			bytWithErasures[intErasureSymPtr++] = PuncturedSymbols[intPuncturedSymPtr++];
		else
			bytWithErasures[intErasureSymPtr++] = bytErasure;
	}

	// I think best way is to call DecodeFromSymbolBits here so we don't need lots of space

	save = totSymbols;
	totSymbols = intErasureSymPtr;
	DecodeFromSymbolBits(Carrier, bytWithErasures );
	totSymbols = save;
}
