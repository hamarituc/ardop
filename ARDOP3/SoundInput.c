//	ARDOP Modem Decode Sound Samples

#define _CRT_SECURE_NO_DEPRECATE
#define _USE_32BIT_TIME_T

#ifdef WIN32
#include <windows.h>
#else
void Sleep(int mS);
#endif

#include "ARDOPC.h"

#pragma warning(disable : 4244)		// Code does lots of float to int

//#ifndef TEENSY
//#define MEMORYARQ
//#endif

#undef PLOTWATERFALL

#ifdef PLOTWATERFALL
#define WHITE  0xffff
#define Tomato 0xffff
#define Orange 0xffff
#define Khaki 0xffff
#define Cyan 0xffff
#define DeepSkyBlue 0
#define RoyalBlue 0 
#define Navy 0
#define Black 0
#endif


#ifdef TEENSY
#define PKTLED LED3		// flash when packet received
extern unsigned int PKTLEDTimer;
#endif

//#define max(x, y) ((x) > (y) ? (x) : (y))
//#define min(x, y) ((x) < (y) ? (x) : (y))

void SendFrametoHost(unsigned char *data, unsigned dlen);

void CheckandAdjustRXLevel(int maxlevel, int minlevel, BOOL Force);
void clearDisplay();
void updateDisplay();
VOID L2Routine(UCHAR * Packet, int Length, int FrameQuality, int totalRSErrors, int NumCar, int pktRXMode);


void DrawAxes(int Qual, const char * Frametype, char * Mode);

extern int lastmax, lastmin;		// Sample Levels

char strRcvFrameTag[32];

BOOL blnLeaderFound = FALSE;

int intLeaderRcvdMs = 1000;		// Leader length??

extern int intLastRcvdFrameQuality;
extern int intReceivedLeaderLen;
extern UCHAR bytLastReceivedDataFrameType;
extern int NErrors;
extern BOOL blnBREAKCmd;
extern UCHAR bytLastACKedDataFrameType;
extern int intARQDefaultDlyMs;
extern unsigned int tmrFinalID;
extern BOOL PKTCONNECTED;

int intLastIDSNReceived;

extern int pktRXMode;

short intPriorMixedSamples[120];  // a buffer of 120 samples to hold the prior samples used in the filter
int	intPriorMixedSamplesLength = 120;  // size of Prior sample buffer

// While searching for leader we must save unprocessed samples
// We may have up to 720 left, so need 1920 

short rawSamples[2400];	// Get Frame Type need 2400 and we may add 1200
int rawSamplesLength = 0;

short intFilteredMixedSamples[10000];	// Get Frame Type need 2400 and we may add 1200
int intFilteredMixedSamplesLength = 0;

int intFrameType;				// Type we are decoding
int LastDataFrameType;			// Last data frame processed (for Memory ARQ, etc)

char strDecodeCapture[1024];

//	Frame type parameters

int intCenterFreq = 1500;
float intCarFreq;			//(was int)	// Are these the same ??
int intNumCar;
int totSymbols;				//Total Symbols in frame
int intBaud;
int intDataLen;
int intRSLen;
int intSampleLen;
int DataRate = 0;				// For SCS Reporting
int intDataPtr;
int intSampPerSym;
int intDataBytesPerCar;
BOOL blnOdd;
char strType[18] = "";
char strMod[16] = "";
UCHAR bytMinQualThresh;
int intPSKMode;

// ARDOP V3 has max 10 carriers and 120 per carrier

// ARDOP 3 has no FSK modes

unsigned char goodCarriers = 0;	// Carriers we have already decoded

//	We always collect all phases for PSK and QAM so we can do phase correction

short intPhases[10][1000] = {0};	// We will decode as soon as we have 4 or 8 depending on mode
								//	(but need one set per carrier)
								// 652 is 163 * 4 (16QAM 100 Baud)

short intMags[10][1000] = {0};

extern UCHAR bytSoftIQ[10][1016 * 4];	


#ifdef MEMORYARQ

// Enough RAM for memory ARQ so keep all samples for FSK and a copy of tones or phase/amplitude

short intCarPhaseAvg[10][652];	// array to accumulate phases for averaging (Memory ARQ)
short intCarMagAvg[10][652];		// array to accumulate mags for averaging (Memory ARQ) 
 
#endif


//219 /3 * 8= 73 * 8 =  584
//163 * 4 = 652

//	If we do Mem ARQ we will need a fair amount of RAM

int intPhasesLen;

// Received Frame

UCHAR bytData[MAXDATALEN * WINDOW];
int frameLen;

int totalRSErrors;

char Good[10] = {1,1,1,1,1,1,1,1,1,1};		// For comparing with CarrierOK
char Bad[10] = {0};							// For comparing with CarrierOK

// We need one raw buffer per carrier

// This can be optimized quite a bit to save space
// We can probably overlay on bytData

// If we still have 600 baud modes may need a lot more for first

UCHAR bytFrameData[MAXCAR][MAXCARRIERLEN];		// Received chars

char CarrierOk[10];			// RS OK Flags per carrier

int charIndex = 0;			// Index into received chars

int SymbolsLeft;			// number still to decode

int DummyCarrier = 0;	// pseudo carrier used for long 600 baud frames
UCHAR * Decode600Buffer = &bytFrameData[0][0];

BOOL PSKInitDone = FALSE;

BOOL blnSymbolSyncFound, blnFrameSyncFound;

extern UCHAR bytLastARQSessionID;
extern UCHAR bytCurrentFrameType;
extern int intShiftUpDn;
extern const char ARQSubStates[10][11];
extern int intLastARQDataFrameToHost;

// dont think I need it short intRcvdSamples[12000];		// 1 second. May need to optimise

float dblOffsetLastGoodDecode = 0;

float dblOffsetHz = 0;;
int dttLastLeaderDetect;

extern int intRmtLeaderMeasure;

extern BOOL blnARQConnected;


extern BOOL blnPending;
extern UCHAR bytPendingSessionID;
extern UCHAR bytSessionID;

int dttLastGoodFrameTypeDecod;
int dttStartRmtLeaderMeasure;

char lastGoodID[11] = "";

int GotBitSyncTicks;

int intARQRTmeasuredMs;

float dbl2Pi = 2 * M_PI; 
float dblPi_6 = M_PI / 6;
float dblPi_4 = M_PI / 4;
float dblPi_3 = M_PI / 3;
float dblPi_2 = M_PI / 2;
float dblPi_2_3 = 2 * M_PI / 3;
float dblPi_3_4 = 3 * M_PI / 4;
float dblPi_5_6 = 5 * M_PI / 6;
 
float dblSNdBPwr;
float dblNCOFreq = 3000;	 // nominal NC) frequency
float dblNCOPhase = 0;
float dblNCOPhaseInc = 2 * M_PI * 3000 / 12000;  // was dblNCOFreq
float dblPwrSNPower_dBPrior = 0;
float dblPhaseDiff1_2Avg;  // an initial value of -10 causes initialization in AcquireFrameSyncRSBAvg


int	intMFSReadPtr = 30;				// reset the MFSReadPtr offset 30 to accomodate the filter delay

int RcvdSamplesLen = 0;				// Samples in RX buffer

float dblPhaseDiff1_2Avg;
int intPhaseError = 0;


BOOL Acquire2ToneLeaderSymbolFraming();
BOOL SearchFor2ToneLeader4(short * intNewSamples, int Length, float * dblOffsetHz, int * intSN);
BOOL AcquireFrameSyncRSBAvg();
int Acquire4PSKFrameType();
void Get4PSKPhaseTargets(UCHAR bytFrameCode, UCHAR bytID, short * intPhaseTargets);
UCHAR AddTypeParity(UCHAR bytFrameType);

void DemodulateFrame(int intFrameType);
void Demod1Car4PSKChar(int Start, UCHAR * Decoded, int Carrier);
int Demod1CarPSKChar(int Start, int Carrier);

VOID Track1Car4FSK(short * intSamples, int * intPtr, int intSampPerSymbol, float intSearchFreq, int intBaud, UCHAR * bytSymHistory);
VOID Decode1CarPSK(int Carrier);
int EnvelopeCorrelator();
int EnvelopeCorrelatorOld();
BOOL DecodeFrame(int intFrameType, UCHAR * bytData);
int Demod1CarViterbi16APSK_8_8(int Start, int Carrier);

void Update4FSKConstellation(int * intToneMags, int * intQuality);
void Update16FSKConstellation(int * intToneMags, int * intQuality);
void Update8FSKConstellation(int * intToneMags, int * intQuality);
void ProcessPingFrame(char * bytData);
int Compute4FSKSN();

VOID InitDemodPSK();
void DemodViterbiPSK();
int Demod1CarViterbiPSK(int Start, int Carrier);
BOOL DecodeFromSymbolBits(int Carrier, UCHAR * symbols);
BOOL SetCodeRate(char * strRate);
void DemodPSK();
void DrawDecode(char * Decode);

BOOL IsShortControlFrame(UCHAR bytType);
BOOL IsDataFrame(UCHAR bytType);
UCHAR GenCRC8(UCHAR * Data, int Len);
VOID PassGoodDataToHost(UCHAR bytType);
VOID ResetRXState();
VOID ResetTXState();
BOOL ProcessMultiACK(UCHAR * Msg);
BOOL CheckAndCorrectCarrier(char * bytFrameData, int intDataLen, int intRSLen, int intFrameType, int Carrier);

float dblEx = 2.0f;
int PeakFromHeader = 30000;
extern int max;

float peaksample = 0;
float peakx = 0;


void PrintCarrierFlags()
{
	if (intNumCar == 1)
		WriteDebugLog(LOGDEBUG, "MEMARQ Flags %d", CarrierOk[0]);
	else if (intNumCar == 2)
		WriteDebugLog(LOGDEBUG, "MEMARQ Flags %d %d", CarrierOk[0], CarrierOk[1]);
	else if (intNumCar == 4)
		WriteDebugLog(LOGDEBUG, "MEMARQ Flags %d %d %d %d", CarrierOk[0], CarrierOk[1], CarrierOk[2], CarrierOk[3]);
	else
	{
		WriteDebugLog(LOGDEBUG, "MEMARQ Flags %d %d %d %d %d %d %d %d %d %d",
			CarrierOk[0], CarrierOk[1], CarrierOk[2], CarrierOk[3], CarrierOk[4], CarrierOk[5], CarrierOk[6], CarrierOk[7], CarrierOk[8], CarrierOk[9]);
	}
}


float SqLawExpander(float Sample)
{
	// Rick processes whole frame. I think I can do it sample by sample

    // Keeping his code for reference
	
	// Experimental to investigate Sq Law comanding to minimize PAPR
	// Applied after demodulation but before Viterbi decoding
	// Will require a companion SqLawExpander at the Transmitting end
	// Typical executation time on laptop about 20 ms on a 5 sec frame
/*
		Dim dttStart As Date = Now
        Dim dttStop As Date
        Dim blnExpansion As Boolean = True
        Dim intMaxExpanded As Int32 = 0
        Dim dblInput(intSamples.Length - 1) As Double
        Dim dblInputMax As Double
        Dim dblNormalizedSample As Double
        Dim dblEx As Double = 2
        For i As Integer = 0 To intSamples.Length - 1
            dblInput(i) = intSamples(i)
            dblInputMax = Math.Max(dblInputMax, Abs(dblInput(i)))
        Next
        If intNumCar = 10 And strMod = "16APSK" Then dblEx = 1 / 0.7 ' Change compression on 10 Car 16APSK

        For j As Integer = 0 To intSamples.Length - 1
            dblNormalizedSample = dblInput(j) / dblInputMax
            If Not blnExpansion Then
                intSamples(j) = objMain.objMod.intAmp * dblNormalizedSample ' simply rescale to max Amp
            Else
                If dblInput(j) >= 0 Then
                    intSamples(j) = objMain.objMod.intAmp * (dblNormalizedSample ^ dblEx)
                    'intSamples(j) = objMain.objMod.intAmp * dblNormalizedSample * dblNormalizedSample
                Else
                    intSamples(j) = -objMain.objMod.intAmp * ((-dblNormalizedSample) ^ dblEx)
                    'intSamples(j) = -objMain.objMod.intAmp * dblNormalizedSample * dblNormalizedSample
                End If
            End If
        Next j
        dttStop = Now
        Debug.WriteLine("SqLawExpander Time: " & Format(dttStop.Subtract(dttStart).TotalMilliseconds, "#0.0"))
    End Sub  ' SqLawExpander
*/
	// So it looks like we normalise, square then multiply by 30000

	// Do we have to know max, or can we assume will be 30000

	// Lets try!

	float NormSample;
	float x;

	if (Sample == 0.0f)
		return 0.0f;

	if (Sample > peaksample)
		peaksample = Sample;

	NormSample = Sample / PeakFromHeader;

	if (Sample > 0)
		x = 30000 * powf(NormSample,  dblEx);
	else 
		x = -30000 * powf(-NormSample,  dblEx);

	if (x > 30000)
		return x;

	if (x > peakx)
		peakx = x;


	return x;
}

// Function to determine if frame type is short control frame
  
BOOL IsShortControlFrame(UCHAR bytType)
{
	switch (bytType)
	{
	case OVER:
	case ConRejBusy:
	case ConRejBW:
	case ConAck:
	case DISCFRAME:
	case BREAK:
	case END:
	case IDLEFRAME:
	case ACK:
		return TRUE;
	}

	return FALSE;
}
 
//	 Function to determine if it is a data frame (Even OR Odd) 

BOOL IsDataFrame(UCHAR intFrameType)
{
	const char * String = Name(intFrameType);

	if (intFrameType == PktFrameHeader)
		return TRUE;
	
	if (String == NULL || String[0] == 0)
		return FALSE;

	if (strstr(String, ".E") || strstr(String, ".O"))
		return TRUE;

	return FALSE;
}

//    Subroutine to clear all mixed samples 

void ClearAllMixedSamples()
{
	intFilteredMixedSamplesLength = 0;
	intMFSReadPtr = 0;
	rawSamplesLength = 0;	// Clear saved
}

//  Subroutine to Initialize mixed samples

void InitializeMixedSamples()
{
	// Measure the time from release of PTT to leader detection of reply.

	intARQRTmeasuredMs = min(10000, Now - dttStartRTMeasure); //?????? needs work
	intPriorMixedSamplesLength = 120;  // zero out prior samples in Prior sample buffer
	intFilteredMixedSamplesLength = 0;	// zero out the FilteredMixedSamples array
	intMFSReadPtr = 30;				// reset the MFSReadPtr offset 30 to accomodate the filter delay
}

//	Subroutine to discard all sampled prior to current intRcvdSamplesRPtr

void DiscardOldSamples()
{
	// This restructures the intRcvdSamples array discarding all samples prior to intRcvdSamplesRPtr
 
	//not sure why we need this !!
/*
	if (RcvdSamplesLen - intRcvdSamplesRPtr <= 0)
		RcvdSamplesLen = intRcvdSamplesRPtr = 0;
	else
	{
		// This is rather slow. I'd prefer a cyclic buffer. Lets see....
		
		memmove(intRcvdSamples, &intRcvdSamples[intRcvdSamplesRPtr], (RcvdSamplesLen - intRcvdSamplesRPtr)* 2);
		RcvdSamplesLen -= intRcvdSamplesRPtr;
		intRcvdSamplesRPtr = 0;
	}
*/
}

//	Subroutine to apply 2500 Hz filter to mixed samples 

float xdblZin_1 = 0, xdblZin_2 = 0, xdblZComb= 0;  // Used in the comb generator

	// The resonators 
      
float xdblZout_0[15] = {0.0f};	// resonator outputs
float xdblZout_1[15] = {0.0f};	// resonator outputs delayed one sample
float xdblZout_2[15] = {0.0f};	// resonator outputs delayed two samples
float xdblCoef[15] = {0.0};		// the coefficients
float xdblR = 0.9995f;			// ensures stability (must be < 1.0) (Value .9995 7/8/2013 gives good results)
int xintN = 60;					// Length of filter 12000/200


void FSMixFilter2500Hz(short * intMixedSamples, int intMixedSamplesLength)
{
	// assumes sample rate of 12000
	// implements  14 200Hz wide sections   (~2500 Hz wide @ - 30dB centered on 1500 Hz)

	// FSF (Frequency Selective Filter) variables

	// This works on intMixedSamples, len intMixedSamplesLength;

	// Filtered data is appended to intFilteredMixedSamples

	float dblRn;
	float dblR2;

	float dblZin = 0;
      
	int i, j;

	float intFilteredSample = 0;			//  Filtered sample

	if (intFilteredMixedSamplesLength < 0)
		WriteDebugLog(LOGERROR, "Corrupt intFilteredMixedSamplesLength");

	// Initialise Expander

	dblEx = 2.0f;			// constant amplitude modes up 0.5 compress

	if (strcmp(strMod, "16APSK") == 0)
	{  
		// Change compression on 16APSK
	
		if (intNumCar == 1)
			dblEx = 1.0f / 1.0f;
		if (intNumCar == 2)
			dblEx = 1.0f / 0.707f;
		if (intNumCar == 4)
			dblEx = 1.0f / 0.6f;
		if (intNumCar == 10)
			dblEx = 1.0f / 0.6f;
	}

	dblRn = powf(xdblR, xintN);

	dblR2 = powf(xdblR, 2);

	// Initialize the coefficients if first time
    
	if (xdblCoef[14] == 0)
	{
		for (i = 1; i <= 14; i++)
		{
			xdblCoef[i] = 2 * xdblR * cosf(2 * M_PI * i / xintN);  // For Frequency = bin i
		}
	}

	for (i = 0; i < intMixedSamplesLength; i++)
	{
		intFilteredSample = 0;

		if (i < xintN)
			dblZin = intMixedSamples[i] - dblRn * intPriorMixedSamples[i];
		else 
			dblZin = intMixedSamples[i] - dblRn * intMixedSamples[i - xintN];
 
		//Compute the Comb

		xdblZComb = dblZin - xdblZin_2 * dblR2;
		xdblZin_2 = xdblZin_1;
		xdblZin_1 = dblZin;

		// Now the resonators
		for (j = 1; j <= 14; j++)	   // calculate output for 3 resonators 
		{
			xdblZout_0[j] = xdblZComb + xdblCoef[j] * xdblZout_1[j] - dblR2 * xdblZout_2[j];
			xdblZout_2[j] = xdblZout_1[j];
			xdblZout_1[j] = xdblZout_0[j];

			//' scale each by transition coeff and + (Even) or - (Odd) 
			//' Resonators 2 and 13 scaled by .389 get best shape and side lobe supression 
			//' Scaling also accomodates for the filter "gain" of approx 60. 
 
			if (j == 1)
				intFilteredSample -= 0.098108f * xdblZout_0[j];
			else if (j == 14)
				intFilteredSample += 0.098108f * xdblZout_0[j];
			else if (j == 2)
				intFilteredSample += 0.572724f * xdblZout_0[j];
			else if (j == 13)
				intFilteredSample -= 0.572724f * xdblZout_0[j];	
			else if ((j & 1) == 0)
				intFilteredSample += xdblZout_0[j];
			else
				intFilteredSample -= xdblZout_0[j];
		}

		intFilteredSample = intFilteredSample * 0.015f; //0.00833333333f;

		// We should only expand once we have frame type

		if (State == AcquireFrame)
			intFilteredMixedSamples[intFilteredMixedSamplesLength++] = SqLawExpander(intFilteredSample);
		else
		{
			intFilteredMixedSamples[intFilteredMixedSamplesLength++] = intFilteredSample;

			// and look for peak sample
			
			if (intFilteredSample > PeakFromHeader)
				PeakFromHeader = intFilteredSample;
		}
	}
	
	// update the prior intPriorMixedSamples array for the next filter call 

//	if (State != AcquireFrame)
//		WriteDebugLog(LOGDEBUG, "Peak Sample in Type %d", PeakFromHeader);
   
	memmove(intPriorMixedSamples, &intMixedSamples[intMixedSamplesLength - xintN], intPriorMixedSamplesLength * 2);		 

	if (intFilteredMixedSamplesLength > 5000)
		WriteDebugLog(LOGERROR, "Corrupt intFilteredMixedSamplesLength %d", intFilteredMixedSamplesLength);

}

//	Function to apply 150Hz filter used in Envelope correlator

void Filter150Hz(short * intFilterOut)
{
	// assumes sample rate of 12000
	// implements  3 100 Hz wide sections   (~150 Hz wide @ - 30dB centered on 1500 Hz)

	// FSF (Frequency Selective Filter) variables

	static float dblR = 0.9995f;		// insures stability (must be < 1.0) (Value .9995 7/8/2013 gives good results)
	static int intN = 120;				//Length of filter 12000/100
	static float dblRn;
	static float dblR2;
	static float dblCoef[17] = {0.0};			// the coefficients
	float dblZin = 0, dblZin_1 = 0, dblZin_2 = 0, dblZComb= 0;  // Used in the comb generator
	// The resonators 
      
	float dblZout_0[17] = {0.0};	// resonator outputs
	float dblZout_1[17] = {0.0};	// resonator outputs delayed one sample
	float dblZout_2[17] = {0.0};	// resonator outputs delayed two samples

	int i, j;

	float FilterOut = 0;			//  Filtered sample
	float largest = 0;

	dblRn = powf(dblR, intN);

	dblR2 = powf(dblR, 2);

	// Initialize the coefficients
    
	if (dblCoef[17] == 0)
	{
		for (i = 14; i <= 16; i++)
		{
			dblCoef[i] = 2 * dblR * cosf(2 * M_PI * i / intN);  // For Frequency = bin i
		}
	}

	for (i = 0; i < 480; i++)
	{
		if (i < intN)
			dblZin = intFilteredMixedSamples[intMFSReadPtr + i] - dblRn * 0;	// no prior mixed samples
		else
			dblZin = intFilteredMixedSamples[intMFSReadPtr + i] - dblRn * intFilteredMixedSamples[intMFSReadPtr + i - intN];

		// Compute the Comb
		
		dblZComb = dblZin - dblZin_2 * dblR2;
		dblZin_2 = dblZin_1;
		dblZin_1 = dblZin;

		// Now the resonators

		for (j = 14; j <= 16; j++)		   // calculate output for 3 resonators 
		{
			dblZout_0[j] = dblZComb + dblCoef[j] * dblZout_1[j] - dblR2 * dblZout_2[j];
			dblZout_2[j] = dblZout_1[j];
			dblZout_1[j] = dblZout_0[j];
	
			//	scale each by transition coeff and + (Even) or - (Odd) 

			// Scaling also accomodates for the filter "gain" of approx 120. 
			// These transition coefficients fairly close to optimum for WGN 0db PSK4, 100 baud (yield highest average quality) 5/24/2014
 
			if (j == 14 || j == 16)
				FilterOut = 0.2f * dblZout_0[j];	 // this transisiton minimizes ringing and peaks
			else
				FilterOut -= dblZout_0[j];
		}
		intFilterOut[i] = (int)ceil(FilterOut * 0.00833333333);	 // rescales for gain of filter
	}

}

//	Function to apply 75Hz filter used in Envelope correlator

void Filter75Hz(short * intFilterOut, BOOL blnInitialise, int intSamplesToFilter)
{
	// assumes sample rate of 12000
    // implements  3 50 Hz wide sections   (~75 Hz wide @ - 30dB centered on 1500 Hz)
	// FSF (Frequency Selective Filter) variables

	static float dblR = 0.9995f;		// insures stability (must be < 1.0) (Value .9995 7/8/2013 gives good results)
	static int intN = 240;				//Length of filter 12000/50 - delays output 120 samples from input
	static float dblRn;
	static float dblR2;
	static float dblCoef[3] = {0.0};			// the coefficients
	float dblZin = 0, dblZin_1 = 0, dblZin_2 = 0, dblZComb= 0;  // Used in the comb generator
	// The resonators 
      
	float dblZout_0[3] = {0.0};	// resonator outputs
	float dblZout_1[3] = {0.0};	// resonator outputs delayed one sample
	float dblZout_2[3] = {0.0};	// resonator outputs delayed two samples

	int i, j;

	float FilterOut = 0;			//  Filtered sample
	float largest = 0;

	dblRn = powf(dblR, intN);

	dblR2 = powf(dblR, 2);

	// Initialize the coefficients
    
	if (dblCoef[2] == 0)
	{
		for (i = 0; i <= 3; i++)
		{
			dblCoef[i] = 2 * dblR * cosf(2 * M_PI * (29 + i)/ intN);  // For Frequency = bin 29, 30, 31
		}
	}

	for (i = 0; i < intSamplesToFilter; i++)
	{
		if (i < intN)
			dblZin = intFilteredMixedSamples[intMFSReadPtr + i] - dblRn * 0;	// no prior mixed samples
		else
			dblZin = intFilteredMixedSamples[intMFSReadPtr + i] - dblRn * intFilteredMixedSamples[intMFSReadPtr + i - intN];

		// Compute the Comb
		
		dblZComb = dblZin - dblZin_2 * dblR2;
		dblZin_2 = dblZin_1;
		dblZin_1 = dblZin;

		// Now the resonators

		for (j = 0; j < 3; j++)		   // calculate output for 3 resonators 
		{
			dblZout_0[j] = dblZComb + dblCoef[j] * dblZout_1[j] - dblR2 * dblZout_2[j];
			dblZout_2[j] = dblZout_1[j];
			dblZout_1[j] = dblZout_0[j];
	
			//	scale each by transition coeff and + (Even) or - (Odd) 

			// Scaling also accomodates for the filter "gain" of approx 120. 
			// These transition coefficients fairly close to optimum for WGN 0db PSK4, 100 baud (yield highest average quality) 5/24/2014
 
			if (j == 0 || j == 2)
				FilterOut -= 0.39811f * dblZout_0[j];	 // this transisiton minimizes ringing and peaks
			else
				FilterOut += dblZout_0[j];
		}
		intFilterOut[i] = (int)ceil(FilterOut * 0.0041f);	 // rescales for gain of filter
	}
}

// Subroutine to Mix new samples with NCO to tune to nominal 1500 Hz center with reversed sideband and filter. 

void MixNCOFilter(short * intNewSamples, int Length, float dblOffsetHz)
{
	// Correct the dimension of intPriorMixedSamples if needed (should only happen after a bandwidth setting change). 

	int i;
	short intMixedSamples[2400];	// All we need at once ( I hope!)		// may need to be int
	int	intMixedSamplesLength ;		//size of intMixedSamples

	if (Length == 0)
		return;

	// Nominal NCO freq is 3000 Hz  to downmix intNewSamples  (NCO - Fnew) to center of 1500 Hz (invertes the sideband too) 

	dblNCOFreq = 3000 + dblOffsetHz;
	dblNCOPhaseInc = dblNCOFreq * dbl2Pi / 12000;

	intMixedSamplesLength = Length;

	for (i = 0; i < Length; i++)
	{
		intMixedSamples[i] = (int)ceilf(intNewSamples[i] * cosf(dblNCOPhase));  // later may want a lower "cost" implementation of "Cos"
		dblNCOPhase += dblNCOPhaseInc;
		if (dblNCOPhase > dbl2Pi)
			dblNCOPhase -= dbl2Pi;
	}

	
	
	// showed no significant difference if the 2000 Hz filer used for all bandwidths.
//	printtick("Start Filter");
	FSMixFilter2500Hz(intMixedSamples, intMixedSamplesLength);   // filter through the FS filter (required to reject image from Local oscillator)
//	printtick("Done Filter");

	// save for analysys

//	WriteSamples(&intFilteredMixedSamples[oldlen], Length);
//	WriteSamples(intMixedSamples, Length);

}

//	Function to Correct Raw demodulated data with Reed Solomon FEC 

int CorrectRawDataWithRS(UCHAR * bytRawData, UCHAR * bytCorrectedData, int intDataLen, int intRSLen, int bytFrameType, int Carrier)
{
	BOOL blnRSOK;
	BOOL FrameOK;

	//Dim bytNoRS(1 + intDataLen + 2 - 1) As Byte  ' 1 byte byte Count, Data, 2 byte CRC 
	//Array.Copy(bytRawData, 0, bytNoRS, 0, bytNoRS.Length)

	if (CarrierOk[Carrier])	// Already decoded this carrier?
	{
		WriteDebugLog(LOGDEBUG, "[CorrectRawDataWithRS] Carrier %d already decoded Len %d, PSN = %d", Carrier, bytRawData[1], bytRawData[0]);
		return bytRawData[1];			// don't do it again
	}

//	if (rand() > ((32767 * 9)/10))		// Testing  Drop 10%
//	{
//		WriteDebugLog(LOGDEBUG, "[CorrectRawDataWithRS] Deliberatly failing Carrier %d", Carrier);
//		goto returnBad;
//	}

	if (CheckCRC16FrameType(bytRawData, intDataLen + 2, bytFrameType)) // No RS correction needed
	{
		// return the actual data
		
		WriteDebugLog(LOGDEBUG, "[CorrectRawDataWithRS] OK without RS, Len = %d, PSN = %d", bytRawData[1], bytRawData[0]);
		CarrierOk[Carrier] = TRUE;
		return bytRawData[1];
	}
	
	// Try correcting with RS Parity

	FrameOK = RSDecode(bytRawData, intDataLen + 4 + intRSLen, intRSLen, &blnRSOK);

	if (blnRSOK)
	{}
//		WriteDebugLog(LOGDEBUG, "RS Says OK without correction");
	else
	if (FrameOK)
	{}
//		WriteDebugLog(LOGDEBUG, "RS Says OK after %d correction(s)", NErrors);
	else
	{
		WriteDebugLog(LOGDEBUG, "[CorrectRawDataWithRS] RS Says Can't Correct");
		goto returnBad;
	}

    if (FrameOK &&  CheckCRC16FrameType(bytRawData, intDataLen + 2, bytFrameType)) // RS correction successful 
	{
		int intFailedByteCnt = 0;
		
		// need to fix this if we want to use it
		//  test code just to determine how many corrections were applied  ...later remove
        //for (j = 0 ; j < intDataLen + 3; j++)
		//{
		//	if (bytRawData[j] <> bytCorrectedData[j])
		//		intFailedByteCnt++;
		//}

        WriteDebugLog(LOGDEBUG, "[CorrectRawDataWithRS] OK with RS %d corrections, Len = %d, PSN = %d",  NErrors, bytRawData[1], bytRawData[0]);
		totalRSErrors += NErrors;
 
		// End of test code

		CarrierOk[Carrier] = TRUE;
		return bytRawData[1];
	}
	else
        WriteDebugLog(LOGDEBUG, "[CorrectRawDataWithRS] RS says ok but CRC still bad");
	
	// return uncorrected data without byte count or RS Parity

returnBad:
     
	CarrierOk[Carrier] = FALSE;
	return intDataLen;
}



// Subroutine to process new samples as received from the sound card via Main.ProcessCapturedData
// Only called when not transmitting

short intNforGoertzel[10];
short intPSKPhase_0[10], intPSKPhase_1[10];
float dblPhase_0[10], dblPhase_1[10];			// for Virerbi, maybe use same for psk

short intCP[10];	  // Cyclic prefix offset 
float dblFreqBin[10];

BOOL CheckFrameTypeParity(int intTonePtr, int * intToneMags);

void ProcessNewSamples(short * Samples, int nSamples)
{
	BOOL blnFrameDecodedOK = FALSE;

//	LookforUZ7HOLeader(Samples, nSamples);

//	printtick("Start afsk");
//	DemodAFSK(Samples, nSamples);
//	printtick("End afsk");

//	return;

	if (ProtocolState == FECSend)
		return;

	// Append new data to anything in rawSamples

	if (rawSamplesLength)
	{
		memcpy(&rawSamples[rawSamplesLength], Samples, nSamples * 2);
		rawSamplesLength += nSamples;

		nSamples = rawSamplesLength;
		Samples = rawSamples;
	}

	rawSamplesLength = 0;

//	printtick("Start Busy");
	UpdateBusyDetector(Samples);
//	printtick("Done Busy");

	// it seems that searchforleader runs on unmixed and unfilered samples

	// Searching for leader

	if (State == SearchingForLeader)
	{
		// Search for leader as long as 960 samples (8  symbols) available

//		printtick("Start Leader Search");

		if (nSamples >= 1200)
		{
			//	printtick("Start Busy");
//			if (State == SearchingForLeader)
//				UpdateBusyDetector(Samples);
			//	printtick("Done Busy");
		
			if (ProtocolState == FECSend)
					return;
		}
		while (State == SearchingForLeader && nSamples >= 1200)
		{
			int intSN;
			
			blnLeaderFound = SearchFor2ToneLeader4(Samples, nSamples, &dblOffsetHz, &intSN);
//			blnLeaderFound = SearchFor2ToneLeader2(Samples, nSamples, &dblOffsetHz, &intSN);
		
			if (blnLeaderFound)
			{
				int i;

//				WriteDebugLog(LOGDEBUG, "Got Leader");

				dttLastLeaderDetect = Now;

				// Get peak amp for square law expander

				PeakFromHeader = 0;

				for (i = 0; i < nSamples; i++)
				{
					if (Samples[i] > PeakFromHeader)
						PeakFromHeader = Samples[i];
				}

				WriteDebugLog(LOGDEBUG, "Peak Sample in Header %d", PeakFromHeader);

				nSamples -= 480;
				Samples += 480;		// !!!! needs attention !!!

				InitializeMixedSamples();
				State = AcquireSymbolSync;
			}
			else
			{
				if (SlowCPU)
				{
					nSamples -= 480;
					Samples += 480;		 // advance pointer 2 symbols (40 ms) ' reduce CPU loading
				}
				else
				{
					nSamples -= 240;
					Samples += 240;		// !!!! needs attention !!!
				}
			}
		}
		if (State == SearchingForLeader)
		{
			// Save unused samples

			memmove(rawSamples, Samples, nSamples * 2);
			rawSamplesLength = nSamples;

//			printtick("End Leader Search");

			return;
		}
	}


	// Got leader

	//	At this point samples haven't been processed, and are in Samples, len nSamples

	// I'm going to filter all samples into intFilteredMixedSamples.

//	printtick("Start Mix");

	MixNCOFilter(Samples, nSamples, dblOffsetHz); // Mix and filter new samples (Mixing consumes all intRcvdSamples)
	nSamples = 0;	//	all used

//	printtick("Done Mix Samples");

	// Acquire Symbol Sync 

    if (State == AcquireSymbolSync)
	{
		if ((intFilteredMixedSamplesLength - intMFSReadPtr) > 960)
		{
			blnSymbolSyncFound = Acquire2ToneLeaderSymbolFraming();  // adjust the pointer to the nominal symbol start based on phase
			if (blnSymbolSyncFound)
				State = AcquireFrameSync;
			else
			{
				// Rick's Latest code (2.0.3) advances pointer instead of clearing samples
//				DiscardOldSamples();
//				ClearAllMixedSamples();
				intMFSReadPtr += 240; //  advance the MFSReadPointer one symbol and try to search for leader again. 

				// Remove used samples

				intFilteredMixedSamplesLength -= intMFSReadPtr;

				if (intFilteredMixedSamplesLength < 0)
					WriteDebugLog(LOGDEBUG, "Corrupt intFilteredMixedSamplesLength");

				memmove(intFilteredMixedSamples,
					&intFilteredMixedSamples[intMFSReadPtr], intFilteredMixedSamplesLength * 2);

				intMFSReadPtr = 0;
				State = SearchingForLeader;
				return;
			}
//			printtick("Got Sym Sync");
		}
	}
	
	//	Acquire Frame Sync
	
	if (State == AcquireFrameSync)
	{
		if ((intFilteredMixedSamplesLength - intMFSReadPtr) < 720)
			return;

		blnFrameSyncFound = AcquireFrameSyncRSBAvg();

		// Remove used samples

		intFilteredMixedSamplesLength -= intMFSReadPtr;

		if (intFilteredMixedSamplesLength < 0)
			WriteDebugLog(LOGDEBUG, "Corrupt intFilteredMixedSamplesLength");

		memmove(intFilteredMixedSamples,
			&intFilteredMixedSamples[intMFSReadPtr], intFilteredMixedSamplesLength * 2);

		intMFSReadPtr = 0;

		if (blnFrameSyncFound)
		{
			State = AcquireFrameType;
				
			//	Have frame Sync. Remove used samples from buffer

			//printtick("Got Frame Sync");

		}
		else if (intPhaseError > 2)
		{
			DiscardOldSamples();
			ClearAllMixedSamples();
			State = SearchingForLeader;
			printtick("frame sync timeout");
		}
//		else
//			printtick("no frame sync");

	}
	
	//	Acquire Frame Type

	if (State == AcquireFrameType)
	{
//		printtick("getting frame type");

		intFrameType = Acquire4PSKFrameType();
		if (intFrameType == -2)
		{
//			sprintf(Msg, "not enough %d %d", intFilteredMixedSamplesLength, intMFSReadPtr);
//			printtick(Msg);
			return;		//  insufficient samples
		}

		if (intFrameType == -1)		  // poor decode quality (large decode distance)
		{
			State = SearchingForLeader;
			ClearAllMixedSamples();
			DiscardOldSamples();
			WriteDebugLog(LOGDEBUG, "poor frame type decode");

			// stcStatus.BackColor = SystemColors.Control
			// stcStatus.Text = ""
			// stcStatus.ControlName = "lblRcvFrame"
			// queTNCStatus.Enqueue(stcStatus)
		}
		else
		{
			//	Get Frame info and Initialise Demodulate variables

			// We've used intMFSReadPtr samples, so remove from Buffer

//			sprintf(Msg, "Got Frame Type %x", intFrameType);
//			printtick(Msg);

			intFilteredMixedSamplesLength -= intMFSReadPtr;
	
			if (intFilteredMixedSamplesLength < 0)
				WriteDebugLog(LOGDEBUG, "Corrupt intFilteredMixedSamplesLength");
	
			memmove(intFilteredMixedSamples,
				&intFilteredMixedSamples[intMFSReadPtr], intFilteredMixedSamplesLength * 2); 

			intMFSReadPtr = 0;

			if (!FrameInfo(intFrameType, &blnOdd, &intNumCar, strMod, &intBaud, &intDataLen, &intRSLen,  &totSymbols))
			{
				printtick("bad frame type");
				State = SearchingForLeader;
				ClearAllMixedSamples();
				DiscardOldSamples();
				return;
			}



			if (IsShortControlFrame(intFrameType))
			{
				// Frame has no data so is now complete

				// See if IRStoISS shortcut can be invoked

				DrawRXFrame(1, Name(intFrameType));
		
				PassGoodDataToHost(intFrameType);

				if (ProtocolState == IRStoISS && intFrameType == OVER)
				{
					//	In this state transition to ISS if  OVER frame 
				
					Sleep(250);

					WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples] ProtocolState=IRStoISS, substate = %s ACK received. Cease BREAKS, NewProtocolState=ISS, substate ISSData", ARQSubStates[ARQState]);
					blnEnbARQRpt = FALSE;	// stop the BREAK repeats
					intLastARQDataFrameToHost = -1; // initialize to illegal value to capture first new ISS frame and pass to host

					if (bytCurrentFrameType == 0) //  hasn't been initialized yet
					{
						WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples, ProtocolState=IRStoISS, Initializing GetNextFrameData");
		   				GetNextFrameData(&intShiftUpDn, 0, "", TRUE); // just sets the initial data, frame type, and sets intShiftUpDn= 0
					}

					SetARDOPProtocolState(ISS);
					intLinkTurnovers += 1;
					ARQState = ISSData;	

					State = SearchingForLeader;
					ClearAllMixedSamples();
					DiscardOldSamples();

					ResetTXState();
		
					SendData();				 //       Send new data from outbound queue and set up repeats
					return;
				}
     
				// prepare for next

				DiscardOldSamples();
				ClearAllMixedSamples();
				State = SearchingForLeader;
				blnFrameDecodedOK = TRUE;
				WriteDebugLog(LOGDEBUG, "[DecodeFrame] Frame: %s ", Name(intFrameType));

				DecodeCompleteTime = Now;

				goto ProcessFrame;
			}

			// Any unprocessed samples need to go through SQLawExpander

			DrawRXFrame(0, Name(intFrameType));

			if (intBaud == 25)
				intSampPerSym = 480;
			else if (intBaud == 50)
				intSampPerSym = 240;
			else if (intBaud == 100)
				intSampPerSym = 120;
			else if (intBaud == 167)
				intSampPerSym = 72;
			else if (intBaud == 600)
				intSampPerSym = 20;

			if (IsDataFrame(intFrameType))
				SymbolsLeft = totSymbols; 
			else
				SymbolsLeft = intDataLen + intRSLen;	// No CRC

			if (intDataLen == 600)
				SymbolsLeft += 6;		// 600 baud has 3 * RS Blocks

			// Save data rate for PTC reporting

			if (Rate[intFrameType] > 0)
				DataRate = Rate[intFrameType];
						
			charIndex = 0;	
			PSKInitDone = 0;
			
			frameLen = 0;
			totalRSErrors = 0;

			DummyCarrier = 0;	// pseudo carrier used for long 600 baud frames
			Decode600Buffer = &bytFrameData[0][0];

			if (!IsShortControlFrame(intFrameType))
			{
               //         stcStatus.BackColor = Color.Khaki
               //         stcStatus.Text = strType
               //         stcStatus.ControlName = "lblRcvFrame"
               //         queTNCStatus.Enqueue(stcStatus)
			}

			State = AcquireFrame;
			
			if (ProtocolMode == FEC && IsDataFrame(intFrameType) && ProtocolState != FECSend)
				SetARDOPProtocolState(FECRcv);

			// if a data frame, and not the same frame type as last, reinitialise 
			// correctly received carriers byte and memory ARQ fields

//			if (IsDataFrame(intFrameType) && LastDataFrameType != intFrameType)

			if (intFrameType == PktFrameHeader || intFrameType == PktFrameData)
			{
				memset(CarrierOk, 0, sizeof(CarrierOk));
#ifdef MEMORYARQ
				memset(intToneMagsAvg, 0, sizeof(intToneMagsAvg));
				memset(intCarPhaseAvg, 0, sizeof(intCarPhaseAvg));
				memset(intCarMagAvg, 0, sizeof(intCarMagAvg));
#endif
				LastDataFrameType = intFrameType;
			}
			else if (LastDataFrameType != intFrameType)
			{
				WriteDebugLog(LOGDEBUG, "New frame type - MEMARQ flags reset");

				// If this is just a change of toggle or a control frame pass any good data to host
				// If a new data type ISS has discarded any acked out of sequence data so we must do the same
				// after passing acked stuff to host. We also reset PSN to 1
				
				PassGoodDataToHost(intFrameType);

				if (IsDataFrame(intFrameType) && (LastDataFrameType & 0xFE) != (intFrameType & 0xFE))
					ResetRXState();

				memset(CarrierOk, 0, sizeof(CarrierOk));
				LastDataFrameType = intFrameType;

				// note that although we only do mem arq if enough RAM we
				// still skip decoding carriers that have been received;

#ifdef MEMORYARQ
				memset(intSumCounts, 0, sizeof(intSumCounts));
				memset(intToneMagsAvg, 0, sizeof(intToneMagsAvg));
				memset(intCarPhaseAvg, 0, sizeof(intCarPhaseAvg));
				memset(intCarMagAvg, 0, sizeof(intCarMagAvg));
#endif
			}

			if (IsDataFrame(intFrameType))
				PrintCarrierFlags();
		}
	}
	// Acquire Frame

	if (State == AcquireFrame)
	{
		// Call DemodulateFrame for each set of samples


		DemodulateFrame(intFrameType);

		if (State == AcquireFrame)

			// We haven't got it all yet so wait for more samples	
			return;	

		//	We have the whole frame, so process it


//		printtick("got whole frame");

		// PSK and QAM quality done in Decode routines

		WriteDebugLog(LOGDEBUG, "Qual = %d", intLastRcvdFrameQuality);

		// This mechanism is to skip actual decoding and reply/change state...no need to decode 

		if (blnBREAKCmd && ProtocolState == IRS && ARQState == IRSData &&
			intFrameType != bytLastACKedDataFrameType)
		{
			// This to immediatly go to IRStoISS if blnBREAKCmd enabled.
		
			//Implements protocol rule 3.4 (allows faster break) and does not require a good frame decode.

			WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples] Skip Data Decoding when blnBREAKCmd and ProtcolState=IRS");
			intFrameRepeatInterval = ComputeInterFrameInterval(1000 + rand() % 2000);
			SetARDOPProtocolState(IRStoISS); // (ONLY IRS State where repeats are used)				
			SendCommandToHost("STATUS QUEUE BREAK new Protocol State IRStoISS");
			blnEnbARQRpt = TRUE;  // setup for repeats until changeover
			WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples] %d bytes to send in ProtocolState: %s: Send BREAK,  New state=IRStoISS (Rule 3.3)",
					bytDataToSendLength,  ARDOPStates[ProtocolState]);
 			EncodeAndSend4FSKControl(BREAK, bytSessionID, LeaderLength);

			WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples] Skip Data Decoding when blnBREAKCmd and ProtcolState=IRS");
			blnBREAKCmd = FALSE;
			goto skipDecode;
		}
						
		if (ProtocolState == IRStoISS && IsDataFrame(intFrameType))
		{
			//	In this state answer any data frame with BREAK 
		    // not necessary to decode the frame ....just frame type

			WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples] Skip Data Decoding when ProtcolState=IRStoISS, Answer with BREAK");
			intFrameRepeatInterval = ComputeInterFrameInterval(1000 + rand() % 2000);
			blnEnbARQRpt = TRUE;  // setup for repeats until changeover
 			EncodeAndSend4FSKControl(BREAK, bytSessionID, LeaderLength);
			goto skipDecode;
		}						
		
		if (ProtocolState == IRStoISS && (intFrameType == ACK || intFrameType == OVER))
		{
			//	In this state transition to ISS if  ACK frame 
		
			WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples] ProtocolState=IRStoISS, substate = %s ACK received. Cease BREAKS, NewProtocolState=ISS, substate ISSData", ARQSubStates[ARQState]);
			blnEnbARQRpt = FALSE;	// stop the BREAK repeats
			intLastARQDataFrameToHost = -1; // initialize to illegal value to capture first new ISS frame and pass to host

			if (bytCurrentFrameType == 0) //  hasn't been initialized yet
			{
				WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples, ProtocolState=IRStoISS, Initializing GetNextFrameData");
   				GetNextFrameData(&intShiftUpDn, 0, "", TRUE); // just sets the initial data, frame type, and sets intShiftUpDn= 0
			}

			Sleep(250);

			SetARDOPProtocolState(ISS);
			intLinkTurnovers += 1;
			ARQState = ISSData;	
			ResetTXState();					// Init PSN control fields
			SendData();						// Send new data from outbound queue and set up repeats
			goto skipDecode;
		}
     
		blnFrameDecodedOK = DecodeFrame(intFrameType, bytData);

ProcessFrame:	

		if (!blnFrameDecodedOK && !IsDataFrame(intFrameType))
			DrawRXFrame(2, Name(intFrameType));

		if (intFrameType == PktFrameData)
		{
#ifdef TEENSY
			SetLED(PKTLED, TRUE);		// Flash LED
			PKTLEDTimer = Now + 400;	// For 400 Ms
#endif	
			return;
		}

		if (blnFrameDecodedOK)
		{
			// Set input level if supported
			
#ifdef HASPOTS
			CheckandAdjustRXLevel(lastmax, lastmin, TRUE);
#endif
			if (AccumulateStats)
				if (IsDataFrame(intFrameType))
					if (strstr (strMod, "PSK"))
						intGoodPSKFrameDataDecodes++;
					else if (strstr (strMod, "QAM"))
						intGoodQAMFrameDataDecodes++;
					else	
						intGoodFSKFrameDataDecodes++;

#ifdef TEENSY
			if (IsDataFrame(intFrameType))
			{
				SetLED(PKTLED, TRUE);		// Flash LED
				PKTLEDTimer = Now + 400;	// For 400 Ms
			}
#endif			
			if (ProtocolMode == FEC)
			{
				if (IsDataFrame(intFrameType))	// ' check to see if a data frame
					ProcessRcvdFECDataFrame(intFrameType, bytData, blnFrameDecodedOK);
				else if (intFrameType == IDFRAME)
					AddTagToDataAndSendToHost(bytData, "IDF", frameLen);
				else if (intFrameType >= ConReq200 && intFrameType <= ConReq2500)
					ProcessUnconnectedConReqFrame(intFrameType, bytData);
				else if (intFrameType == PING)
					ProcessPingFrame(bytData);
				else if (intFrameType == DISCFRAME) 
				{
					// Special case to process DISC from previous connection (Ending station must have missed END reply to DISC) Handles protocol rule 1.5
    
					WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.ProcessNewSamples]  DISC frame received in ProtocolMode FEC, Send END with SessionID= %XX", bytLastARQSessionID);

					tmrFinalID = Now + 3000;			
					blnEnbARQRpt = FALSE;

					EncodeAndSend4FSKControl(END, bytLastARQSessionID, LeaderLength);

					// Drop through
				}
			}				
			else if (ProtocolMode == ARQ)
			{
				if (!blnTimeoutTriggered)
					ProcessRcvdARQFrame(intFrameType, bytData, frameLen, blnFrameDecodedOK);  // Process connected ARQ frames here 

				// If still in DISC monitor it
				
				if (ProtocolState == DISC && Monitor)		  // allows ARQ mode to operate like FEC when not connected
					if (intFrameType == IDFRAME)				
						AddTagToDataAndSendToHost(bytData, "IDF", frameLen);			
					else if (intFrameType >= ConReq200 && intFrameType <= ConReq2500)
						ProcessUnconnectedConReqFrame(intFrameType, bytData);

				// Data has already been passed to host by PassGoodDataToHost
			}
			else
			{
				// Unknown Mode
				bytData[frameLen] = 0;
				WriteDebugLog(LOGDEBUG, "Received Data, No State %s", bytData);
			}
		}
		else
		{
			//	Bad decode

            if (AccumulateStats)
				if (IsDataFrame(intFrameType))
					if (strstr (strMod, "PSK"))
						intFailedPSKFrameDataDecodes++;
					else if (strstr (strMod, "QAM"))
						intFailedQAMFrameDataDecodes++;
					else
						intFailedFSKFrameDataDecodes++;


            // Debug.WriteLine("[DecodePSKData2] bytPass = " & Format(bytPass, "X"))
	
			if (ProtocolMode == FEC)
			{
				if (IsDataFrame(intFrameType))	// ' check to see if a data frame
					ProcessRcvdFECDataFrame(intFrameType, bytData, blnFrameDecodedOK);
				else if (intFrameType == IDFRAME)
					AddTagToDataAndSendToHost(bytData, "ERR", frameLen);
			}				
			else if (ProtocolMode == ARQ)
			{
				if (ProtocolState == DISC)		  // allows ARQ mode to operate like FEC when not connected
				{
					if (intFrameType == IDFRAME)				
						AddTagToDataAndSendToHost(bytData, "ERR", frameLen);			

					else if (IsDataFrame(intFrameType))		// check to see if a data frame
						ProcessRcvdFECDataFrame(intFrameType, bytData, blnFrameDecodedOK);
				}
				if (!blnTimeoutTriggered)
					ProcessRcvdARQFrame(intFrameType, bytData, frameLen, blnFrameDecodedOK);  // Process connected ARQ frames here 
 
			}
  			if (ProtocolMode == FEC && ProtocolState != FECSend)
			{
				SetARDOPProtocolState(DISC);
				InitializeConnection();
			}
		}
		if (ProtocolMode == FEC && ProtocolState != FECSend)
		{
			SetARDOPProtocolState(DISC);
			InitializeConnection();
		}
skipDecode:			
		State = SearchingForLeader;
		ClearAllMixedSamples();
		DiscardOldSamples();
		return;

	}
}
// Subroutine to compute Goertzel algorithm and return Real and Imag components for a single frequency bin

void GoertzelRealImag(short intRealIn[], int intPtr, int N, float m, float * dblReal, float * dblImag)
{
	// intRealIn is a buffer at least intPtr + N in length
	// N need not be a power of 2
	// m need not be an integer
	// Computes the Real and Imaginary Freq values for bin m
	// Verified to = FFT results for at least 10 significant digits
	// Timings for 1024 Point on Laptop (64 bit Core Duo 2.2 Ghz)
	//        GoertzelRealImag .015 ms   Normal FFT (.5 ms)
	//  assuming Goertzel is proportional to N and FFT time proportional to Nlog2N
	//  FFT:Goertzel time  ratio ~ 3.3 Log2(N)

	//  Sanity check

	//if (intPtr < 0 Or (intRealIn.Length - intPtr) < N Then
    //        dblReal = 0 : dblImag = 0 : Exit Sub
     //   End If

	float dblZ_1 = 0.0f, dblZ_2 = 0.0f, dblW = 0.0f;
	float dblCoeff = 2 * cosf(2 * M_PI * m / N);
	int i;

	for (i = 0; i <= N; i++)
	{
		if (i == N)
			dblW = dblZ_1 * dblCoeff - dblZ_2;
		else
			dblW = intRealIn[intPtr] + dblZ_1 * dblCoeff - dblZ_2;

		dblZ_2 = dblZ_1;
		dblZ_1 = dblW;
		intPtr++;
	}
	*dblReal = 2 * (dblW - cosf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2
	*dblImag = 2 * (sinf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2   (this sign agrees with Scope DSP phase values) 
}

// Subroutine to compute Goertzel algorithm and return Real and Imag components for a single frequency bin with a Hanning Window function

float dblHanWin[120];
float dblHanAng;
int HanWinLen = 0;

float dblHannWin[480];
float dblHannAng;

// Subroutine to compute Goertzel algorithm and return Real and Imag components for a single frequency bin with a Hann Window function for N a multiple of 120

void GoertzelRealImagHann120(short intRealIn[], int intPtr, int N, float m, float * dblReal, float * dblImag)
{
    // This version precomputes the raised cosine (Hann or Hanning) window and uses it for any length that is a multiple of 120 samples
	// intRealIn is a buffer at least intPtr + N in length
    // N must be 960 to use this function
    // Hann coefficients are approximated for N>120 but should be close
	// m need not be an integer
	// Computes the Real and Imaginary Freq values for bin m
	// Verified to = FFT results for at least 10 significant digits
	// Timings for 1024 Point on Laptop (64 bit Core Duo 2.2 Ghz)
	//        GoertzelRealImag .015 ms   Normal FFT (.5 ms)
	//  assuming Goertzel is proportional to N and FFT time proportional to Nlog2N
	//  FFT:Goertzel time  ratio ~ 3.3 Log2(N)

 
  	float dblZ_1 = 0.0f, dblZ_2 = 0.0f, dblW = 0.0f;
	float dblCoeff = 2 * cosf(2 * M_PI * m / N);

	int i;
	int intM = N / 120;	// No if 120 sample blocks

	if (HanWinLen != N)  //if there is any change in N this is then recalculate the Hanning Window...this mechanism reduces use of Cos
	{
		HanWinLen = N;

		dblHanAng = 2 * M_PI / 120;

		for (i = 0; i < 60; i++)
		{
			dblHanWin[i] = 0.5 - 0.5 * cosf(i * dblHanAng + dblHanAng);
		}
	}

	for (i = 0; i <= N; i++)
	{
		if (i == N)
			dblW = dblZ_1 * dblCoeff - dblZ_2;
		
		else if (i < (N / 2))	// ist half of 120 sample block
				// looks like we use values 0 ti 59 then 59 down to 0
			dblW = intRealIn[intPtr] * dblHanWin[(i /intM) % 60] + dblZ_1 * dblCoeff - dblZ_2;
		else
			dblW = intRealIn[intPtr]  * dblHanWin[59 - ((i /intM) % 60)] + dblZ_1 * dblCoeff - dblZ_2;

		dblZ_2 = dblZ_1;
		dblZ_1 = dblW;
		intPtr++;
	}
	
	*dblReal = 2 * (dblW - cosf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2
	*dblImag = 2 * (sinf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2   (this sign agrees with Scope DSP phase values) 

}




void GoertzelRealImagHann960(short intRealIn[], int intPtr, int N, float m, float * dblReal, float * dblImag)
{
    // This version precomputes the raised cosine (Hann or Hanning) window and uses it for any length that is a multiple of 120 samples
	// intRealIn is a buffer at least intPtr + N in length
    // N must be a multiple of 120 to use this function
    // Hann coefficients are approximated for N>120 but should be close
	// m need not be an integer
	// Computes the Real and Imaginary Freq values for bin m
	// Verified to = FFT results for at least 10 significant digits
	// Timings for 1024 Point on Laptop (64 bit Core Duo 2.2 Ghz)
	//        GoertzelRealImag .015 ms   Normal FFT (.5 ms)
	//  assuming Goertzel is proportional to N and FFT time proportional to Nlog2N
	//  FFT:Goertzel time  ratio ~ 3.3 Log2(N)

 
  	float dblZ_1 = 0.0f, dblZ_2 = 0.0f, dblW = 0.0f;
	float dblCoeff = 2 * cosf(2 * M_PI * m / N);

	int i;
	int intM = N / 120;	// No if 120 sample blocks

	if (dblHannWin[479] < 0.5)  //if there is any change in N this is then recalculate the Hanning Window...this mechanism reduces use of Cos
	{
		dblHannAng = 2 * M_PI / 960;

		for (i = 0; i < 480; i++)
		{
			dblHannWin[i] = 0.5 - 0.5 * cosf(i * dblHannAng + dblHannAng);
		}
	}

	for (i = 0; i <= N; i++)
	{
		if (i == N)
			dblW = dblZ_1 * dblCoeff - dblZ_2;
		
		else if (i < (N / 2))	// ist half of 120 sample block
				// looks like we use values 0 ti 59 then 59 down to 0
			dblW = intRealIn[intPtr] * dblHannWin[(i /intM) % 60] + dblZ_1 * dblCoeff - dblZ_2;
		else
			dblW = intRealIn[intPtr]  * dblHannWin[479 - ((i /intM) % 60)] + dblZ_1 * dblCoeff - dblZ_2;

		dblZ_2 = dblZ_1;
		dblZ_1 = dblW;
		intPtr++;
	}
	
	*dblReal = 2 * (dblW - cosf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2
	*dblImag = 2 * (sinf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2   (this sign agrees with Scope DSP phase values) 

}





void GoertzelRealImagHanning(short intRealIn[], int intPtr, int N, float m, float * dblReal, float * dblImag)
{
	// intRealIn is a buffer at least intPtr + N in length
	// N need not be a power of 2
	// m need not be an integer
	// Computes the Real and Imaginary Freq values for bin m
	// Verified to = FFT results for at least 10 significant digits
	// Timings for 1024 Point on Laptop (64 bit Core Duo 2.2 Ghz)
	//        GoertzelRealImag .015 ms   Normal FFT (.5 ms)
	//  assuming Goertzel is proportional to N and FFT time proportional to Nlog2N
	//  FFT:Goertzel time  ratio ~ 3.3 Log2(N)

	//  Sanity check
 
  	float dblZ_1 = 0.0f, dblZ_2 = 0.0f, dblW = 0.0f;
	float dblCoeff = 2 * cosf(2 * M_PI * m / N);

	int i;

	if (HanWinLen != N)  //if there is any change in N this is then recalculate the Hanning Window...this mechanism reduces use of Cos
	{
		HanWinLen = N;

		dblHanAng = 2 * M_PI / (N - 1);

		for (i = 0; i < N; i++)
		{
			dblHanWin[i] = 0.5 - 0.5 * cosf(i * dblHanAng);
		}
	}

	for (i = 0; i <= N; i++)
	{
		if (i == N)
			dblW = dblZ_1 * dblCoeff - dblZ_2;
		else
			dblW = intRealIn[intPtr]  * dblHanWin[i] + dblZ_1 * dblCoeff - dblZ_2;

		dblZ_2 = dblZ_1;
		dblZ_1 = dblW;
		intPtr++;
	}
	
	*dblReal = 2 * (dblW - cosf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2
	*dblImag = 2 * (sinf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2   (this sign agrees with Scope DSP phase values) 
}

float  dblHammingWin480[480];

void GoertzelRealImagHamming960(short intRealIn[], int intPtr, int N, float m, float * dblReal, float * dblImag)
{
	//	 This version precomputes the Hamming window the length must be 960 samples
	// intRealIn is a buffer at least intPtr + N in length
	// N must be 960 to use this function
	// m need not be an integer

	// Computes the Real and Imaginary Freq values for bin m
	// Verified to = FFT results for at least 10 significant digits
	// Timings for 1024 Point on Laptop (64 bit Core Duo 2.2 Ghz)
	//        GoertzelRealImag .015 ms   Normal FFT (.5 ms)
	//  assuming Goertzel is proportional to N and FFT time proportional to Nlog2N
	//  FFT:Goertzel time  ratio ~ 3.3 Log2(N)

 
  	float dblZ_1 = 0.0f, dblZ_2 = 0.0f, dblW = 0.0f;
	float dblCoeff = 2 * cosf(2 * M_PI * m / N);

	int i;

	if (dblHammingWin480[479] < 0.5 )  //if there is any cHamge in N this is then recalculate the Hanning Window...this mechanism reduces use of Cos
	{
		float dblAng = M_PI / 480;

		for (i = 0; i < 480; i++)
		{
			dblHammingWin480[i] = 0.54f - 0.46f * cosf(i * dblAng); //'Hamming
		}
	}

	for (i = 0; i <= N; i++)
	{
		if (i == N)
			dblW = dblZ_1 * dblCoeff - dblZ_2;
		else
		{
			if (i < 480)
				dblW = intRealIn[intPtr]  * dblHammingWin480[i] + dblZ_1 * dblCoeff - dblZ_2;
			else
				dblW = intRealIn[intPtr]  * dblHammingWin480[479 - (i - 480)] + dblZ_1 * dblCoeff - dblZ_2;
		}
		dblZ_2 = dblZ_1;
		dblZ_1 = dblW;
		intPtr++;
	}
	
	*dblReal = 2 * (dblW - cosf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2
	*dblImag = 2 * (sinf(2 * M_PI * m / N) * dblZ_2) / N;  // scale results by N/2   (this sign agrees with Scope DSP phase values) 
}

// Function to interpolate spectrum peak using Quinn algorithm

float QuinnSpectralPeakLocator(float XkM1Re, float XkM1Im, float XkRe, float XkIm, float XkP1Re, float XkP1Im)
{
	// based on the Quinn algorithm in Streamlining Digital Processing page 139
	// Alpha1 = Re(Xk-1/Xk)
	// Alpha2 = Re(Xk+1/Xk)
	//Delta1 = Alpha1/(1 - Alpha1)
	//'Delta2 = Alpha2/(1 - Alpha2)
	// if Delta1 > 0 and Delta2 > 0 then Delta = Delta2 else Delta = Delta1
	// should be within .1 bin for S:N > 2 dB

	float dblDenom = powf(XkRe, 2) + powf(XkIm, 2);
	float dblAlpha1;
	float dblAlpha2;
	float dblDelta1;
	float dblDelta2;

	dblAlpha1 = ((XkM1Re * XkRe) + (XkM1Im * XkIm)) / dblDenom;
	dblAlpha2 = ((XkP1Re * XkRe) + (XkP1Im * XkIm)) / dblDenom;
	dblDelta1 = dblAlpha1 / (1 - dblAlpha1);
	dblDelta2 = dblAlpha2 / (1 - dblAlpha2);

	if (dblDelta1 > 0 &&  dblDelta2 > 0)
		return dblDelta2;
	else
		return dblDelta1;
}

// Function to interpolate spectrum peak using simple interpolation 

float SpectralPeakLocator(float XkM1Re, float XkM1Im, float XkRe, float XkIm, float XkP1Re, float XkP1Im, float * dblCentMag, char * Win)
{
	// Use this for Windowed samples instead of QuinnSpectralPeakLocator

	float dblLeftMag, dblRightMag;
	*dblCentMag = sqrtf(powf(XkRe, 2) + powf(XkIm, 2));

	dblLeftMag  = sqrtf(powf(XkM1Re, 2) + powf(XkM1Im, 2));
	dblRightMag  = sqrtf(powf(XkP1Re, 2) + powf(XkP1Im, 2));

	//Factor 1.22 empirically determine optimum for Hamming window
	// For Hanning Window use factor of 1.36
	// For Blackman Window use factor of  1.75
    
	if (strcmp(Win, "Blackman"))
		return 1.75 * (dblRightMag - dblLeftMag) / (dblLeftMag + *dblCentMag + dblRightMag);  // Optimized for Hamming Window
	if (strcmp(Win, "Hann"))
		return 1.36 * (dblRightMag - dblLeftMag) / (dblLeftMag + *dblCentMag + dblRightMag);  // Optimized for Hamming Window
	if (strcmp(Win, "Hamming"))
		return 1.22 * (dblRightMag - dblLeftMag) / (dblLeftMag + *dblCentMag + dblRightMag);  // Optimized for Hamming Window

	return 0;
}

// Function to detect and tune the 50 baud 2 tone leader (for all bandwidths) Updated version of SearchFor2ToneLeader2 

float dblPriorFineOffset = 1000.0f;

/*
BOOL SearchFor2ToneLeader3(short * intNewSamples, int Length, float * dblOffsetHz, int * intSN)
{
	// This version uses 10Hz bin spacing. Hamming window on Goertzel, and simple spectral peak interpolator
	// It requires about 50% more CPU time when running but produces more sensive leader detection and more accurate tuning
	// search through the samples looking for the telltail 50 baud 2 tone pattern (nominal tones 1475, 1525 Hz)
	// Find the offset in Hz (due to missmatch in transmitter - receiver tuning
	// Finds the S:N (power ratio of the tones 1475 and 1525 ratioed to "noise" averaged from bins at 1425, 1450, 1550, and 1575Hz)
 
	float dblGoertzelReal[56];
	float dblGoertzelImag[56];
	float dblMag[56];
	float dblPower, dblLeftMag, dblRightMag;
	float dblMaxPeak = 0.0, dblMaxPeakSN = 0.0, dblBinAdj;
	int intInterpCnt = 0;  // the count 0 to 3 of the interpolations that were < +/- .5 bin
	int  intIatMaxPeak = 0;
	float dblAlpha = 0.3f;  // Works well possibly some room for optimization Changed from .5 to .3 on Rev 0.1.5.3
	float dblInterpretThreshold= 1.0f; // Good results June 6, 2014 (was .4)  ' Works well possibly some room for optimization
	float dblFilteredMaxPeak = 0;
	int intStartBin, intStopBin;
	float dblLeftCar, dblRightCar, dblBinInterpLeft, dblBinInterpRight, dblCtrR, dblCtrI, dblLeftP, dblRightP;
	float dblLeftR[3], dblLeftI[3], dblRightR[3], dblRightI[3];
	int i;
	int Ptr = 0;
	float dblAvgNoisePerBin, dblCoarsePwrSN, dblBinAdj1475, dblBinAdj1525, dblCoarseOffset = 1000;
	float dblTrialOffset, dblPowerEarly, dblSNdBPwrEarly;

	if ((Length) < 1200)
		return FALSE;		// ensure there are at least 1200 samples (5 symbols of 240 samples)

	if ((Now - dttLastGoodFrameTypeDecode > 20000) && TuningRange > 0)
	{
		// this is the full search over the full tuning range selected.  Uses more CPU time and with possibly larger deviation once connected. 
		
		intStartBin = ((200 - TuningRange) / 10);
		intStopBin = 55 - intStartBin;

		dblMaxPeak = 0;

		// Generate the Power magnitudes for up to 56 10 Hz bins (a function of MCB.TuningRange) 
  
		for (i = intStartBin; i <= intStopBin; i++)
		{
            // note hamming window reduces end effect caused by 1200 samples (not an even multiple of 240)  but spreads response peaks
		
			GoertzelRealImagHamming(intNewSamples, Ptr, 1200, i + 122.5f, &dblGoertzelReal[i], &dblGoertzelImag[i]);
			dblMag[i] = powf(dblGoertzelReal[i], 2) + powf(dblGoertzelImag[i], 2); // dblMag(i) in units of power (V^2)
 		}

		// Search the bins to locate the max S:N in the two tone signal/avg noise.  

 		for (i = intStartBin + 5; i <= intStopBin - 10; i++)	// ' +/- MCB.TuningRange from nominal 
		{
			dblPower = sqrtf(dblMag[i] * dblMag[i + 5]); // using the product to minimize sensitivity to one strong carrier vs the two tone
			// sqrt converts back to units of power from Power ^2
			// don't use center noise bin as too easily corrupted by adjacent carriers

			dblAvgNoisePerBin = (dblMag[i - 5] + dblMag[i - 3] + dblMag[i + 8] + dblMag[i + 10]) / 4;  // Simple average
			dblMaxPeak = dblPower / dblAvgNoisePerBin;
			if (dblMaxPeak > dblMaxPeakSN)
			{
				dblMaxPeakSN = dblMaxPeak;
				dblCoarsePwrSN = 10 * log10f(dblMaxPeak);
				intIatMaxPeak = i + 122;
			}
		}
		// Do the interpolation based on the two carriers at nominal 1475 and 1525Hz

		if (((intIatMaxPeak - 123) >= intStartBin) && ((intIatMaxPeak - 118) <= intStopBin)) // check to ensure no index errors
		{
			// Interpolate the adjacent bins using QuinnSpectralPeakLocator

			dblBinAdj1475 = SpectralPeakLocator(
				dblGoertzelReal[intIatMaxPeak - 123], dblGoertzelImag[intIatMaxPeak - 123],
				dblGoertzelReal[intIatMaxPeak - 122], dblGoertzelImag[intIatMaxPeak - 122], 
				dblGoertzelReal[intIatMaxPeak - 121], dblGoertzelImag[intIatMaxPeak - 121], &dblLeftMag);

			if (dblBinAdj1475 < dblInterpretThreshold && dblBinAdj1475 > -dblInterpretThreshold)
			{
				dblBinAdj = dblBinAdj1475;
				intInterpCnt += 1;
			} 

			dblBinAdj1525 = SpectralPeakLocator(
				dblGoertzelReal[intIatMaxPeak - 118], dblGoertzelImag[intIatMaxPeak - 118], 
				dblGoertzelReal[intIatMaxPeak - 117], dblGoertzelImag[intIatMaxPeak - 117], 
				dblGoertzelReal[intIatMaxPeak - 116], dblGoertzelImag[intIatMaxPeak - 116], &dblRightMag);

			if (dblBinAdj1525 < dblInterpretThreshold && dblBinAdj1525 > -dblInterpretThreshold)
			{
				dblBinAdj += dblBinAdj1525;
        		intInterpCnt += 1;
			}
			if (intInterpCnt == 0)					
			{
				dblPriorFineOffset = 1000.0f;
				return FALSE;
			}
			else
			{	
				dblBinAdj = dblBinAdj / intInterpCnt;	 // average the offsets that are within 1 bin
				dblCoarseOffset = 10.0f * (intIatMaxPeak + dblBinAdj - 147); // compute the Coarse tuning offset in Hz
			}
		}
		else
		{
			dblPriorFineOffset = 1000.0f;
			return FALSE;
		}
	}
	
	// Drop into Narrow Search
  
           
	if (dblCoarseOffset < 999)
		dblTrialOffset = dblCoarseOffset;  // use the CoarseOffset calculation from above
	else
		dblTrialOffset = *dblOffsetHz; // use the prior offset value
	
    if (fabsf(dblTrialOffset) > TuningRange && TuningRange > 0)
	{
		dblPriorFineOffset = 1000.0f;	
		return False;
	}

	dblLeftCar = 147.5f + dblTrialOffset / 10.0f;  // the nominal positions of the two tone carriers based on the last computerd dblOffsetHz
	dblRightCar = 152.5f + dblTrialOffset / 10.0f;

	// Calculate 4 bins total for Noise values in S/N computation (calculate average noise)  ' Simple average of noise bins      
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, 142.5f + dblTrialOffset / 10.0f, &dblCtrR, &dblCtrI);  // nominal center -75 Hz
	dblAvgNoisePerBin = powf(dblCtrR, 2) + powf(dblCtrI, 2);
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, 145.0f + dblTrialOffset / 10.0f, &dblCtrR, &dblCtrI); // center - 50 Hz
	dblAvgNoisePerBin += powf(dblCtrR, 2) + powf(dblCtrI, 2);
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, 155.0 + dblTrialOffset / 10.0f, &dblCtrR, &dblCtrI); // center + 50 Hz
	dblAvgNoisePerBin += powf(dblCtrR, 2) + powf(dblCtrI, 2);
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, 157.5 + dblTrialOffset / 10.0f, &dblCtrR, &dblCtrI);  // center + 75 Hz
	dblAvgNoisePerBin += powf(dblCtrR, 2) + powf(dblCtrI, 2);
	dblAvgNoisePerBin = dblAvgNoisePerBin * 0.25f; // simple average,  now units of power
  
	// Calculate one bin above and below the two nominal 2 tone positions for Quinn Spectral Peak locator
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, dblLeftCar - 1, &dblLeftR[0], &dblLeftI[0]);
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, dblLeftCar, &dblLeftR[1], &dblLeftI[1]);
	dblLeftP = powf(dblLeftR[1], 2) + powf(dblLeftI[1],  2);
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, dblLeftCar + 1, &dblLeftR[2], &dblLeftI[2]);
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, dblRightCar - 1, &dblRightR[0], &dblRightI[0]);
	GoertzelRealImagHamming(intNewSamples, Ptr, 1200, dblRightCar, &dblRightR[1], &dblRightI[1]);
	dblRightP = powf(dblRightR[1], 2) + powf(dblRightI[1], 2);
	GoertzelRealImag(intNewSamples, Ptr, 1200, dblRightCar + 1, &dblRightR[2], &dblRightI[2]);

	// Calculate the total power in the two tones 
	// This mechanism designed to reject single carrier but average both carriers if ratios is less than 4:1

	if (dblLeftP > 4 * dblRightP)
		dblPower = dblRightP;
	else if (dblRightP > 4 * dblLeftP)
		dblPower = dblLeftP;
	else
		dblPower = sqrtf(dblLeftP * dblRightP);
 
	dblSNdBPwr = 10 * log10f(dblPower / dblAvgNoisePerBin);

	// Early leader detect code to calculate S:N on the first 2 symbols)
	//  concept is to allow more accurate framing and sync detection and reduce false leader detects

	GoertzelRealImag(intNewSamples, Ptr, 480, 57.0f + dblTrialOffset / 25.0f, &dblCtrR, &dblCtrI); //  nominal center -75 Hz
	dblAvgNoisePerBin = powf(dblCtrR, 2) + powf(dblCtrI, 2);
	GoertzelRealImag(intNewSamples, Ptr, 480, 58.0f + dblTrialOffset / 25.0f, &dblCtrR, &dblCtrI); //  nominal center -75 Hz
	dblAvgNoisePerBin += powf(dblCtrR, 2) + powf(dblCtrI, 2);
	GoertzelRealImag(intNewSamples, Ptr, 480, 62.0f + dblTrialOffset / 25.0f, &dblCtrR, &dblCtrI); //  nominal center -75 Hz
	dblAvgNoisePerBin += powf(dblCtrR, 2) + powf(dblCtrI, 2);
	GoertzelRealImag(intNewSamples, Ptr, 480, 63.0f + dblTrialOffset / 25.0f, &dblCtrR, &dblCtrI); //  nominal center -75 Hz 
	dblAvgNoisePerBin = max(1000.0f, 0.25 * (dblAvgNoisePerBin + powf(dblCtrR, 2) + powf(dblCtrI, 2))); // average of 4 noise bins
	dblLeftCar = 59 + dblTrialOffset / 25;  // the nominal positions of the two tone carriers based on the last computerd dblOffsetHz
	dblRightCar = 61 + dblTrialOffset / 25;

	GoertzelRealImag(intNewSamples, Ptr, 480, dblLeftCar, &dblCtrR, &dblCtrI); // LEFT carrier
	dblLeftP = powf(dblCtrR, 2) + powf(dblCtrI, 2);
	GoertzelRealImag(intNewSamples, Ptr, 480, dblRightCar, &dblCtrR, &dblCtrI); // Right carrier
	dblRightP = powf(dblCtrR, 2) + powf(dblCtrI, 2);

	// the following rejects a single tone carrier but averages the two tones if ratio is < 4:1

	if (dblLeftP > 4 * dblRightP)
		dblPowerEarly = dblRightP;
	else if (dblRightP > 4 * dblLeftP)
		dblPowerEarly = dblLeftP;
	else
		dblPowerEarly = sqrtf(dblLeftP * dblRightP);

	dblSNdBPwrEarly = 10 * log10f(dblPowerEarly / dblAvgNoisePerBin);

	// End of Early leader detect test code 
  
	if (dblSNdBPwr > (4 + Squelch) && dblSNdBPwrEarly > Squelch && (dblAvgNoisePerBin > 100.0f || dblPriorFineOffset != 1000.0f)) // making early threshold = lower (after 3 dB compensation for bandwidth)
	{
//		WriteDebugLog(LOGDEBUG, "Fine Search S:N= %f dB, Early S:N= %f dblAvgNoisePerBin %f ", dblSNdBPwr, dblSNdBPwrEarly, dblAvgNoisePerBin);

		// Calculate the interpolation based on the left of the two tones

		dblBinInterpLeft = SpectralPeakLocator(dblLeftR[0], dblLeftI[0], dblLeftR[1], dblLeftI[1], dblLeftR[2], dblLeftI[2], &dblLeftMag);
		
		// And the right of the two tones

		dblBinInterpRight = SpectralPeakLocator(dblRightR[0], dblRightI[0], dblRightR[1], dblRightI[1], dblRightR[2], dblRightI[2], &dblRightMag);

		// Weight the interpolated values in proportion to their magnitudes
		
		dblBinInterpLeft = dblBinInterpLeft * dblLeftMag / (dblLeftMag + dblRightMag);
		dblBinInterpRight = dblBinInterpRight * dblRightMag / (dblLeftMag + dblRightMag);
	
#ifdef ARMLINUX
		{
			int x = round(dblBinInterpLeft);	// odd, but PI doesnt print floats properly 
			int y = round(dblBinInterpRight);
		
//			WriteDebugLog(LOGDEBUG, " SPL Left= %d  SPL Right= %d Offset %f, LeftMag %f RightMag %f", x, y, *dblOffsetHz, dblLeftMag, dblRightMag);
		}
#else
//		WriteDebugLog(LOGDEBUG, " SPL Left= %f  SPL Right= %f, Offset %f, LeftMag %f RightMag %f",
//			dblBinInterpLeft, dblBinInterpRight, *dblOffsetHz, dblLeftMag, dblRightMag);
#endif    
		if (fabsf(dblBinInterpLeft + dblBinInterpRight) < 1.0) // sanity check for the interpolators 
		{
			if (dblBinInterpLeft + dblBinInterpRight > 0)  // consider different bounding below
				*dblOffsetHz = dblTrialOffset + min((dblBinInterpLeft + dblBinInterpRight) * 10.0f, 3); // average left and right, adjustment bounded to +/- 3Hz max
			else
				*dblOffsetHz = dblTrialOffset + max((dblBinInterpLeft + dblBinInterpRight) * 10.0f, -3);

			// Note the addition of requiring a second detect with small offset dramatically reduces false triggering even at Squelch values of 3
			// The following demonstrated good detection down to -10 dB S:N with squelch = 3 and minimal false triggering. 
			// Added rev 0.8.2.2 11/6/2016 RM

			if (abs(dblPriorFineOffset - *dblOffsetHz) < 2.9f)
			{
				WriteDebugLog(LOGDEBUG, "Prior-Offset= %f", (dblPriorFineOffset - *dblOffsetHz));
                   		
				// Capture power for debugging ...note: convert to 3 KHz noise bandwidth from 25Hz or 12.Hz for reporting consistancy.
	
				sprintf(strDecodeCapture, "Ldr; S:N(3KHz) Early= %f dB, Full %f dB, Offset= %f Hz: ", dblSNdBPwrEarly - 20.8f, dblSNdBPwr  - 24.77f, *dblOffsetHz);

				if (AccumulateStats)
				{              
					dblLeaderSNAvg = ((dblLeaderSNAvg * intLeaderDetects) + dblSNdBPwr) / (1 + intLeaderDetects); 
					intLeaderDetects++;
				}

				dblNCOFreq = 3000 + *dblOffsetHz; // Set the NCO frequency and phase inc for mixing         
				dblNCOPhaseInc = dbl2Pi * dblNCOFreq / 12000;
				dttLastLeaderDetect = dttStartRmtLeaderMeasure = Now;
    
				State = AcquireSymbolSync;
				*intSN = dblSNdBPwr - 24.77; // 23.8dB accomodates ratio of 3Kz BW:10 Hz BW (10Log 3000/10 = 24.77)

				// don't advance the pointer here
              
				dblPriorFineOffset = 1000.0f;
				return TRUE;
			}
			else
				dblPriorFineOffset = *dblOffsetHz;

			// always use 1 symbol inc when looking for next minimal offset
		}
	}
	return FALSE;
}	



*/
BOOL SearchFor2ToneLeader4(short * intNewSamples, int Length, float * dblOffsetHz, int * intSN)
{
    // This version uses 12.5 Hz bin spacing. Hann960 window on Goertzel, and simple spectral peak interpolator optimized for Hann
    // Hann window selected for best compromise between adjacent tone rejection and sensitivity.
	// search through the samples looking for the telltail 50 baud 2 tone pattern (nominal tones 1475, 1525 Hz)
	// Find the offset in Hz (due to missmatch in transmitter - receiver tuning
	// Finds the S:N (power ratio of the tones 1475 and 1525 ratioed to "noise" averaged from bins at 1425, 1450, 1550, and 1575Hz)
 
	float dblGoertzelReal[45];
	float dblGoertzelImag[45];
	float dblMag[45];
	float dblPower, dblPwrSNdB, dblLeftMag, dblRightMag, dblAvgNoisePerBinAtPeak;
	float dblRealL, dblRealR, dblImagL, dblImagR;
	float dblMaxPeak = 0.0, dblMaxPeakSN = 0.0, dblMagWindow;
	int intInterpCnt = 0;  // the count 0 to 3 of the interpolations that were < +/- .5 bin
	int  intIatMaxPeak = 0;
	float dblAlpha = 0.3f;  // Works well possibly some room for optimization Changed from .5 to .3 on Rev 0.1.5.3
	float dblInterpretThreshold= 1.0f; // Good results June 6, 2014 (was .4)  ' Works well possibly some room for optimization
	float dblFilteredMaxPeak = 0;
	int intStartBin, intStopBin;
	int i;
	int Ptr = 0;
	float dblAvgNoisePerBin, dblBinAdj1475, dblBinAdj1525, dblCoarseOffset = 1000;
	float dblOffset = 1000; //  initialize to impossible value

	// This should allow tunning from nominal bins at 1425Hz to 1575Hz +/- 200 Hz tuning range

//	if ((Now - dttLastGoodFrameTypeDecode > 20000) && TuningRange > 0)
//	{
//		// this is the full search over the full tuning range selected.  Uses more CPU time and with possibly larger deviation once connected. 
		
	intStartBin = ((200 - TuningRange) / 12.5);
	intStopBin = 44 - intStartBin;

	dblMaxPeak = 0;
	dblMagWindow = 0;
	dblMaxPeakSN = -100;
    
	// Generate the Power magnitudes for up to 56 10 Hz bins (a function of MCB.TuningRange) 
  
	for (i = intStartBin; i <= intStopBin; i++)
	{
		// note Blackman window reduced end effect but looses sensitivity so sticking with Hann window
		// Test of 4/22/2018 indicated accurate Hann window (960) gives about 1-2 dB more sensitivity than Blackman window
		
		GoertzelRealImagHamming960(intNewSamples, Ptr, 960, i + 98, &dblGoertzelReal[i], &dblGoertzelImag[i]);
		dblMag[i] = powf(dblGoertzelReal[i], 2) + powf(dblGoertzelImag[i], 2); // dblMag(i) in units of power (V^2)
		dblMagWindow += dblMag[i];
	}

	// Search the bins to locate the max S:N in the two tone signal/avg noise.  

 	for (i = intStartBin + 4; i <= intStopBin - 8; i++)	// ' +/- MCB.TuningRange from nominal 
	{
		dblPower = sqrtf(dblMag[i] * dblMag[i + 4]); // using the product to minimize sensitivity to one strong carrier vs the two tone
		// sqrt converts back to units of power from Power ^2
		// don't use center 7 noise bins as too easily corrupted by adjacent two-tone carriers
       
		dblAvgNoisePerBin = (dblMagWindow - (dblMag[i - 1] + dblMag[i] + dblMag[i + 1] + dblMag[i + 2] + dblMag[i + 3] + dblMag[i + 4] + dblMag[i + 5])) / (intStopBin - (intStartBin + 7));
		dblMaxPeak = dblPower / dblAvgNoisePerBin;

		if (dblMaxPeak > dblMaxPeakSN)
		{
			dblMaxPeakSN = dblMaxPeak;
			dblAvgNoisePerBinAtPeak = max(dblAvgNoisePerBin, 1000.0f);
			intIatMaxPeak = i + 98;
		}
	}
		
	dblMaxPeakSN = (dblMag[intIatMaxPeak - 98] + dblMag[intIatMaxPeak - 94]) / dblAvgNoisePerBinAtPeak;
	dblPwrSNdB = 10.0f * log10f(dblMaxPeakSN);
 
	// Check aquelch

	if ((dblPwrSNdB > (2 * Squelch)) && dblPwrSNPower_dBPrior > (2 * Squelch))
	{

		// Do the interpolation based on the two carriers at nominal 1475 and 1525Hz

		if (((intIatMaxPeak - 99) >= intStartBin) && ((intIatMaxPeak - 103) <= intStopBin)) // check to ensure no index errors
		{
			// Interpolate the adjacent bins using QuinnSpectralPeakLocator

			dblBinAdj1475 = SpectralPeakLocator(
				dblGoertzelReal[intIatMaxPeak - 99], dblGoertzelImag[intIatMaxPeak - 99],
				dblGoertzelReal[intIatMaxPeak - 98], dblGoertzelImag[intIatMaxPeak - 98], 
				dblGoertzelReal[intIatMaxPeak - 97], dblGoertzelImag[intIatMaxPeak - 97], &dblLeftMag, "Hamming");

			dblBinAdj1525 = SpectralPeakLocator(
				dblGoertzelReal[intIatMaxPeak - 95], dblGoertzelImag[intIatMaxPeak - 95], 
				dblGoertzelReal[intIatMaxPeak - 94], dblGoertzelImag[intIatMaxPeak - 94], 
				dblGoertzelReal[intIatMaxPeak - 93], dblGoertzelImag[intIatMaxPeak - 93], &dblRightMag, "Hamming");

			// Weight the offset calculation by the magnitude of the dblLeftMag and dblRightMag carriers 
			
			dblOffset = 12.5 * (intIatMaxPeak + dblBinAdj1475 * dblLeftMag / (dblLeftMag + dblRightMag) + dblBinAdj1525 * dblRightMag / (dblLeftMag + dblRightMag) - 118);  // compute the Coarse tuning offset in Hz
				
			if (fabsf(dblOffset) > (TuningRange + 7))		// should always be < .5 bin or 6.25 Hz
			{
				dblPwrSNPower_dBPrior = dblPwrSNdB;
				return False;
			}
			
			// recompute the S:N based on the interpolated bins and average with computation 1 and 2 symbols in the future 
			//  Use of Hann window increases sensitivity slightly (1-2 dB)

			GoertzelRealImagHann120(intNewSamples, 0, 960, intIatMaxPeak + dblOffset / 12.5, &dblRealL, &dblImagL);
			GoertzelRealImagHann120(intNewSamples, 0, 960, intIatMaxPeak + 4 + dblOffset / 12.5, &dblRealR, &dblImagR);
			dblMaxPeakSN = (powf(dblRealL, 2) + powf(dblImagL, 2) + powf(dblRealR, 2) + powf(dblImagR, 2)) / dblAvgNoisePerBinAtPeak;
			// now compute for 120 samples later
			GoertzelRealImagHann120(intNewSamples, 120, 960, intIatMaxPeak + dblOffset / 12.5, &dblRealL, &dblImagL);
			GoertzelRealImagHann120(intNewSamples, 120, 960, intIatMaxPeak + 4 + dblOffset / 12.5, &dblRealR, &dblImagR);
			dblMaxPeakSN += (powf(dblRealL, 2) + powf(dblImagL, 2) + powf(dblRealR, 2) + powf(dblImagR, 2)) / dblAvgNoisePerBinAtPeak;
			//  and a third 240 samples later
			GoertzelRealImagHann120(intNewSamples, 240, 960, intIatMaxPeak + dblOffset / 12.5, &dblRealL, &dblImagL);
			GoertzelRealImagHann120(intNewSamples, 240, 960, intIatMaxPeak + 4 + dblOffset / 12.5, &dblRealR, &dblImagR);
			dblMaxPeakSN += (powf(dblRealL, 2) + powf(dblImagL, 2) + powf(dblRealR, 2) + powf(dblImagR, 2)) / dblAvgNoisePerBinAtPeak;
 
			dblMaxPeakSN = dblMaxPeakSN / 3;  // average the dblMaxPeakSN over the three calculations
	// ???? Calc Twice ????
 			dblMaxPeakSN = (powf(dblRealL, 2) + powf(dblImagL, 2) + powf(dblRealR, 2) + powf(dblImagR, 2)) / dblAvgNoisePerBinAtPeak;
      
			
			dblPwrSNdB = 10 * log10f(dblMaxPeakSN);
			
			if (dblPwrSNdB > 2 * Squelch)	// This average power now includes two samples from symbols +120 and + 240 samples 
			{
				//strDecodeCapture = "Ldr; S:N(3KHz) Prior=" & Format(dblPwrSNPower_dBPrior, "#.0") & "dB, Current=" & Format(dblPwrSNdB, "#.0") & "dB, Offset=" & Format(dblOffset, "##0.00") & "Hz "
  				WriteDebugLog(LOGDEBUG, "Ldr; S:N(3KHz) Avg= %f dB, Offset== %f Hz", dblPwrSNdB, dblOffset);
				dttStartRmtLeaderMeasure = Now;
				if (AccumulateStats)
				{
					dblLeaderSNAvg = ((dblLeaderSNAvg * intLeaderDetects) + dblPwrSNdB) / (1 + intLeaderDetects);
					intLeaderDetects += 1;
				}
				*dblOffsetHz = dblOffset;
				dblNCOFreq = 3000 + *dblOffsetHz;	// Set the NCO frequency and phase inc for mixing 
				dblNCOPhaseInc = dbl2Pi * dblNCOFreq / 12000;
				// don't advance the pointer here
				State = AcquireSymbolSync;
				dttLastLeaderDetect = Now;
				dblPhaseDiff1_2Avg = 10; //  initialize to 10 to cause initialization of exponential averager in AcquireFrameSyncRSBAvg 
				*intSN = round(dblPwrSNdB - 20.8); // 20.8dB accomodates ratio of 3Kz BW: (effective Blackman Window bandwidth  of ~25 Hz) 
				return True;
			}
			else
			{
				return False;
			}
		}
	}
	
	dblPwrSNPower_dBPrior = dblPwrSNdB;

	return FALSE;
}	




//	Function to look at the 2 tone leader and establishes the Symbol framing using envelope search and minimal phase error. 

BOOL Acquire2ToneLeaderSymbolFraming()
{
	float dblCarPh;
	float dblReal, dblImag;
	int intLocalPtr = intMFSReadPtr;  // try advancing one symbol to minimize initial startup errors 
	float dblAbsPhErr;
	float dblMinAbsPhErr = 5000;	 // initialize to an excessive value
	int intIatMinErr;
	float dblPhaseAtMinErr;
	int intAbsPeak = 0;
	int intJatPeak = 0;
	int i;

	// Use Phase of 1500 Hz leader  to establish symbol framing. Nominal phase is 0 or 180 degrees

	if ((intFilteredMixedSamplesLength - intLocalPtr) < 960)
		return FALSE;			// not enough
	
	intLocalPtr = intMFSReadPtr + EnvelopeCorrelator(); // should position the pointer at the symbol boundary

	if (intLocalPtr < intMFSReadPtr)
		return False; // use negative value of EnvelopeCorrelator to indicate insufficient correlation. 


	// Check 2 samples either side of the intLocalPtr for minimum phase error.(closest to Pi or -Pi) 
	// Could be as much as .4 Radians (~70 degrees) depending on sampling positions.
   
	for (i = -2; i <= 2; i++)	 // 0 To 0 '  -2 To 2 ' for just 5 samples
	{
		// using the full symbol seemed to work best on weak Signals (0 to -5 dB S/N) June 15, 2015
	
		GoertzelRealImagHann120(intFilteredMixedSamples, intLocalPtr + i, 240, 30, &dblReal, &dblImag); // Carrier at 1500 Hz nominal Positioning 

		dblCarPh = atan2f(dblImag, dblReal);

		if (dblCarPh > M_PI / 2)
			dblAbsPhErr = M_PI - dblCarPh;
		else
			dblAbsPhErr = dblCarPh;

		if (dblAbsPhErr < dblMinAbsPhErr)
		{
			dblMinAbsPhErr = dblAbsPhErr;
			intIatMinErr = i;
			dblPhaseAtMinErr = dblCarPh;
		}     
	}

	intMFSReadPtr = intLocalPtr + intIatMinErr;
	WriteDebugLog(LOGDEBUG, "[Acquire2ToneLeaderSymbolFraming] intIatMinError= %d", intIatMinErr);
	State = AcquireFrameSync;

	if (AccumulateStats)
		intLeaderSyncs++;

	//Debug.WriteLine("   [Acquire2ToneLeaderSymbolSync] iAtMinError = " & intIatMinErr.ToString & "   Ptr = " & intMFSReadPtr.ToString & "  MinAbsPhErr = " & Format(dblMinAbsPhErr, "#.00"))
	//Debug.WriteLine("   [Acquire2ToneLeaderSymbolSync]      Ph1500 @ MinErr = " & Format(dblPhaseAtMinErr, "#.000"))
        
	//strDecodeCapture &= "Framing; iAtMinErr=" & intIatMinErr.ToString & ", Ptr=" & intMFSReadPtr.ToString & ", MinAbsPhErr=" & Format(dblMinAbsPhErr, "#.00") & ": "
     intPhaseError = 0;
	return TRUE;
}

// Function to establish symbol sync 
int EnvelopeCorrelatorOld()
{
	// Compute the two symbol correlation with the Two tone leader template.
	// slide the correlation one sample and repeat up to 240 steps 
	// keep the point of maximum or minimum correlation...and use this to identify the the symbol start. 

	float dblCorMax  = -1000000.0f;		//  Preset to excessive values
	float dblCorMin  = 1000000.0f;
	int intJatMax = 0, intJatMin = 0;
	float dblCorSum, dblCorProduct, dblCorMaxProduct = 0.0;
	int i,j;
	short int75HzFiltered[720];

	if (intFilteredMixedSamplesLength < intMFSReadPtr + 720)
		return -1;
	
	Filter75Hz(int75HzFiltered, TRUE, 720); // This filter appears to help reduce avg decode distance (10 frames) by about 14%-19% at WGN-5 May 3, 2015
	
	for (j = 0; j < 360; j++)		// Over 1.5 symbols
	{
		dblCorSum = 0;
		for (i = 0; i < 240; i++)	 // over 1 50 baud symbol (may be able to reduce to 1 symbol)
		{
			dblCorProduct = int50BaudTwoToneLeaderTemplate[i] * int75HzFiltered[120 + i + j]; // note 120 accomodates filter delay of 120 samples
			dblCorSum += dblCorProduct;
            if (fabsf(dblCorProduct) > dblCorMaxProduct)
				dblCorMaxProduct = fabsf(dblCorProduct);
		}

		if (fabsf(dblCorSum) > dblCorMax)
		{
			dblCorMax = fabsf(dblCorSum);
			intJatMax = j;
		}		
	}
	
	if (AccumulateStats)
	{
		dblAvgCorMaxToMaxProduct = (dblAvgCorMaxToMaxProduct * intEnvelopeCors + (dblCorMax / dblCorMaxProduct)) / (intEnvelopeCors + 1);
		intEnvelopeCors++;
	}
 
	if (dblCorMax > 40 * dblCorMaxProduct)
	{
		WriteDebugLog(LOGDEBUG, "EnvelopeCorrelator CorMax:MaxProd= %f  J= %d", dblCorMax / dblCorMaxProduct, intJatMax);
		return intJatMax;
	}
	else
		return -1;
}
 

int EnvelopeCorrelator()
{
	// Compute the two symbol correlation with the Two tone leader template.
	// slide the correlation one sample and repeat up to 240 steps 
	// keep the point of maximum or minimum correlation...and use this to identify the the symbol start. 

	float dblCorMax  = -1000000.0f;		//  Preset to excessive values
	int intJatMax = 0, intJatMin = 0;
	float dblCorSum, dblCorProduct, dblCorMaxProduct = 0.0;
	int i,j;
	short int75HzFiltered[960];

	if (intFilteredMixedSamplesLength < intMFSReadPtr + 960)
		return -1;
	
	Filter75Hz(int75HzFiltered, TRUE, 960); // This filter appears to help reduce avg decode distance (10 frames) by about 14%-19% at WGN-5 May 3, 2015
	
	for (j = 360; j < 600; j++)		// Over 2 symbols
	{
		dblCorSum = 0;
		for (i = 0; i < 240; i++)	 // over 1 50 baud symbol (may be able to reduce to 1 symbol)
		{
			dblCorProduct = int50BaudTwoToneLeaderTemplate[i] * int75HzFiltered[120 + i + j]; // note 120 accomdates filter delay of 120 samples
			dblCorSum += dblCorProduct;
            if (fabsf(dblCorProduct) > dblCorMaxProduct)
				dblCorMaxProduct = fabsf(dblCorProduct);
		}

		if (fabsf(dblCorSum) > dblCorMax)
		{
			dblCorMax = fabsf(dblCorSum);
			intJatMax = j;
		}		
	}
	
	if (AccumulateStats)
	{
		dblAvgCorMaxToMaxProduct = (dblAvgCorMaxToMaxProduct * intEnvelopeCors + (dblCorMax / dblCorMaxProduct)) / (intEnvelopeCors + 1);
		intEnvelopeCors++;
	}

	// The following test probably not needed....almost always seems to be about 60 
 
	if (dblCorMax > 40 * dblCorMaxProduct)
	{
		WriteDebugLog(LOGDEBUG, "EnvelopeCorrelator CorMax:MaxProd= %f  J= %d", dblCorMax / dblCorMaxProduct, intJatMax);
		return intJatMax;
	}
		
	WriteDebugLog(LOGDEBUG, "EnvelopeCorrelator failed");
	
	return -1;
}
 

//	Function to acquire the Frame Sync for all Frames 

BOOL AcquireFrameSyncRSB()
{
	// Two improvements could be incorporated into this function:
	//    1) Provide symbol tracking until the frame sync is found (small corrections should be less than 1 sample per 4 symbols ~2000 ppm)
	//    2) Ability to more accurately locate the symbol center (could be handled by symbol tracking 1) above. 

	//  This is for acquiring FSKFrameSync After Mixing Tones Mirrored around 1500 Hz. e.g. Reversed Sideband
	//  Frequency offset should be near 0 (normally within +/- 1 Hz)  
	//  Locate the sync Symbol which has no phase change from the prior symbol (BPSK leader @ 1500 Hz)   

	int intLocalPtr = intMFSReadPtr;
	int intAvailableSymbols = (intFilteredMixedSamplesLength - intMFSReadPtr) / 240;
	float dblPhaseSym1;	//' phase of the first symbol 
	float dblPhaseSym2;	//' phase of the second symbol 
	float dblPhaseSym3;	//' phase of the third symbol

	float dblReal, dblImag;
	float dblPhaseDiff12, dblPhaseDiff23;

	int i;

	if (intAvailableSymbols < 3)
		return FALSE;				// must have at least 360 samples to search
 
	// Calculate the Phase for the First symbol 
	
	GoertzelRealImag(intFilteredMixedSamples, intLocalPtr, 240, 30, &dblReal, &dblImag); // Carrier at 1500 Hz nominal Positioning with no cyclic prefix
	dblPhaseSym1 = atan2f(dblImag, dblReal);
	intLocalPtr += 240;	// advance one symbol
	GoertzelRealImag(intFilteredMixedSamples, intLocalPtr, 240, 30, &dblReal, &dblImag); // Carrier at 1500 Hz nominal Positioning with no cyclic prefix
	dblPhaseSym2 = atan2f(dblImag, dblReal);
	intLocalPtr += 240;		// advance one symbol

	for (i = 0; i <=  intAvailableSymbols - 3; i++)
	{
		// Compute the phase of the next symbol  
	
		GoertzelRealImag(intFilteredMixedSamples, intLocalPtr, 240, 30, &dblReal, &dblImag); // Carrier at 1500 Hz nominal Positioning with no cyclic prefix
		dblPhaseSym3 = atan2f(dblImag, dblReal);
		// Compute the phase differences between sym1-sym2, sym2-sym3
		dblPhaseDiff12 = dblPhaseSym1 - dblPhaseSym2;
		if (dblPhaseDiff12 > M_PI)		// bound phase diff to +/- Pi
			dblPhaseDiff12 -= dbl2Pi;
		else if (dblPhaseDiff12 < -M_PI)
			dblPhaseDiff12 += dbl2Pi;

		dblPhaseDiff23 = dblPhaseSym2 - dblPhaseSym3;
		if (dblPhaseDiff23 > M_PI)		//  bound phase diff to +/- Pi
			dblPhaseDiff23 -= dbl2Pi;
		else if (dblPhaseDiff23 < -M_PI)
			dblPhaseDiff23 += dbl2Pi;

		if (fabsf(dblPhaseDiff12) > 0.6667f * M_PI && fabsf(dblPhaseDiff23) < 0.3333f * M_PI)  // Tighten the margin to 60 degrees
		{
//			intPSKRefPhase = (short)dblPhaseSym3 * 1000;

			intLeaderRcvdMs = (int)ceil((intLocalPtr - 30) / 12);	 // 30 is to accomodate offset of inital pointer for filter length. 
			intMFSReadPtr = intLocalPtr + 240;		 // Position read pointer to start of the symbol following reference symbol 
		
			if (AccumulateStats)
				intFrameSyncs += 1;		 // accumulate tuning stats
	
			//strDecodeCapture &= "Sync; Phase1>2=" & Format(dblPhaseDiff12, "0.00") & " Phase2>3=" & Format(dblPhaseDiff23, "0.00") & ": "
	
			return TRUE;	 // pointer is pointing to first 4FSK data symbol. (first symbol of frame type)
		}
		else
		{
			dblPhaseSym1 = dblPhaseSym2;           
			dblPhaseSym2 = dblPhaseSym3;
			intLocalPtr += 240;			// advance one symbol 
		}
	}

	intMFSReadPtr = intLocalPtr - 480;		 // back up 2 symbols for next attempt (Current Sym2 will become new Sym1)
	return FALSE;	
}



// Function to acquire the Frame Sync for all Frames using exponential averaging 
int AcquireFrameSyncRSBAvg()
{
	//	This new routine uses exponential averaging on the ptr reference leader phases to minimize noise contribution
	//	Needs optimization of filter values and decision thresholds with actual simulator at low S:N and multipath. 

	//	This is for acquiring FSKFrameSync After Mixing Tones Mirrored around 1500 Hz. e.g. Reversed Sideband
	//	Frequency offset should be near 0 (normally within +/- 1 Hz)  
	//	Locate the sync Symbol which has no phase change from the prior symbol (50 baud BPSK leader @ 1500 Hz)   

	int intLocalPtr = intMFSReadPtr;
	int intAvailableSymbols = (intFilteredMixedSamplesLength - intMFSReadPtr) / 240;
	float dblPhaseSym1;	//' phase of the first symbol 
	float dblPhaseSym2;	//' phase of the second symbol 
	float dblPhaseSym3;	//' phase of the third symbol

	float dblReal, dblImag;
	float dblPhaseDiff12, dblPhaseDiff23;

	int i;

	if (intAvailableSymbols < 3)
		return FALSE;				// must have at least 360 samples to search

	// Calculate the Phase for the First symbol 
	
	GoertzelRealImagHann120(intFilteredMixedSamples, intLocalPtr, 240, 30, &dblReal, &dblImag); // Carrier at 1500 Hz nominal Positioning with no cyclic prefix
	dblPhaseSym1 = atan2f(dblImag, dblReal);
	intLocalPtr += 240;	// advance one symbol
	GoertzelRealImagHann120(intFilteredMixedSamples, intLocalPtr, 240, 30, &dblReal, &dblImag); // Carrier at 1500 Hz nominal Positioning with no cyclic prefix
	dblPhaseSym2 = atan2f(dblImag, dblReal);
	intLocalPtr += 240;		// advance one symbol

 	for (i = 0; i <=  intAvailableSymbols - 3; i++)
	{
 		// Compute the phase of the next symbol  
	
		GoertzelRealImagHann120(intFilteredMixedSamples, intLocalPtr, 240, 30, &dblReal, &dblImag); // Carrier at 1500 Hz nominal Positioning with no cyclic prefix
		dblPhaseSym3 = atan2f(dblImag, dblReal);
		// Compute the phase differences between sym1-sym2, sym2-sym3
		dblPhaseDiff12 = dblPhaseSym1 - dblPhaseSym2;
		if (dblPhaseDiff12 > M_PI)		// bound phase diff to +/- Pi
			dblPhaseDiff12 -= dbl2Pi;
		else if (dblPhaseDiff12 < -M_PI)
			dblPhaseDiff12 += dbl2Pi;

		if (dblPhaseDiff1_2Avg > 9)
			dblPhaseDiff1_2Avg = fabsf(dblPhaseDiff12); // initialize the difference average after a prior detect
		else
			dblPhaseDiff1_2Avg = 0.75 * dblPhaseDiff1_2Avg + 0.25 * fabsf(dblPhaseDiff12);  // exponential average 
    
 		
		dblPhaseDiff23 = dblPhaseSym2 - dblPhaseSym3;
		if (dblPhaseDiff23 > M_PI)		//  bound phase diff to +/- Pi
			dblPhaseDiff23 -= dbl2Pi;
		else if (dblPhaseDiff23 < -M_PI)
			dblPhaseDiff23 += dbl2Pi;
			
             

		if (fabsf(dblPhaseDiff1_2Avg ) > (0.75 * M_PI) && fabsf(dblPhaseDiff23) < (0.25f * M_PI))  // Margin ~30 deg and 45 degrees
		{
			intLeaderRcvdMs = (int)ceil((intLocalPtr - 30) / 12);	 // 30 is to accomodate offset of inital pointer for filter length. 
			intMFSReadPtr = intLocalPtr + 240;		 // Position read pointer to start of the symbol following reference symbol 
		
			if (AccumulateStats)
				intFrameSyncs += 1;		 // accumulate tuning stats
	
			//strDecodeCapture &= "Sync; Phase1>2=" & Format(dblPhaseDiff12, "0.00") & " Phase2>3=" & Format(dblPhaseDiff23, "0.00") & ": "
        //            If MCB.DebugLog Then Logs.WriteDebug("[AcquireFrameSyncRSBAvg] Phase1>2Avg=" & Format(dblPhaseDiff1_2Avg, "#.00") & " Phase2>3=" & Format(dblPhaseDiff2_3, "#.00"))


// not used			dttLastLeaderSync = Now;
			dblPwrSNPower_dBPrior = -1000;  // Reset the prior Leader power to small value to insure minimum of two symbol passes on next leader detect. 
			return TRUE;	 // pointer is pointing to first 4FSK data symbol. (first symbol of frame type)
		}
		//	The following looks for phase errors (which should nomimally be Pi or 180 deg) and counts errors 
		//	abandoning search on the second error, Then advancing the main intMFSReadPtr one symbol (240 samples) and returning to SearchingForLeader state.
		    
		if (fabsf(dblPhaseDiff1_2Avg) < (0.6667 * M_PI) || fabsf(dblPhaseDiff23) < (0.6667 * M_PI)) // Margin 60 deg 
		{
			intPhaseError += 1;
			dblPhaseSym1 = dblPhaseSym2;           
			dblPhaseSym2 = dblPhaseSym3;
			intLocalPtr += 240;			// advance one symbol 
	 
			if (intPhaseError > 2) // This bailout mechanism for sync failure is superior and doesn't make any assumptions about leader length
			{
				intMFSReadPtr += 240; // advance the MFSReadPointer one symbol and try to search for leader again. 
				State = SearchingForLeader;
				return False;
			}
		}
		else
		{
			//	keep searching available samples

			dblPhaseSym1 = dblPhaseSym2;           
			dblPhaseSym2 = dblPhaseSym3;
			intLocalPtr += 240;			// advance one symbol 
		}
	}
 
	intMFSReadPtr = intLocalPtr - 480;		 // back up 2 symbols for next attempt (Current Sym2 will become new Sym1)
	return FALSE;

}
//	 Function to Demod FrameType4FSK

// Function to compute the "distance" from a specific bytFrame Xored by bytID using 1 symbol parity 

float ComputeDecodeDistance(int intTonePtr, int * intToneMags, UCHAR bytFrameType, UCHAR bytID)
{
	// intTonePtr is the offset into the Frame type symbols. 0 for first Frame byte 16 = (4 x 4) for second frame byte 

	float dblDistance = 0;
	int int4ToneSum;
	int intToneIndex;
	UCHAR bytMask = 0x30;
	int j, k;

	for (j = 0; j <= 3; j++)		//  over 4 symbols
	{
		int4ToneSum = 0;
		for (k = 0; k <=3; k++)
		{
			int4ToneSum += intToneMags[intTonePtr + (4 * j) + k];
		}
		if (int4ToneSum == 0)
			int4ToneSum = 1;		//  protects against possible overflow
		if (j < 3)
		    intToneIndex = ((bytFrameType ^ bytID) & bytMask) >> (4 - 2 * j);
		else
			intToneIndex = ComputeTypeParity(bytFrameType ^ bytID);

		dblDistance += 1.0f - ((1.0f * intToneMags[intTonePtr + (4 * j) + (3 - intToneIndex)]) / (1.0f * int4ToneSum));
		bytMask = bytMask >> 2;
	}
	
	dblDistance = dblDistance / 4;		// normalize back to 0 to 1 range 
	return dblDistance;
}

//	A function to check the parity symbol used in the frame type decoding

BOOL CheckTypeParity(UCHAR bytFrameType)
{
	// Returns True if Parity OK

	UCHAR bytMask = 0x30;	 // Look at only 6 bits of data (values only 0 to 63)
	UCHAR bytParitySum = 3;
	UCHAR bytSym = 0;
	int k;

	for (k = 0; k < 3; k++)
	{
		bytSym = (bytMask & bytFrameType) >> (2 * (2 - k));
		bytParitySum = bytParitySum ^ bytSym;
		bytMask = bytMask >> 2;
	}

	return bytParitySum == ((bytFrameType & 0x0C0) >> 6);
 }


// Function to check Parity of frame type bytes

UCHAR GetFrameTypeByte(int intTonePtr, int * intToneMags)
{
	// Demodulate the byte pointed to postion of tone PTR and return it
	 
	UCHAR bytData = 0, bytParity, bytSym;
    int intIndex = intTonePtr;
	int j;

	for (j = 0; j < 4; j++)
	{
		// for each 4FSK symbol (2 bits) in a byte

		if (intToneMags[intIndex] > intToneMags[intIndex + 1] && intToneMags[intIndex] > intToneMags[intIndex + 2] && intToneMags[intIndex] > intToneMags[intIndex + 3])
			bytSym = 3;
		else if (intToneMags[intIndex + 1] > intToneMags[intIndex] && intToneMags[intIndex + 1] > intToneMags[intIndex + 2] && intToneMags[intIndex + 1] > intToneMags[intIndex + 3])
			bytSym = 2;
		else if (intToneMags[intIndex + 2] > intToneMags[intIndex] && intToneMags[intIndex + 2] > intToneMags[intIndex + 1] && intToneMags[intIndex + 2] > intToneMags[intIndex + 3])
			bytSym = 1;
		else
			bytSym = 0;

		if (j < 3)
			bytData = (bytData << 2) + bytSym;
		else
			bytParity = bytSym << 6;
		
		intIndex += 4;
	}
	return bytData | bytParity;
} 


BOOL CheckFrameTypeParity(int intTonePtr, int * intToneMags)
{
	// Demodulate the byte pointed to postion of tone PTR and check Parity Return True if OK

	UCHAR bytData = GetFrameTypeByte(intTonePtr, intToneMags);

	return CheckTypeParity(bytData);
} 

//Determine the nominal 4PSK phases for a perfect decode Phase values -3146 to 3146 (milliradians) 

void Get4PSKPhaseTargets(UCHAR bytFrameCode, UCHAR bytID, short * intPhaseTargets)
{
        UCHAR bytEncodedFrameType[2];
		UCHAR bytMask;
        UCHAR bytLastSym = 0;
		UCHAR bytSymToDec = 0;
		int j;

        bytEncodedFrameType[0] = AddTypeParity(bytFrameCode);
        bytEncodedFrameType[1] = AddTypeParity(bytFrameCode ^ (bytSessionID & 0x3F));
  
		for (j = 0; j < 8; j++)		// 4PSK so 4 symbols per byte + reference
		{
            if (j % 4 == 0)
				bytMask = 0xC0;

			// Differential phase encocoding (add to last symbol sent Mod 4)  statement below confirmed 1/29/19

			if (j == 0)
                bytSymToDec = (bytEncodedFrameType[0] & bytMask) >> 6;
			else
                bytSymToDec = (bytEncodedFrameType[j / 4] & bytMask) >> (2 * (3 - (j % 4)));
  
            switch (bytSymToDec)
			{
			case 0:
				intPhaseTargets[j] = 0;
				break;

			case 1:
				intPhaseTargets[j] = 1573;
				break;

			case 2:
				intPhaseTargets[j] = 3146;  // Distance computation will accept +/- values near 3146
				break;

			case 3:

				intPhaseTargets[j] = -1573;
			}

            bytLastSym = bytSymToDec;
            bytMask = bytMask >> 2;
		}

//        If bytFrameCode = &H4 Or bytFrameCode = &H6 Then
//            bytFrameCode += 0

}
//	Function to compute the "distance" (total phase error in milliradians from perfect decode) from a specific bytFrame Xored by bytID using 1 symbol parity 
//	This version used for 50 Baud 4PSK frame type modulation 

float ComputePSKDecodeDistance(int intPhasePtr, short * intPhases, UCHAR bytFrameType, UCHAR bytID)  
{
	//intPhasePtr is the offset into the Frame type symbols. 0 for first FrameID byte,  4 for second frame byte 
    
	int intDistance = 0;		// distance in milliradians
    short intTargetPhase[8];
	int i, intSum = 0;
	
	Get4PSKPhaseTargets(bytFrameType, bytID, &intTargetPhase[0]);

	for (i = 0; i <= 3; i++)		//  over 4 symbols
	{
		intDistance = abs(intPhases[i + intPhasePtr] - intTargetPhase[i + intPhasePtr]);

		if (intDistance > 3142)
			intDistance = abs(6282 - intDistance);

		intSum += intDistance;
	}

	return intSum;  //return total absolute error distance (min = 0, max = 12584 milliradians)
}

//	Function to compute the frame type by selecting the minimal distance from all valid frame types.

int MinimalDistance4PSKFrameType(short * intPhases, UCHAR bytSessionID)
{
	float dblMinDistance1 = 100000; // minimal distance for the first byte initialize to large value
	float dblMinDistance2 = 100000; // minimal distance for the second byte initialize to large value
	float dblMinDistance3 = 100000; // minimal distance for the second byte under exceptional cases initialize to large value
	int intIatMinDistance1, intIatMinDistance2, intIatMinDistance3;
	float dblDistance1, dblDistance2, dblDistance3;
	int i;

	int intThresh = 1500 - (Squelch - 5) * 500;  // This adjusts the threshold based on MCB.Squelch (nominal = 5)

	strDecodeCapture[0] = 0;

	if (ProtocolState == ISS)
	{
		bytValidFrameTypes = bytValidFrameTypesISS;
		bytValidFrameTypesLength = bytValidFrameTypesLengthISS;
	}
	else
	{
		bytValidFrameTypes = bytValidFrameTypesALL;
		bytValidFrameTypesLength = bytValidFrameTypesLengthALL;
	}

	// Search through all the valid frame types finding the minimal distance 
	// This looks like a lot of computation but measured < 1 ms for 135 iterations....RM 11/1/2016

	for (i = 0; i < bytValidFrameTypesLength; i++)
	{
		dblDistance1 = ComputePSKDecodeDistance(0, intPhases, bytValidFrameTypes[i], 0);
		dblDistance2 = ComputePSKDecodeDistance(4, intPhases, bytValidFrameTypes[i], bytSessionID);

		if (blnPending)
		    dblDistance3 = ComputePSKDecodeDistance(4, intPhases, bytValidFrameTypes[i], 0x3F);
		else
			dblDistance3 = ComputePSKDecodeDistance(4, intPhases, bytValidFrameTypes[i], bytLastARQSessionID);

		if (dblDistance1 < dblMinDistance1)
		{
			dblMinDistance1 = dblDistance1;
			intIatMinDistance1 = bytValidFrameTypes[i];
		}
		if (dblDistance2 < dblMinDistance2)
		{
			dblMinDistance2 = dblDistance2;
			intIatMinDistance2 = bytValidFrameTypes[i];
		}
		if (dblDistance3 < dblMinDistance3)
		{
			dblMinDistance3 = dblDistance3;
			intIatMinDistance3 = bytValidFrameTypes[i];
		}
	}

	WriteDebugLog(LOGDEBUG, "Frame Decode type %x %x %x Dist %.2f %.2f %.2f Sess %x pend %d conn %d lastsess %d",
		intIatMinDistance1, intIatMinDistance2, intIatMinDistance3, 
		dblMinDistance1, dblMinDistance2, dblMinDistance3, 
		bytSessionID, blnPending, blnARQConnected, bytLastARQSessionID); 
	
	if (bytSessionID == 0x3F || blnPending)	
	{
		// 'we are in a FEC QSO, monitoring an ARQ session or have not yet reached the ARQ Pending or Connected status 

		if (intIatMinDistance1 == ConReq200 || intIatMinDistance1 == ConReq500 || intIatMinDistance1 == ConReq2500)
		{
			if (dblMinDistance1 < intThresh)
			{
				WriteDebugLog(LOGDEBUG, "[Frame Type OK]4 %s, D1= %d", Name(intIatMinDistance1), dblMinDistance1);
				return intIatMinDistance1;
			}
			else
				return -1;
		}
   
		if (intIatMinDistance2 == ConReq200 || intIatMinDistance2 == ConReq500 || intIatMinDistance2 == ConReq2500)
		{
			if (dblMinDistance2 < intThresh)
			{
				WriteDebugLog(LOGDEBUG, "[Frame Type OK]5 %s, D2= %d", Name(intIatMinDistance2), dblMinDistance2);
				return intIatMinDistance2;
			}
			else
				return -1;
		}

		if (dblMinDistance1 < intThresh && intIatMinDistance1 == ConAck)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]6 %s, D1= %d", Name(intIatMinDistance1), dblMinDistance1);
			return intIatMinDistance1;
		}

		if (blnPending && dblMinDistance2 < intThresh && intIatMinDistance2 == ConAck)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]7 %s, D2= %d", Name(intIatMinDistance2), dblMinDistance2);
			return intIatMinDistance2;
		}
 
		if (dblMinDistance1 < intThresh && intIatMinDistance1 == IDFRAME)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]8 %s, D1= %d", Name(intIatMinDistance1), dblMinDistance1);
			return intIatMinDistance1;
		}

		if (dblMinDistance2 < intThresh && intIatMinDistance2 == IDFRAME)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]9 %s, D2= %d", Name(intIatMinDistance2), dblMinDistance2);
			return intIatMinDistance2;
		}
 
		if (dblMinDistance1 < intThresh && intIatMinDistance1 == ACK)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]10 %s, D1= %d", Name(intIatMinDistance1), dblMinDistance1);
			return intIatMinDistance1;
		}

		if (dblMinDistance2 < intThresh && intIatMinDistance2 == ACK)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]11 %s, D2= %d", Name(intIatMinDistance2), dblMinDistance2);
			return intIatMinDistance2;
		}

		if (dblMinDistance1 < intThresh && intIatMinDistance1 == DISCFRAME)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]12 %s, D1= %d", Name(intIatMinDistance1), dblMinDistance1);
			return intIatMinDistance1;
		}

		if (dblMinDistance3 < intThresh && intIatMinDistance3 == DISCFRAME)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]13 %s, D3= %d", Name(intIatMinDistance3), dblMinDistance3);
			return intIatMinDistance3;
		}

		// this would handle the case of monitoring an ARQ connection where the SessionID is not &H3F 

		if (dblMinDistance1 < intThresh)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]14 %s, D1= %d", Name(intIatMinDistance1), dblMinDistance1);
			return intIatMinDistance1;
		}

		// this would handle the case of monitoring an FEC transmission that failed above when the session ID is = &H3F 

		if (dblMinDistance2 < intThresh)
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]15 %s, D2= %d", Name(intIatMinDistance2), dblMinDistance2);
			return intIatMinDistance2;
		}

		WriteDebugLog(LOGDEBUG, "[Frame Type Fail]16");	
		return -1;
	}

	if (blnARQConnected)		// ' we have an ARQ connected session.
	{
		if (AccumulateStats)
		{
			dblAvgDecodeDistance = (dblAvgDecodeDistance * intDecodeDistanceCount + 0.5f * (dblMinDistance1 + dblMinDistance2)) / (intDecodeDistanceCount + 1);
			intDecodeDistanceCount++;
		}
		// require matching index and at least one < threshold
		
		if (intIatMinDistance1 == intIatMinDistance2 && (dblMinDistance1 < intThresh || dblMinDistance2 < intThresh)) 		
		{
			WriteDebugLog(LOGDEBUG, "[Frame Type OK]17 %s,D1= %.2f, D2= %.2f %s",
				 Name(intIatMinDistance1), dblMinDistance1, dblMinDistance2, strDecodeCapture);

			return intIatMinDistance1;
		}
	}
	sprintf(strDecodeCapture, "%s MD Decode;18  Type1=H%X: Type2=H%X: , D1= %.2f, D2= %.2f",
		strDecodeCapture, intIatMinDistance1 , intIatMinDistance2, dblMinDistance1, dblMinDistance2);
	WriteDebugLog(LOGDEBUG, "[Frame Type Decode Fail] %s", strDecodeCapture);
	return -1; // indicates poor quality decode so  don't use
}


//	Function to acquire the 4FSK frame type

int Acquire4PSKFrameType()
{
	// intMFSReadPtr is pointing to start of first symbol of Frame Type (total of 8 4FSK symbols in frame type (2 bytes) + 1 parity symbol per byte 
	// returns -1 if minimal distance decoding is below threshold (low likelyhood of being correct)
	// returns -2 if insufficient samples 
	// Else returns frame type 0-255

	int NewType = 0;
	int Start = 0;
	int Save = CarrierOk[0];		// Save 

	if ((intFilteredMixedSamplesLength - intMFSReadPtr) < (240 * 10))
		return -2;		//  Check for 8 available 4FSK Symbols plus 2 reference symbols
	
	// This can't use the normal PSK decode at that include decode as well as demod
	// But can use  Demod1CarPSKChar 2 times

	strMod[0] = '4';
	intBaud = 50;
	intNumCar = 1;
	CarrierOk[0] = FALSE;

	InitDemodPSK();				// Gets reference phase

	Start = intSampPerSym;		// Used for reference

	Start += Demod1CarPSKChar(Start ,0);			// Does 4 symbols
	Start += Demod1CarPSKChar(Start, 0);

	CarrierOk[0] = Save;

	intRmtLeaderMeasure = (Now - dttStartRmtLeaderMeasure);

	// Now do check received  Tone array for testing minimum distance decoder

	if (blnPending)			// If we have a pending connection (btween the IRS first decode of ConReq until it receives a ConAck from the iSS)  
		NewType = MinimalDistance4PSKFrameType(&intPhases[0][0], bytPendingSessionID);		 // The pending session ID will become the session ID once connected) 
	else if (blnARQConnected)		// If we are connected then just use the stcConnection.bytSessionID
		NewType = MinimalDistance4PSKFrameType(&intPhases[0][0], bytSessionID);
	else					// not connected and not pending so use &FF (FEC or ARQ unconnected session ID
		NewType = MinimalDistance4PSKFrameType(&intPhases[0][0], 0x3F);
  
	if (NewType == ConReq200 || NewType ==  ConReq500 || NewType == ConReq2500 || NewType == PING)
		QueueCommandToHost("PENDING");			 // early pending notice to stop scanners

	if (NewType >= 0 &&  IsShortControlFrame(NewType))		// update the constellation if a short frame (no data to follow)
		intLastRcvdFrameQuality = UpdatePhaseConstellation(&intPhases[0][0], &intMags[0][0], "4PSK", FALSE);

	if (AccumulateStats)
		if (NewType >= 0)
			intGoodFSKFrameTypes++;
		else
			intFailedFSKFrameTypes++;

	if (IsShortControlFrame(NewType))
		return NewType;			// Frame decode complete no update of pointer needed 

	if (IsDataFrame(NewType))
		intMFSReadPtr += (240 * 9);	// Advance pointer to new reference symbol for all data frames
	else
		intMFSReadPtr += (240 * 8);	 // Advance pointer to the last symbol of frame type which will serve as the reference 
            // symbol for the following short run of 1 Carrier 4PSK 50 baud data.
            // This applies to: ConReq200, ConReq500, ConReq2500,  Ping, PingAck, ID, CQ 
	
	return NewType;
}


//	Demodulate Functions. These are called repeatedly as samples arrive
//	and buld a frame in static array  bytFrameData

// Function to demodulate one carrier for all low baud rate 4FSK frame types
 
//	Is called repeatedly to decode multitone modes
int Corrections = 0;


extern int intBW;

BOOL Decode4PSKConReq()
{
	UCHAR strTarget [32];
	UCHAR bytCall[6];
	BOOL FrameOK = FALSE;

	// Modified May 24, 2015 to use RS encoding vs CRC (similar to ID Frame)
 
	if (GenCRC8(&bytFrameData[0][0], 6) == bytFrameData[0][6])
	{
		FrameOK = TRUE;

		memcpy(bytCall, bytFrameData, 6);
		DeCompressCallsign(bytCall, strTarget);
	
		sprintf(strRcvFrameTag, "_ > %s", strTarget);
		sprintf(bytData, "%s", strTarget);
	}
	
	if (intFrameType == ConReq200)
		intBW = 200;
	else if (intFrameType == ConReq500)
		intBW = 500;
	else if (intFrameType == ConReq2500)
		intBW = 2500;

	if (!FrameOK)
		SendCommandToHost("CANCELPENDING");	

	return FrameOK;
}

// Feb 21, 2019  New Function to estimate S:N in Ping and ID frames

int EstimateSN()
{
	// Samples are in intMags

	float dblAvgMag = 0.0f;
	float dblAvgDeviation = 0.0f;
	float dblCalFactor = -20.0f;  // Chosen to cal with added noise at 10dB S:N (-20 looks good Feb 22, 2019) 
	int i, intSN;

	for (i = 0; i < intPhasesLen; i++)
	{
		dblAvgMag += intMags[0][i];
	}
	
	dblAvgMag = dblAvgMag / intPhasesLen;

	for (i = 0; i < intPhasesLen; i++)
	{
		dblAvgDeviation += powf((intMags[0][i] - dblAvgMag), 2) / intPhasesLen;
	}
	dblAvgDeviation = sqrtf(dblAvgDeviation);

	if (dblAvgDeviation == 0)
		dblAvgDeviation = 1.0f;

	intSN = dblCalFactor + (20 * log10f(dblAvgMag / dblAvgDeviation));

	return intSN;
}



// Function to Demodulate and Decode 1 carrier 4FSK 50 baud Ping frame  

BOOL Decode4FSKPing()
{
	UCHAR strCaller[10];
	UCHAR strTarget [10];
	UCHAR bytCall[6];
	BOOL blnRSOK;
	BOOL FrameOK;
	int intSNdB;

	FrameOK = RSDecode(bytFrameData[0], 16, 4, &blnRSOK);

	if (FrameOK && blnRSOK == FALSE)
	{
		// RS Claims to have corrected it, but check

		WriteDebugLog(LOGDEBUG, "PING Frame Corrected by RS");

		FrameOK = RSDecode(bytFrameData[0], 16, 4, &blnRSOK);

		// Should now be OK without connections, if not RS didn't fix it

		if (blnRSOK == FALSE)
		{
			WriteDebugLog(LOGDEBUG, "PING Still bad after RS");
			FrameOK = FALSE;
		}
	}

	memcpy(bytCall, bytFrameData[0], 6);
	DeCompressCallsign(bytCall, strCaller);
	memcpy(bytCall, &bytFrameData[0][6], 6);
	DeCompressCallsign(bytCall, strTarget);

//	printtick(strCaller);
//	printtick(strTarget);
	
	sprintf(strRcvFrameTag, "_%s > %s", strCaller, strTarget);
	sprintf(bytData, "%s %s", strCaller, strTarget);

	if (FrameOK == FALSE)
	{
		SendCommandToHost("CANCELPENDING");	
		return FALSE;
	}
	
	intSNdB = EstimateSN();
 
	if (ProtocolState == DISC)
	{
		char Msg[80];

		sprintf(Msg, "PING %s>%s %d %d", strCaller, strTarget, intSNdB, intLastRcvdFrameQuality);
		SendCommandToHost(Msg);

		WriteDebugLog(LOGDEBUG, "[DemodDecode4FSKPing] PING %s>%s S:N=%d Q=%d", strCaller, strTarget, intSNdB, intLastRcvdFrameQuality);
		
		stcLastPingdttTimeReceived = time(NULL);
		memcpy(stcLastPingstrSender, strCaller, 10);
		memcpy(stcLastPingstrTarget, strTarget, 10);
		stcLastPingintRcvdSN = intSNdB;
		stcLastPingintQuality = intLastRcvdFrameQuality;

		return TRUE;
	}
	else
		SendCommandToHost("CANCELPENDING");	

	return FALSE;
}




//  Function  Decode 1 carrier 4FSK 50 baud PingACK with S:N and Quality 
BOOL Decode4FSKPingACK(UCHAR bytFrameType, int * intSNdB, int * intQuality)
{
	int Ack = -1;

	if ((bytFrameData[0][0] == bytFrameData[0][1])|| (bytFrameData[0][0] == bytFrameData[0][2]))
		Ack = bytFrameData[0][0];
	else
		if (bytFrameData[0][1] == bytFrameData[0][2])
			Ack = bytFrameData[0][1];

	if (Ack >= 0)
	{
		*intSNdB = ((Ack & 0xF8) >> 3) - 10;	// Range -10 to + 21 dB steps of 1 dB
        *intQuality = (Ack & 7) * 10 + 30;		// Range 30 to 100 steps of 10
       
		if (*intSNdB == 21)
			WriteDebugLog(LOGDEBUG, "[DemodDecode4FSKPingACK]  S:N> 20 dB Quality=%d" ,*intQuality);
		else
			WriteDebugLog(LOGDEBUG, "[DemodDecode4FSKPingACK]  S:N= %d dB Quality=%d",  *intSNdB, *intQuality);

		blnPINGrepeating = False;
		blnFramePending = False;	//  Cancels last repeat
		return TRUE;
	}
	return FALSE;
}


BOOL Decode4PSKID(UCHAR bytFrameType, char * strCallID, char * strGridSquare)
{
	UCHAR bytCall[10];
	UCHAR temp[20];
	BOOL blnRSOK;
	BOOL FrameOK;

	// RS 

	FrameOK = RSDecode(bytFrameData[0], 16, 4, &blnRSOK);

	if (FrameOK && blnRSOK == FALSE)
	{
		// RS Claims to have corrected it, but check

		WriteDebugLog(LOGDEBUG, "ID Frame Corrected by RS");

		FrameOK = RSDecode(bytFrameData[0], 16, 4, &blnRSOK);

		// Should now be OK without connections, if not RS didn't fix it

		if (blnRSOK == FALSE)
		{
			WriteDebugLog(LOGDEBUG, "ID Still bad after RS");
			FrameOK = FALSE;
		}
	}

	memcpy(bytCall, bytFrameData, 6);
	DeCompressCallsign(bytCall, strCallID);
	memcpy(bytCall, &bytFrameData[0][6], 6);
	DeCompressGridSquare(bytCall, temp);
      
	if (strlen(temp) > 5)
	{
		temp[4] = tolower(temp[4]);
		temp[5] = tolower(temp[5]);
	}
	sprintf(strGridSquare, "[%s]", temp);

	intLastIDSNReceived = EstimateSN();

	if (AccumulateStats)
		if (FrameOK)
			intGoodFSKFrameDataDecodes++;
		else
			intFailedFSKFrameDataDecodes++;

	if (FrameOK)
		memcpy(lastGoodID, strCallID, 10);

	return FrameOK;	// Not correctable
}

// Function to Demodulate and Decode 1 carrier 4FSK 50 baud Ping frame  

BOOL Decode4FSKCQ_de(char * strCallID, char * strGridSquare)
{
	UCHAR bytCall[10];
	UCHAR temp[20];
	BOOL blnRSOK;
	BOOL FrameOK;

	FrameOK = RSDecode(bytFrameData[0], 16, 4, &blnRSOK);

	if (FrameOK && blnRSOK == FALSE)
	{
		// RS Claims to have corrected it, but check

		WriteDebugLog(LOGDEBUG, "ID Frame Corrected by RS");

		FrameOK = RSDecode(bytFrameData[0], 16, 4, &blnRSOK);

		// Should now be OK without connections, if not RS didn't fix it

		if (blnRSOK == FALSE)
		{
			WriteDebugLog(LOGDEBUG, "ID Still bad after RS");
			FrameOK = FALSE;
		}
	}

	memcpy(bytCall, bytFrameData, 6);
	DeCompressCallsign(bytCall, strCallID);
	memcpy(bytCall, &bytFrameData[0][6], 6);
	DeCompressGridSquare(bytCall, temp);
      
	if (strlen(temp) > 5)
	{
		temp[4] = tolower(temp[4]);
		temp[5] = tolower(temp[5]);
	}
	sprintf(strGridSquare, "[%s]", temp);

	if (AccumulateStats)
		if (FrameOK)
			intGoodFSKFrameDataDecodes++;
		else
			intFailedFSKFrameDataDecodes++;


	return FrameOK;	// Not correctable

}



 
//  Function to Demodulate Frame based on frame type
//	Will be called repeatedly as new samples arrive

void DemodulateFrame(int intFrameType)
{
 //       Dim stcStatus As Status = Nothing

	int intConstellationQuality = 0;

 //       ReDim bytData(-1)

	strRcvFrameTag[0] = 0;


	switch (intFrameType)
	{
	case ConReq200:
	case ConReq500:
	case ConReq2500:
	case PING:
	case IDFRAME:
	case PINGACK:
	case CQ_de:
	case MultiACK:
	case PktFrameHeader:	// Experimental Variable Length Frame 

		DemodPSK();
		return;

	case PktFrameData:	// Experimantal Variable Length Frame 
			
//		if (strcmp(strMod, "4FSK") == 0)
//			Demod1Car4FSK();
//		else
		DemodPSK();
		return;
	}


	DemodViterbiPSK();			// Are all data modes Viterbi ??
	return;


	WriteDebugLog(LOGDEBUG, "Unsupported frame type %x", intFrameType);
	DiscardOldSamples();
	ClearAllMixedSamples();
	State = SearchingForLeader;

	//		intFilteredMixedSamplesLength = 0;	// Testing

}


// Function to Strip quality from ACK/NAK frame types

BOOL DecodeACKNAK(int intFrameType, int *  intQuality)
{
	*intQuality = 38 + (2 * (intFrameType & 0x1F));  //mask off lower 5 bits ' Range of 38 to 100 in steps of 2
     // strRcvFrameTag = "_Q" & intQuality.ToString
	return TRUE;
}


int intSNdB = 0, intQuality = 0;


BOOL DecodeFrame(int xxx, UCHAR * bytData)
{
	BOOL blnDecodeOK = FALSE;
	char strCallerCallsign[10] = "";
	char strTargetCallsign[10] = "";
	char strIDCallSign[11] = "";
	char strGridSQ[20] = "";
	char Reply[80];
	
	strRcvFrameTag[0] = 0;

	//DataACK/NAK and short control frames 

	if (CarrierOk[0] != 0 && CarrierOk[0] != 1)
		CarrierOk[0] = 0;

	if (IsShortControlFrame(intFrameType)) // Short Control Frames
	{
		blnDecodeOK = TRUE;
		goto returnframe;
	}

	totalRSErrors = 0;
			
	if (IsDataFrame(intFrameType))
		PrintCarrierFlags();

	switch (intFrameType)
	{
	case ConReq200:
	case ConReq500:
	case ConReq2500:

		blnDecodeOK = Decode4PSKConReq();

		if (blnDecodeOK)
			DrawRXFrame(1, Name(intFrameType));
	
		break;

	case PINGACK:		 // 3D
 
		blnDecodeOK = Decode4FSKPingACK(intFrameType, &intSNdB, &intQuality);
			
		if (blnDecodeOK && ProtocolState == DISC && Now  - dttLastPINGSent < 5000)	
		{
			sprintf(Reply, "PINGACK %d %d", intSNdB, intQuality);
			SendCommandToHost(Reply);
		}

 		break;
    

	case IDFRAME:		 // ID FrameReply, 
						
		blnDecodeOK = Decode4PSKID(0x30, strIDCallSign, strGridSQ);
			
		frameLen = sprintf(bytData, "ID:%s %s: S:N %d" , strIDCallSign, strGridSQ, intLastIDSNReceived);
		
		if (blnDecodeOK)
			DrawRXFrame(1, bytData);

		break;
	
	case PING:	

		blnDecodeOK = Decode4FSKPing();
		break;

	case CQ_de:	

		blnDecodeOK = Decode4FSKCQ_de( strIDCallSign, strGridSQ);
		frameLen = sprintf(bytData, "CQ %s %s" , strIDCallSign, strGridSQ);

		break;

	case MultiACK:

		// Multicarrier ACK

		blnDecodeOK = ProcessMultiACK(&bytFrameData[0][0]);
		break;


	case PktFrameHeader:
	{
			// Variable Length Packet Frame Header
			// 6 bits Type 10 Bits Len

			int Len;
			int pktNumCar;
			int pktDataLen;
			int pktRSLen;
						
			frameLen = CorrectRawDataWithRS(&bytFrameData[0][0], bytData, intDataLen, intRSLen, intFrameType, 0);
		
			if (CarrierOk[0])
			{
					pktRXMode = bytFrameData[0][1] >> 2;
					pktNumCar = pktCarriers[pktRXMode];

					Len =  ((bytFrameData[0][1] & 0x3) << 8) | bytFrameData[0][2];
				}
//	Now only using one carrier

//				else if (CarrierOk[1])
//				{
//					pktRXMode = bytFrameData2[1] >> 5;
//					pktNumCar = ((bytFrameData2[1] & 0x1c) >> 2) + 1;
//					Len =  ((bytFrameData2[1] & 0x3) << 8) | bytFrameData2[2];
//				}
				else
				{
					// Cant decode
	
					DiscardOldSamples();
					ClearAllMixedSamples();
					break;
				}
								
				strcpy(strMod, &pktMod[pktRXMode][0]);

				// Reset to receive rest of frame

				pktDataLen = (Len + (pktNumCar - 1))/pktNumCar; // Round up

				// This must match the encode settings
				
				pktRSLen = pktDataLen >> 2;			// Try 25% for now
				if (pktRSLen & 1)
					pktRSLen++;						// Odd RS bytes no use

				if (pktRSLen < 4)
					pktRSLen = 4;					// At least 4

				SymbolsLeft = pktDataLen + pktRSLen + 3; // Data has crc + length byte
				State = AcquireFrame;
				intFrameType = PktFrameData;
				CarrierOk[1] = CarrierOk[0] = 0;
				charIndex = 0;	
				frameLen = 0;
				intPhasesLen = 0;
				intDataLen = pktDataLen;
				intRSLen = pktRSLen;
				intNumCar = pktNumCar;
				PSKInitDone = 0;

				WriteDebugLog(6, "Pkt Frame Header Type %s Len %d", strMod, Len);
				strlop(strMod, '/');
				blnDecodeOK = TRUE;

				return 0;
	}

					
	case PktFrameData:
	{
			if (pktFSK[pktRXMode])
			{
				// Need to Check RS

				frameLen = CorrectRawDataWithRS(&bytFrameData[0][0], bytData, intDataLen, intRSLen, intFrameType, 0);
				if (intNumCar > 1)
					frameLen +=  CorrectRawDataWithRS(&bytFrameData[1][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 1);

				if (intNumCar > 2)
				{
					frameLen +=  CorrectRawDataWithRS(&bytFrameData[2][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 2);
					frameLen +=  CorrectRawDataWithRS(&bytFrameData[3][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 3);
				}
			}

			if (memcmp(CarrierOk, Good, intNumCar) == 0)
			{
				blnDecodeOK = TRUE;

				// Packet Data  - if KISS interface ias active
				// Pass to Host as KISS frame, else pass to
				// Session code

				// Data in bytData  len in frameLen

#ifdef TEENSY
				L2Routine(bytData, frameLen, intLastRcvdFrameQuality, totalRSErrors, intNumCar, pktRXMode);
#else
//				if (PKTCONNECTED)
//					SendFrametoHost(bytData, frameLen);
//				else
					L2Routine(bytData, frameLen, intLastRcvdFrameQuality, totalRSErrors, intNumCar, pktRXMode);
#endif
			}
			break;
	}


	default:

		//	Others are odd/evenpairs, and are all PSK 

		// already decoded and checked

//		if (memcmp(CarrierOk, Good, intNumCar) == 0)
		{
			char Msg[64];
			char Carriers[16];

			blnDecodeOK = TRUE;

			if (intNumCar == 1)
				sprintf(Carriers, "%d", CarrierOk[0]);
			else if (intNumCar == 2)
				sprintf(Carriers,  "%d%d", CarrierOk[0], CarrierOk[1]);
			else if (intNumCar == 4)
				sprintf(Carriers, "%d%d%d%d", CarrierOk[0], CarrierOk[1], CarrierOk[2], CarrierOk[3]);
			else
				sprintf(Carriers, "%d%d%d%d%d%d%d%d%d%d",
			CarrierOk[0], CarrierOk[1], CarrierOk[2], CarrierOk[3], CarrierOk[4], CarrierOk[5], CarrierOk[6], CarrierOk[7], CarrierOk[8], CarrierOk[9]);
			
			sprintf(Msg, "%s : %s", Name(intFrameType), Carriers);

			if (memcmp(CarrierOk, Bad, intNumCar) == 0)
				DrawRXFrame(2, Msg);
			else
				DrawRXFrame(1, Msg);

		}
	}


	if (blnDecodeOK)
	{
		WriteDebugLog(LOGINFO, "[DecodeFrame] Frame: %s Decode PASS,   Constellation Quality= %d", Name(intFrameType),  intLastRcvdFrameQuality);
#ifdef PLOTCONSTELLATION
		if (intFrameType >= 0x30 && intFrameType <= 0x38)
			DrawDecode(lastGoodID);		// ID or CONREQ
		else
			DrawDecode("PASS");
		updateDisplay();
#endif
	}

	else
	{
		WriteDebugLog(LOGINFO, "[DecodeFrame] Frame: %s Decode FAIL,   Constellation Quality= %d", Name(intFrameType),  intLastRcvdFrameQuality);
#ifdef PLOTCONSTELLATION
		DrawDecode("FAIL");
		updateDisplay();
#endif
	}


returnframe:

	if (blnDecodeOK && IsDataFrame(intFrameType))
		bytLastReceivedDataFrameType = intFrameType;
	
//	if (DebugLog)
//		if (blnDecodeOK)
//			WriteDebugLog(LOGDEBUG, "[DecodeFrame] Frame: %s Decode PASS, Constellation Quality= %d", Name(intFrameType), intLastRcvdFrameQuality);
//		else
//			WriteDebugLog(LOGDEBUG, "[DecodeFrame] Frame: %s Decode FAIL, Constellation Quality= %d", Name(intFrameType), intLastRcvdFrameQuality);

	return blnDecodeOK;
}

// Subroutine to update the 4FSK Constellation

void drawFastVLine(int x0, int y0, int length, int color);
void drawFastHLine(int x0, int y0, int length, int color);

int UpdatePhaseConstellation(short * intPhases, short * intMag, char * strMod, BOOL blnQAM)
{
	// Subroutine to update bmpConstellation plot for PSK modes...
	// Skip plotting and calculations of intPSKPhase(0) as this is a reference phase (9/30/2014)

	int intPSKPhase = strMod[0] - '0';
	float dblPhaseError; 
	float dblPhaseErrorSum = 0;
	int intPSKIndex;
	int intP = 0;
	float dblRad = 0;
	float dblAvgRad = 0;
	float intMagMax = 0;
	float dblPi4 = 0.25 * M_PI;
	float dbPhaseStep;
	float dblRadError = 0;
	float dblPlotRotation = 0;
	int intRadInner = 0, intRadOuter = 0;
	float dblAvgRadOuter = 0, dblAvgRadInner = 0, dblRadErrorInner = 0, dblRadErrorOuter = 0;
 
	int i,j, k, intQuality;

#ifdef PLOTCONSTELLATION

	int intX, intY;
	int yCenter = (ConstellationHeight - 2)/ 2;
	int xCenter = (ConstellationWidth - 2) / 2;

	unsigned short clrPixel = WHITE;

	clearDisplay();
#endif

	if (intPSKPhase == 4)
		intPSKIndex = 0;
	else
		intPSKIndex = 1;

	if (blnQAM)
	{
		intPSKPhase = 8;
		intPSKIndex = 1;
		dbPhaseStep  = 2 * M_PI / intPSKPhase;
		for (j = 1; j < intPhasesLen; j++)   // skip the magnitude of the reference in calculation
		{
			intMagMax = max(intMagMax, intMag[j]); // find the max magnitude to auto scale
		}

		for (k = 4; k < intPhasesLen; k++)
		{
			if (intMag[k] < 0.75f * intMagMax)
			{
				dblAvgRadInner += intMag[k];
				intRadInner++;
			}
			else
			{
				dblAvgRadOuter += intMag[k];
				intRadOuter++;
			}
		}

		dblAvgRadInner = dblAvgRadInner / intRadInner;
		dblAvgRadOuter = dblAvgRadOuter / intRadOuter;
	}
	else
	{
		dbPhaseStep  = 2 * M_PI / intPSKPhase;
		for (j = 4; j < intPhasesLen; j++)   // skip the magnitude of the reference in calculation
		{
			intMagMax = max(intMagMax, intMag[j]); // find the max magnitude to auto scale
            dblAvgRad += intMag[j];	
		}
	}
           
	dblAvgRad = dblAvgRad / (intPhasesLen - 1); // the average radius

	for (i = 1; i <  intPhasesLen; i++)  // Don't plot the first phase (reference)
	{
		intP = round((0.001 * intPhases[i]) / dbPhaseStep);

		// compute the Phase and Radius errors
 
		if (intMag[i] > (dblAvgRadInner + dblAvgRadOuter) / 2) 
			dblRadErrorOuter += fabsf(dblAvgRadOuter - intMag[i]);
		else
			dblRadErrorInner += fabsf(dblAvgRadInner - intMag[i]);

		dblPhaseError = fabsf(((0.001 * intPhases[i]) - intP * dbPhaseStep)); // always positive and < .5 *  dblPhaseStep
		dblPhaseErrorSum += dblPhaseError;

#ifdef PLOTCONSTELLATION
		dblRad = PLOTRADIUS * intMag[i] / intMagMax; //  scale the radius dblRad based on intMagMax
		intX = xCenter + dblRad * cosf(dblPlotRotation + intPhases[i] / 1000.0f);
		intY = yCenter + dblRad * sinf(dblPlotRotation + intPhases[i] / 1000.0f);
    
		
		if (intX > 0 && intY > 0)
			if (intX != xCenter && intY != yCenter)
				mySetPixel(intX, intY, WHITE); // don't plot on top of axis
#endif
	}

	if (blnQAM) 
	{
//		intQuality = max(0, ((100 - 200 * (dblPhaseErrorSum / (intPhasesLen)) / dbPhaseStep))); // ignore radius error for (PSK) but include for QAM
		intQuality = max(0, (1 - (dblRadErrorInner / (intRadInner * dblAvgRadInner) + dblRadErrorOuter / (intRadOuter * dblAvgRadOuter))) * (100 - 200 * (dblPhaseErrorSum / intPhasesLen) / dbPhaseStep));

//		intQuality = max(0, ((100 - 200 * (dblPhaseErrorSum / (intPhasesLen)) / dbPhaseStep))); // ignore radius error for (PSK) but include for QAM
		
		if (AccumulateStats)
		{
			intQAMQualityCnts += 1;
			intQAMQuality += intQuality;
			intQAMSymbolsDecoded += intPhasesLen;
		}
	}
	else
	{
		intQuality =  max(0, ((100 - 200 * (dblPhaseErrorSum / (intPhasesLen)) / dbPhaseStep))); // ignore radius error for (PSK) but include for QAM
	
		if (AccumulateStats)
		{
			intPSKQualityCnts[intPSKIndex]++;
			intPSKQuality[intPSKIndex] += intQuality;
			intPSKSymbolsDecoded += intPhasesLen;
		}
	}	
#ifdef PLOTCONSTELLATION
	DrawAxes(intQuality, shortName(intFrameType), strMod);
#endif
	return intQuality;

}


// Subroutine to track 1 carrier 4FSK. Used for both single and multiple simultaneous carrier 4FSK modes.


VOID Track1Car4FSK(short * intSamples, int * intPtr, int intSampPerSymbol, float dblSearchFreq, int intBaud, UCHAR * bytSymHistory)
{
	// look at magnitude of the tone for bytHistory(1)  2 sample2 earlier and 2 samples later.  and pick the maximum adjusting intPtr + or - 1
	// this seems to work fine on test Mar 16, 2015. This should handle sample rate offsets (sender to receiver) up to about 2000 ppm

	float dblReal, dblImag, dblMagEarly, dblMag, dblMagLate;
	float dblBinToSearch = (dblSearchFreq - (intBaud * bytSymHistory[1])) / intBaud; //  select the 2nd last symbol for magnitude comparison


	GoertzelRealImag(intSamples, (*intPtr - intSampPerSymbol - 2), intSampPerSymbol, dblBinToSearch, &dblReal, &dblImag);
	dblMagEarly = powf(dblReal, 2) + powf(dblImag, 2);
	GoertzelRealImag(intSamples, (*intPtr - intSampPerSymbol), intSampPerSymbol, dblBinToSearch, &dblReal, &dblImag);
	dblMag = powf(dblReal, 2) + powf(dblImag, 2);
	GoertzelRealImag(intSamples, (*intPtr - intSampPerSymbol + 2), intSampPerSymbol, dblBinToSearch, &dblReal, &dblImag);
	dblMagLate = powf(dblReal, 2) + powf(dblImag, 2);

	if (dblMagEarly > dblMag && dblMagEarly > dblMagLate)
	{
		*intPtr --;
		Corrections--;
		if (AccumulateStats)
			intAccumFSKTracking--;
	}
	else if (dblMagLate > dblMag && dblMagLate > dblMagEarly)
	{
		*intPtr ++;
		Corrections++;
		if (AccumulateStats)
			intAccumFSKTracking++;
	}
}

//	Function to Decode one Carrier of PSK modulation 

//	Ideally want to be able to call on for each symbol, as I don't have the
//	RAM to build whole frame

//	Call for each set of 4 or 8 Phase Values

int pskStart = 0;


VOID Decode1CarPSK(int Carrier)
{
	unsigned int int24Bits;
	UCHAR bytRawData;
	int k;
	int Len = intPhasesLen;
	UCHAR * Decoded = &bytFrameData[Carrier][0];

	if (CarrierOk[Carrier])
		return;							// don't do it again

	pskStart = 0;
	charIndex = 0;

    	
	while (Len >= 0)
	{

		// Phase Samples are in intPhases

		switch (intPSKMode)
		{
		case 4:		// process 4 sequential phases per byte (2 bits per phase)

			for (k = 0; k < 4; k++)
			{
				if (k == 0)
					bytRawData = 0;
				else
					bytRawData <<= 2;
				
				if (intPhases[Carrier][pskStart] < 786 && intPhases[Carrier][pskStart] > -786)
				{
				}		// Zero so no need to do anything
				else if (intPhases[Carrier][pskStart] >= 786 && intPhases[Carrier][pskStart] < 2356)
					bytRawData += 1;
				else if (intPhases[Carrier][pskStart] >= 2356 || intPhases[Carrier][pskStart] <= -2356)
					bytRawData += 2;
				else
					bytRawData += 3;

				pskStart++;
			}

			Decoded[charIndex++] = bytRawData;
			Len -= 4;
			break;

		case 8: // Process 8 sequential phases (3 bits per phase)  for 24 bits or 3 bytes  

			//	Status verified on 1 Carrier 8PSK with no RS needed for High S/N

			//	Assume we check for 8 available phase samples before being called

			int24Bits = 0;

			for (k = 0; k < 8; k++)
			{
				int24Bits <<= 3;

				if (intPhases[Carrier][pskStart] < 393 && intPhases[Carrier][pskStart] > -393)
				{
				}		// Zero so no need to do anything
				else if (intPhases[Carrier][pskStart] >= 393 && intPhases[Carrier][pskStart] < 1179)
					int24Bits += 1;
				else if (intPhases[Carrier][pskStart] >= 1179 && intPhases[Carrier][pskStart] < 1965)
					int24Bits += 2;
				else if (intPhases[Carrier][pskStart] >= 1965 && intPhases[Carrier][pskStart] < 2751)
				int24Bits += 3;
				else if (intPhases[Carrier][pskStart] >= 2751 || intPhases[Carrier][pskStart] < -2751)
					int24Bits += 4;
				else if (intPhases[Carrier][pskStart] >= -2751 && intPhases[Carrier][pskStart] < -1965)
				int24Bits += 5;
				else if (intPhases[Carrier][pskStart] >= -1965 && intPhases[Carrier][pskStart] <= -1179)
					int24Bits += 6;
				else 
					int24Bits += 7;

				pskStart ++;
	
			}
			Decoded[charIndex++] = int24Bits >> 16;
			Decoded[charIndex++] = int24Bits >> 8;
			Decoded[charIndex++] = int24Bits;

			Len -= 8;
			break;
	
		default:
			return; //????
		}
	}
	return;
}

//	Function to compute PSK symbol tracking (all PSK modes, used for single or multiple carrier modes) 

int Track1CarPSK(int intCarFreq, char * strPSKMod, float dblUnfilteredPhase, BOOL blnInit)
{
	// This routine initializes and tracks the phase offset per symbol and adjust intPtr +/-1 when the offset creeps to a threshold value.
	// adjusts (by Ref) intPtr 0, -1 or +1 based on a filtering of phase offset. 
	// this seems to work fine on test Mar 21, 2015. May need optimization after testing with higher sample rate errors. This should handle sample rate offsets (sender to receiver) up to about 2000 ppm

	float dblAlpha = 0.3f; // low pass filter constant  may want to optimize value after testing with large sample rate error. 
		// (Affects how much averaging is done) lower values of dblAlpha will minimize adjustments but track more slugishly.

	float dblPhaseOffset;

	static float dblTrackingPhase = 0;
	static float dblModFactor;
	static float dblRadiansPerSample;  // range is .4188 @ car freq = 800 to 1.1195 @ car freq 2200
	static float dblPhaseAtLastTrack;
	static int intCountAtLastTrack;
	static float dblFilteredPhaseOffset;

	if (blnInit)
	{
		// dblFilterredPhase = dblUnfilteredPhase;
		dblTrackingPhase = dblUnfilteredPhase;
		
		if (strPSKMod[0] == '8' || strPSKMod[0] == '1')
			dblModFactor = M_PI / 4;
		else if (strPSKMod[0] == '4')
			dblModFactor = M_PI / 2;

		dblRadiansPerSample = (intCarFreq * dbl2Pi) / 12000.0f;
		dblPhaseOffset = dblUnfilteredPhase - dblModFactor * round(dblUnfilteredPhase / dblModFactor);
		dblPhaseAtLastTrack = dblPhaseOffset;
		dblFilteredPhaseOffset = dblPhaseOffset;
		intCountAtLastTrack = 0;
		return 0;
	}

	intCountAtLastTrack += 1;
	dblPhaseOffset = dblUnfilteredPhase - dblModFactor * round(dblUnfilteredPhase / dblModFactor);
	dblFilteredPhaseOffset = (1 - dblAlpha) * dblFilteredPhaseOffset + dblAlpha * dblPhaseOffset;

	if ((dblFilteredPhaseOffset - dblPhaseAtLastTrack) > dblRadiansPerSample)
	{
		//Debug.WriteLine("Filtered>LastTrack: Cnt=" & intCountAtLastTrack.ToString & "  Filtered = " & Format(dblFilteredPhaseOffset, "00.000") & "  Offset = " & Format(dblPhaseOffset, "00.000") & "  Unfiltered = " & Format(dblUnfilteredPhase, "00.000"))
		dblFilteredPhaseOffset = dblPhaseOffset - dblRadiansPerSample;
		dblPhaseAtLastTrack = dblFilteredPhaseOffset;
	
		if (AccumulateStats)
		{
			if (strPSKMod[0] == '1')	// 16QAM" Then
			{ 
				intQAMTrackAttempts++;
				intAccumQAMTracking--;
			}
			else
			{
				intPSKTrackAttempts++;
				intAccumPSKTracking--;
			}
		}
		return -1;
	}

	if ((dblPhaseAtLastTrack - dblFilteredPhaseOffset) > dblRadiansPerSample)
	{
		//'Debug.WriteLine("Filtered<LastTrack: Cnt=" & intCountAtLastTrack.ToString & "  Filtered = " & Format(dblFilteredPhaseOffset, "00.000") & "  Offset = " & Format(dblPhaseOffset, "00.000") & "  Unfiltered = " & Format(dblUnfilteredPhase, "00.000"))
		dblFilteredPhaseOffset = dblPhaseOffset + dblRadiansPerSample;
		dblPhaseAtLastTrack = dblFilteredPhaseOffset;

		if (AccumulateStats)
		{
			if (strPSKMod[0] == '1')	// 16QAM" Then
			{ 
				intQAMTrackAttempts++;
				intAccumQAMTracking++;
			}
			else
			{
				intPSKTrackAttempts++;
				intAccumPSKTracking++;
			}
		}
		return 1;
	}
	// 'Debug.WriteLine("Filtered Phase = " & Format(dblFilteredPhaseOffset, "00.000") & "  Offset = " & Format(dblPhaseOffset, "00.000") & "  Unfiltered = " & Format(dblUnfilteredPhase, "00.000"))

	return 0;
}
 
// Function to compute the differenc of two angles 

int ComputeAng1_Ang2(int intAng1, int intAng2)
{
	// do an angle subtraction intAng1 minus intAng2 (in milliradians) 
	// Results always between -3142 and 3142 (+/- Pi)

	int intDiff;

	intDiff = intAng1 - intAng2;

	if (intDiff < -3142)
		intDiff += 6284;
	else if (intDiff > 3142 )
		intDiff -= 6284;

	return intDiff;
}

// Subroutine to "rotate" the phases to try and set the average offset to 0. 

void CorrectPhaseForTuningOffset(short * intPhase, int intPhaseLength, char * strMod)
{
	// A tunning error of -1 Hz will rotate the phase calculation Clockwise ~ 64 milliradians (~4 degrees)
	//   This corrects for:
	// 1) Small tuning errors which result in a phase bias (rotation) of then entire constellation
	// 2) Small Transmitter/receiver drift during the frame by averaging and adjusting to constellation to the average. 
	//   It only processes phase values close to the nominal to avoid generating too large of a correction from outliers: +/- 30 deg for 4PSK, +/- 15 deg for 8PSK
	//  Is very affective in handling initial tuning error.  

	// This only works if you collect all samples before decoding them. 
	// Can I do something similar????

	short intPhaseMargin  = 2095 / intPSKMode; // Compute the acceptable phase correction range (+/-30 degrees for 4 PSK)
	short intPhaseInc = 6284 / intPSKMode;
	int intTest;
	int i;
	int intOffset, intAvgOffset, intAvgOffsetBeginning, intAvgOffsetEnd;
	int intAccOffsetCnt = 0, intAccOffsetCntBeginning = 0, intAccOffsetCntEnd = 0;
	int	intAccOffsetBeginning = 0, intAccOffsetEnd = 0, intAccOffset = 0;
    int intPSKMode;


	if (strcmp(strMod, "8PSK") == 0 || strcmp(strMod, "16QAM") == 0)
		intPSKMode = 8;
	else
		intPSKMode = 4;
       
	// Note Rev 0.6.2.4 The following phase margin value increased from 2095 (120 deg) to 2793 (160 deg) yielded an improvement in decode at low S:N

	intPhaseMargin  = 2793 / intPSKMode; // Compute the acceptable phase correction range (+/-30 degrees for 4 PSK)
	intPhaseInc = 6284 / intPSKMode;

	// Compute the average offset (rotation) for all symbols within +/- intPhaseMargin of nominal
            
	for (i = 0; i <  intPhaseLength; i++)
	{
		intTest = (intPhase[i] / intPhaseInc);
		intOffset = intPhase[i] - intTest * intPhaseInc;

		if ((intOffset >= 0 && intOffset <= intPhaseMargin) || (intOffset < 0 && intOffset >= -intPhaseMargin))
		{
			intAccOffsetCnt += 1;
			intAccOffset += intOffset;
			
			if (i <= intPhaseLength / 4)
			{
				intAccOffsetCntBeginning += 1;
				intAccOffsetBeginning += intOffset;
			}
			else if (i >= (3 * intPhaseLength) / 4)
			{
				intAccOffsetCntEnd += 1;
				intAccOffsetEnd += intOffset;
			}
		}
	}
	
	if (intAccOffsetCnt > 0)
		intAvgOffset = (intAccOffset / intAccOffsetCnt);
	if (intAccOffsetCntBeginning > 0)
		intAvgOffsetBeginning = (intAccOffsetBeginning / intAccOffsetCntBeginning);
	if (intAccOffsetCntEnd > 0)
		intAvgOffsetEnd = (intAccOffsetEnd / intAccOffsetCntEnd);
     
	//WriteDebugLog(LOGDEBUG, "[CorrectPhaseForOffset] Beginning: %d End: %d Total: %d",
		//intAvgOffsetBeginning, intAvgOffsetEnd, intAvgOffset);

	if ((intAccOffsetCntBeginning > intPhaseLength / 8) && (intAccOffsetCntEnd > intPhaseLength / 8))
	{
		for (i = 0; i < intPhaseLength; i++)
		{
			intPhase[i] = intPhase[i] - ((intAvgOffsetBeginning * (intPhaseLength - i) / intPhaseLength) + (intAvgOffsetEnd * i / intPhaseLength));
			if (intPhase[i] > 3142)
				intPhase[i] -= 6284;
			else if (intPhase[i] < -3142)
				intPhase[i] += 6284;
		}
		WriteDebugLog(LOGDEBUG, "[CorrectPhaseForTuningOffset] AvgOffsetBeginning=%d AvgOffsetEnd=%d AccOffsetCnt=%d/%d",
				intAvgOffsetBeginning, intAvgOffsetEnd, intAccOffsetCnt, intPhaseLength);
	}
	else if (intAccOffsetCnt > intPhaseLength / 2)
	{
		for (i = 0; i < intPhaseLength; i++)
		{
			intPhase[i] -= intAvgOffset;
			if (intPhase[i] > 3142)
				intPhase[i] -= 6284;
			else if (intPhase[i] < -3142)
				intPhase[i] += 6284;
		}
		WriteDebugLog(LOGDEBUG, "[CorrectPhaseForTuningOffset] AvgOffset=%d AccOffsetCnt=%d/%d",
				intAvgOffset, intAccOffsetCnt, intPhaseLength);

	}
}

// Function to Decode one Carrier of 16QAM modulation 

//	Call for each set of 4 or 8 Phase Values

float intCarMagThreshold[10] = {0};

float dbl16APSK_8_8_CarRatio = 0.5f;

//	Functions to demod all PSKData frames single or multiple carriers 


VOID InitDemodPSK()
{
	// Called at start of frame

	int i;
	float dblPhase, dblReal, dblImag;

	intPSKMode = 4;					// ALways decode 4 sets at a time
	PSKInitDone = TRUE;
	intPhasesLen = 0;

	if (intBaud == 50)
		intSampPerSym = 240;
	else if (intBaud == 100)
		intSampPerSym = 120;
	else
		intSampPerSym = 60;

	if (intNumCar == 1)
		intCarFreq = 1500;
	else
		intCarFreq = 1400 + (intNumCar / 2) * 200; // start at the highest carrier freq which is actually the lowest transmitted carrier due to Reverse sideband mixing
  
	for (i= 0; i < intNumCar; i++)
	{
		if (intBaud == 50)
		{
			intCP[i] = 0;
			intNforGoertzel[i] = 240;
            dblFreqBin[i] = intCarFreq / 50;
		}
		else if (intBaud == 100)
		{
			//Experimental use of Hanning Windowing
				
            intNforGoertzel[i] = 120;
            dblFreqBin[i] = intCarFreq / 100;
            intCP[i] = 0;
		}
 
		else if (intBaud == 200)
		{
			if (strstr(strMod, "4PSKC"))		// Cyclic Prefix
			{
	            intNforGoertzel[i] = 60;				// Using 72 samples
		        dblFreqBin[i] = intCarFreq / 200;
			    intCP[i] = 6;
				intSampPerSym = 72;
			}
			else
			{
	            intNforGoertzel[i] = 60;
		        dblFreqBin[i] = intCarFreq / 200;
			    intCP[i] = 0;
			}
		}


/*		if (intBaud == 100 && intCarFreq == 1500) 
		{
		intCP[i] = 20;  //  These values selected for best decode percentage (92%) and best average 4PSK Quality (82) on MPP0dB channel
		dblFreqBin[i] = intCarFreq / 150;
		intNforGoertzel[i] = 80;
		}
		else if (intBaud == 100)
		{
			intCP[i] = 28; // This value selected for best decoding percentage (56%) and best Averag 4PSK Quality (77) on mpg +5 dB
			intNforGoertzel[i] = 60;
			dblFreqBin[i] = intCarFreq / 200;
		}
		else if (intBaud == 167)
		{
			intCP[i] = 6;  // Need to optimize (little difference between 6 and 12 @ wgn5, 2 Car 500 Hz)
			intNforGoertzel[i] = 60;
			dblFreqBin[i] = intCarFreq / 200;
		}
*/	
		// Get initial Reference Phase		

		if (intBaud < 200)
			GoertzelRealImagHann120(intFilteredMixedSamples, 0, intNforGoertzel[i], dblFreqBin[i], &dblReal, &dblImag);
		else
			GoertzelRealImag(intFilteredMixedSamples, intCP[i] , intNforGoertzel[i], dblFreqBin[i], &dblReal, &dblImag);
            
		dblPhase = atan2f(dblImag, dblReal);
		Track1CarPSK(intCarFreq, strMod, dblPhase, TRUE);
		intPSKPhase_1[i] = -1000 * dblPhase;  // negative sign compensates for phase reverse after mixing
		dblPhase_1[i] =- dblPhase;		// For Virerbi

		// Set initial mag from Reference Phase (which should be full power)
		// Done here as well as in initQAM for pkt where we may switch mode midpacket

        intCarMagThreshold[i] = sqrtf(powf(dblReal, 2) + powf(dblImag, 2));
		intCarMagThreshold[i] *= 0.75;

		intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 
	}
}

int Demod1CarPSKChar(int Start, int Carrier);
void SavePSKSamples(int i);


short WeightedAngleAvg(short intAng1, short intAng2);

int CheckCarrierPairPSK(int Base, int Dup, int frameLen)
{
	int i, Len;
	
	WriteDebugLog(LOGDEBUG, "DemodPSK Carriers %d and %d", Base, Dup);

	Decode1CarPSK(Base);
	Len = CorrectRawDataWithRS(&bytFrameData[Base][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, Base);

	if (CarrierOk[Base])
	{
		// No need to decode 2nd

		CarrierOk[Dup] = 1;		// So FrameOk test is passed
		return Len + frameLen;
	}
	
	WriteDebugLog(LOGDEBUG, "DemodPSK Carrier %d bad, trying %d", Base, Dup);

	Decode1CarPSK(Dup);			// Decode Dup carrier
	Len =  CorrectRawDataWithRS(&bytFrameData[Dup][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, Base);	// Save as carrier 1
	
	if (CarrierOk[Base])
	{
		CarrierOk[Dup] = 1;		// So FrameOk test is passed
		bytFrameData[Base][0] = Len;
		memcpy(&bytFrameData[Base][1], &bytData[frameLen], Len);		// Any retry will use first copy without new decode
		return Len + frameLen;
	}


	// Try to average phases for the two carriers

	WriteDebugLog(LOGDEBUG, "DemodPSK both bad, trying average");

	for (i = 0; i <intPhasesLen; i++)
	{
		intPhases[Base][i] = WeightedAngleAvg(intPhases[Base][i], intPhases[Dup][i]);
	}
	
	// Try decode again on averages

	Decode1CarPSK(Base);
	Len = CorrectRawDataWithRS(&bytFrameData[Base][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, Base);

	if (CarrierOk[Base])
		CarrierOk[Dup] = 1;		// So FrameOk test is passed
		
	return Len + frameLen;
}

void DemodViterbiPSK()
{
	int Used[10] = {0};
	int Start = 0;
	int MemARQRetries = 0;
	
	intPSKMode = 4;		// Always docode blocks of 4 for now

	// We can't wait for the full frame as we don't have enough RAM, so
	// we do one DMA Buffer at a time, until we run out or end of frame

	// Only continue if we have enough samples
	
	while (State == AcquireFrame)
	{
		if (intFilteredMixedSamplesLength < intPSKMode * intSampPerSym + 10) // allow for a few phase corrections
		{
			// Move any unprocessessed data down buffer

			//	(while checking process - will use cyclic buffer eventually

			if (intFilteredMixedSamplesLength > 0 && Start > 0)
				memmove(intFilteredMixedSamples,
					&intFilteredMixedSamples[Start], intFilteredMixedSamplesLength * 2); 

			return;
		}
		
		if (PSKInitDone == 0)		// First time through
		{	
			if (intFilteredMixedSamplesLength < 2 * intPSKMode * intSampPerSym + 10) 
				return;				// Wait for at least 2 chars worth

			InitDemodPSK();
			intFilteredMixedSamplesLength -= intSampPerSym;
			Start += intSampPerSym;	
		}

		// If this is a multicarrier mode, we must call the
		// decode char routing for each carrier

		if (intNumCar == 1)
			intCarFreq = 1500;
		else
			intCarFreq = 1400 + (intNumCar / 2) * 200; // start at the highest carrier freq which is actually the lowest transmitted carrier due to Reverse sideband mixing

		Used[0] = Demod1CarViterbiPSK(Start, 0);

		if (intNumCar > 1)
		{
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[1] = Demod1CarViterbiPSK(Start, 1);
		}

		if (intNumCar > 2)
		{
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 
			Used[2] = Demod1CarViterbiPSK(Start, 2);
			
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 
			Used[3] = Demod1CarViterbiPSK(Start, 3);
		}

		if (intNumCar > 4)
		{
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[4] = Demod1CarViterbiPSK(Start, 4);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[5] = Demod1CarViterbiPSK(Start, 5);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[6] = Demod1CarViterbiPSK(Start, 6);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[7] = Demod1CarViterbiPSK(Start, 7);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[8] = Demod1CarViterbiPSK(Start, 8);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[9] = Demod1CarViterbiPSK(Start, 9);	
		}

		SymbolsLeft -= 4;		// number still to decode

		// If/when we reenable phase correstion we can take average of Used values.
		// ?? Should be also keep start value per carrier ??

		Start += Used[0];
		intFilteredMixedSamplesLength -= Used[0];

		if (intFilteredMixedSamplesLength < 0)
			WriteDebugLog(LOGERROR, "Corrupt intFilteredMixedSamplesLength");

		if (SymbolsLeft > 0)
			continue;	

		// Decode the Virerbi Samples

		DecodeCompleteTime = Now;

		if (strchr(strMod, 'R'))
			SetCodeRate("R=3/4");
		else
			SetCodeRate("R=1/2");

		if (memcmp(strMod, "16", 2) == 0)	// 16APSK
			totSymbols *= 2;

		DoErasureAndDecode(0);

//		CorrectPhaseForTuningOffset(&intPhases[0][0], intPhasesLen, strMod);
			
//		if (intNumCar > 1)
//			CorrectPhaseForTuningOffset(&intPhases[1][0], intPhasesLen, strMod);
			
		if (intNumCar > 2)
		{
//			CorrectPhaseForTuningOffset(&intPhases[2][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[3][0], intPhasesLen, strMod);
		}
		if (intNumCar > 4)
		{
//			CorrectPhaseForTuningOffset(&intPhases[4][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[5][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[6][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[7][0], intPhasesLen, strMod);
		}

		// Rick uses the last carier for Quality

		if (memcmp(strMod, "16", 2) == 0)	// 16APSK
			intLastRcvdFrameQuality = UpdatePhaseConstellation(&intPhases[intNumCar - 1][0], &intMags[intNumCar - 1][0], strMod, TRUE);
		else
			intLastRcvdFrameQuality = UpdatePhaseConstellation(&intPhases[intNumCar - 1][0], &intMags[intNumCar - 1][0], strMod, FALSE);

		// prepare for next

		State = SearchingForLeader;
		DiscardOldSamples();
		ClearAllMixedSamples();

		CheckAndCorrectCarrier(&bytFrameData[0][0], intDataLen, intRSLen, intFrameType, 0);
		
		if (intNumCar > 1)
		{
			DoErasureAndDecode(1);
			CheckAndCorrectCarrier(&bytFrameData[1][0], intDataLen, intRSLen, intFrameType,1 );
		}
		if (intNumCar > 2)
		{
			DoErasureAndDecode(2);
			DoErasureAndDecode(3);
			CheckAndCorrectCarrier(&bytFrameData[2][0], intDataLen, intRSLen, intFrameType, 2);
			CheckAndCorrectCarrier(&bytFrameData[3][0], intDataLen, intRSLen, intFrameType, 3);
		}
		if (intNumCar > 4)
		{
			DoErasureAndDecode(4);
			DoErasureAndDecode(5);
			DoErasureAndDecode(6);
			DoErasureAndDecode(7);
			DoErasureAndDecode(8);
			DoErasureAndDecode(9);
		
			CheckAndCorrectCarrier(&bytFrameData[4][0], intDataLen, intRSLen, intFrameType, 4);
			CheckAndCorrectCarrier(&bytFrameData[5][0], intDataLen, intRSLen, intFrameType, 5);
			CheckAndCorrectCarrier(&bytFrameData[6][0], intDataLen, intRSLen, intFrameType, 6);
			CheckAndCorrectCarrier(&bytFrameData[7][0], intDataLen, intRSLen, intFrameType, 7);
			CheckAndCorrectCarrier(&bytFrameData[8][0], intDataLen, intRSLen, intFrameType, 8);
			CheckAndCorrectCarrier(&bytFrameData[9][0], intDataLen, intRSLen, intFrameType, 9);
		}

		// If variable length packet frame header we only have header - leave rx running
		
		if (intFrameType == PktFrameHeader)
		{
			State = SearchingForLeader;
			
			// Save any unused samples
			
			if (intFilteredMixedSamplesLength > 0 && Start > 0)
				memmove(intFilteredMixedSamples,
					&intFilteredMixedSamples[Start], intFilteredMixedSamplesLength * 2); 

			return;
		}
	
#ifdef MEMORYARQ

		for (Carrier = 0; Carrier < intNumCar; Carrier++)
		{
			if (!CarrierOk[Carrier])
			{
				// Decode error - save data for MEM ARQ

				SavePSKSamples(Carrier);
				
				if (intSumCounts[Carrier] > 1)
				{
					Decode1CarQAM(Carrier); // try to decode based on the WeightedAveragePhases
					MemARQRetries++;
				}
			}
		}

		if (MemARQRetries)
		{
			// We've retryed to decode - see if ok now

			int OKNow = TRUE;

			WriteDebugLog(LOGDEBUG, "DemodPSK retry RS on MEM ARQ Corrected frames");
			frameLen = 0;
	
			for (Carrier = 0; Carrier < intNumCar; Carrier++)
			{
				frameLen += CorrectRawDataWithRS(bytFrameData[Carrier], bytData, intDataLen, intRSLen, intFrameType, Carrier);
				if (CarrierOk[Carrier] == 0)
					OKNow = FALSE;
			}

			if (OKNow && AccumulateStats) 
				intGoodPSKSummationDecodes++;
		}
#endif
	}
	return;
}




void DemodPSK()
{
	int Used[10] = {0};
	int Start = 0;
	int MemARQRetries = 0;

	// We can't wait for the full frame as we don't have enough RAM, so
	// we do one DMA Buffer at a time, until we run out or end of frame

	// Only continue if we have enough samples
	
	intPSKMode = strMod[0] - '0';

	while (State == AcquireFrame)
	{
		if (intFilteredMixedSamplesLength < intPSKMode * intSampPerSym + 10) // allow for a few phase corrections
		{
			// Move any unprocessessed data down buffer

			//	(while checking process - will use cyclic buffer eventually

			if (intFilteredMixedSamplesLength > 0 && Start > 0)
				memmove(intFilteredMixedSamples,
					&intFilteredMixedSamples[Start], intFilteredMixedSamplesLength * 2); 

			return;
		}
		

		if (PSKInitDone == 0)		// First time through
		{	
			if (intFilteredMixedSamplesLength < 2 * intPSKMode * intSampPerSym + 10) 
				return;				// Wait for at least 2 chars worth

			InitDemodPSK();
			intFilteredMixedSamplesLength -= intSampPerSym;
			Start += intSampPerSym;	
		}

		// If this is a multicarrier mode, we must call the
		// decode char routing for each carrier

		if (intNumCar == 1)
			intCarFreq = 1500;
		else
			intCarFreq = 1400 + (intNumCar / 2) * 200; // start at the highest carrier freq which is actually the lowest transmitted carrier due to Reverse sideband mixing

		Used[0] = Demod1CarPSKChar(Start, 0);

		if (intNumCar > 1)
		{
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[1] = Demod1CarPSKChar(Start, 1);
		}

		if (intNumCar > 2)
		{
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 
			Used[2] = Demod1CarPSKChar(Start, 2);
			
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 
			Used[3] = Demod1CarPSKChar(Start, 3);
		}

		if (intNumCar > 4)
		{
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[4] = Demod1CarPSKChar(Start, 4);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[5] = Demod1CarPSKChar(Start, 5);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[6] = Demod1CarPSKChar(Start, 6);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[7] = Demod1CarPSKChar(Start, 7);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[8] = Demod1CarPSKChar(Start, 8);
			intPhasesLen -= intPSKMode;
			intCarFreq -= 200;  // Step through each carrier Highest to lowest which is equivalent to lowest to highest before RSB mixing. 

			Used[9] = Demod1CarPSKChar(Start, 9);	
		}

		if (intPSKMode == 4)
			SymbolsLeft--;		// number still to decode
		else
			SymbolsLeft -=3;

		// If/when we reenable phase correstion we can take average of Used values.
		// ?? Should be also keep start value per carrier ??

		Start += Used[0];
		intFilteredMixedSamplesLength -= Used[0];

		if (intFilteredMixedSamplesLength < 0)
			WriteDebugLog(LOGERROR, "Corrupt intFilteredMixedSamplesLength");

		if (SymbolsLeft > 0)
			continue;	

		// Decode the phases

		DecodeCompleteTime = Now;

//		CorrectPhaseForTuningOffset(&intPhases[0][0], intPhasesLen, strMod);
			
//		if (intNumCar > 1)
//			CorrectPhaseForTuningOffset(&intPhases[1][0], intPhasesLen, strMod);
			
		if (intNumCar > 2)
		{
//			CorrectPhaseForTuningOffset(&intPhases[2][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[3][0], intPhasesLen, strMod);
		}
		if (intNumCar > 4)
		{
//			CorrectPhaseForTuningOffset(&intPhases[4][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[5][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[6][0], intPhasesLen, strMod);
//			CorrectPhaseForTuningOffset(&intPhases[7][0], intPhasesLen, strMod);
		}

		// Rick uses the last carier for Quality
		intLastRcvdFrameQuality = UpdatePhaseConstellation(&intPhases[intNumCar - 1][0], &intMags[intNumCar - 1][0], strMod, FALSE);

		// prepare for next

		State = SearchingForLeader;
		DiscardOldSamples();
		ClearAllMixedSamples();

		if (strchr(strMod, 'R'))
		{
			// Robust Mode - data is repeated (1-2 or 1-6, 2-7, etc

			if (intNumCar == 2)
			{
				frameLen = CheckCarrierPairPSK(0, 1, 0);
				return;
			}

			//Only have 2 or 10 (500 or 2500 modes)


			frameLen = CheckCarrierPairPSK(0, 5, 0);
			frameLen = CheckCarrierPairPSK(1, 6, frameLen);
			frameLen = CheckCarrierPairPSK(2, 7, frameLen);
			frameLen = CheckCarrierPairPSK(3, 8, frameLen);
			frameLen = CheckCarrierPairPSK(4, 9, frameLen);

			return;
		}

		// Non -robust


		Decode1CarPSK(0);

		// If ID, CONREQ etc there is no CRC so can't use normal path

		switch (intFrameType)
		{
		case IDFRAME:
		case PING:
		case CQ_de:
		case ConAck:
		case ConReq200:
		case ConReq500:
		case ConReq2500:
		case PINGACK:
		case MultiACK:

			return;
		}


		frameLen = CorrectRawDataWithRS(&bytFrameData[0][0], bytData, intDataLen, intRSLen, intFrameType, 0);
		
		if (intNumCar > 1)
		{
			Decode1CarPSK(1);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[1][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 1);
		}
		if (intNumCar > 2)
		{
			Decode1CarPSK(2);
			Decode1CarPSK(3);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[2][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 2);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[3][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 3);

		}
		if (intNumCar > 4)
		{
			Decode1CarPSK(4);
			Decode1CarPSK(5);
			Decode1CarPSK(6);
			Decode1CarPSK(7);
			Decode1CarPSK(8);
			Decode1CarPSK(9);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[4][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 4);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[5][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 5);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[6][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 6);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[7][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 7);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[8][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 8);
			frameLen +=  CorrectRawDataWithRS(&bytFrameData[9][0], &bytData[frameLen], intDataLen, intRSLen, intFrameType, 9);

		}

		// If variable length packet frame header we only have header - leave rx running
		
		if (intFrameType == PktFrameHeader)
		{
			State = SearchingForLeader;
			
			// Save any unused samples
			
			if (intFilteredMixedSamplesLength > 0 && Start > 0)
				memmove(intFilteredMixedSamples,
					&intFilteredMixedSamples[Start], intFilteredMixedSamplesLength * 2); 

			return;
		}
	
#ifdef MEMORYARQ

		for (Carrier = 0; Carrier < intNumCar; Carrier++)
		{
			if (!CarrierOk[Carrier])
			{
				// Decode error - save data for MEM ARQ

				SavePSKSamples(Carrier);
				
				if (intSumCounts[Carrier] > 1)
				{
					Decode1CarQAM(Carrier); // try to decode based on the WeightedAveragePhases
					MemARQRetries++;
				}
			}
		}

		if (MemARQRetries)
		{
			// We've retryed to decode - see if ok now

			int OKNow = TRUE;

			WriteDebugLog(LOGDEBUG, "DemodPSK retry RS on MEM ARQ Corrected frames");
			frameLen = 0;
	
			for (Carrier = 0; Carrier < intNumCar; Carrier++)
			{
				frameLen += CorrectRawDataWithRS(bytFrameData[Carrier], bytData, intDataLen, intRSLen, intFrameType, Carrier);
				if (CarrierOk[Carrier] == 0)
					OKNow = FALSE;
			}

			if (OKNow && AccumulateStats) 
				intGoodPSKSummationDecodes++;
		}
#endif
	}
	return;
}


// Function to demodulate one carrier for all Viterbi PSK frame types
int Demod1CarViterbiPSK(int Start, int Carrier)
{
	// Converts intSample to an array of differential phase and magnitude values for the Specific Carrier Freq
	// intPtr should be pointing to the approximate start of the first reference/training symbol (1 of 3) 
	// intPhase() is an array of phase values (in milliradians range of 0 to 6283) for each symbol 
	// intMag() is an array of Magnitude values (not used in PSK decoding but for constellation plotting or QAM decoding)
	// Objective is to use Minimum Phase Error Tracking to maintain optimum pointer position
    // note: Hann windowing possible on PSK modes if carrier spacing is 2 x baud rate. 

	//	This is called for one DMA buffer of samples (normally 1200)

	float dblReal, dblImag;
	int intMiliRadPerSample = intCarFreq * M_PI / 6;
	int i;
	int intNumOfSymbols = intPSKMode;
	int origStart = Start;
	float dblPhaseSymbol;

	if (CarrierOk[Carrier])		// Already decoded this carrier?
	{
		intPhasesLen += intNumOfSymbols;
		return intSampPerSym * intNumOfSymbols;
	}

	if (strstr(strMod, "16APSK"))
		return Demod1CarViterbi16APSK_8_8(Start, Carrier);
   
	for (i = 0; i < intNumOfSymbols; i++)
	{
		if (intBaud < 200)
			GoertzelRealImagHann120(intFilteredMixedSamples, Start, intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);
		else
			GoertzelRealImag(intFilteredMixedSamples, Start + intCP[Carrier], intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);
	
		intMags[Carrier][intPhasesLen] = sqrtf(powf(dblReal, 2) + powf(dblImag, 2));
		dblPhase_0[Carrier] = -atan2f(dblImag, dblReal);

		dblPhaseSymbol = dblPhase_0[Carrier] - dblPhase_1[Carrier];

		if (dblPhaseSymbol > M_PI)
			dblPhaseSymbol += -dbl2Pi;

		else if (dblPhaseSymbol < -M_PI)
			dblPhaseSymbol += dbl2Pi;
		
		intPhases[Carrier][intPhasesLen] = 1000 * dblPhaseSymbol; //????
		
		if (0 <= dblPhaseSymbol && dblPhaseSymbol < dblPi_2)
		{
			bytSoftIQ[Carrier][2 * intPhasesLen] = 28;
			bytSoftIQ[Carrier][2 * intPhasesLen + 1] = 28 + (200 * dblPhaseSymbol / dblPi_2);
		}
		else if (dblPi_2 <= dblPhaseSymbol &&  dblPhaseSymbol <= M_PI)
		{
			bytSoftIQ[Carrier][2 * intPhasesLen] = 28 + (200 * (dblPhaseSymbol - dblPi_2) / dblPi_2);
			bytSoftIQ[Carrier][2 * intPhasesLen + 1] = 228;
		}
		else if (-M_PI <= dblPhaseSymbol && dblPhaseSymbol < -dblPi_2)
		{
			bytSoftIQ[Carrier][2 * intPhasesLen] = 228;
			bytSoftIQ[Carrier][2 * intPhasesLen + 1] = 228 - (200 * (M_PI + dblPhaseSymbol) / dblPi_2);
		}
		else
		{
			bytSoftIQ[Carrier][2 * intPhasesLen] = 228 - (200 * (dblPi_2 + dblPhaseSymbol) / dblPi_2);
			bytSoftIQ[Carrier][2 * intPhasesLen + 1] = 28;
		}
		
		dblPhase_1[Carrier] = dblPhase_0[Carrier];		//
 
/*
		if (Carrier == 0)
		{
			Corrections = Track1CarPSK(intCarFreq, strMod, atan2f(dblImag, dblReal), FALSE);

			if (Corrections != 0)
			{
				Start += Corrections;

				if (intCP[i] == 0)
					GoertzelRealImagHanning(intFilteredMixedSamples, Start, intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);
				else
					GoertzelRealImag(intFilteredMixedSamples, Start + intCP[Carrier], intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);

				intPSKPhase_0[Carrier] = 1000 * atan2f(dblImag, dblReal);
			}
		}
*/
		intPhasesLen++;
		Start += intSampPerSym;
	
	}
	if (AccumulateStats)
		intPSKSymbolCnt += intNumOfSymbols;

	return (Start - origStart);	// Symbols we've consumed
}

UCHAR MagToIQ_8_8(float intMag, float dblMagRef)
{
	// For symmetrical inner/outer layers with true gray coding for phase 
	// Returns a value 228 to 28 based on the ratio of intMag to dblMagRef
	// if intMag = dblMagRef returns a value of 128 which is essentially an erasure of the IQ bit

	UCHAR bytMagAdj;
	float intNomMax = dblMagRef * 2.0f / (1 + dbl16APSK_8_8_CarRatio);
	float intNomMin  = dbl16APSK_8_8_CarRatio * intNomMax;

	if (intMag > dblMagRef)
		bytMagAdj = (128 + min(100, 100 * intMag / intNomMax)); // 128 to 228 (outer ring)
	else
		bytMagAdj = (128 - min(100, 100 * (dblMagRef - intMag) / (dblMagRef - intNomMin))); // 28 to 128 (inner ring)
 
	return bytMagAdj;
}

int Demod1CarViterbi16APSK_8_8(int Start, int Carrier)
{
	// Converts intSamples to an array of soft IQ pairs differential phase and magnitude values for the Specific Carrier Freq
	// intPtr should be pointing to the approximate start of the first reference/training symbol  

	// intMag() is an array of Magnitude values (used in 16APSK decoding)
	// Objective is to use Minimum Phase Error Tracking to maintain optimum pointer position

	float dblReal, dblImag;
	int intMiliRadPerSample = intCarFreq * M_PI / 6;
	int i;
	int intNumOfSymbols = 4;
	int origStart = Start;
	float dblPhaseSymbol;
	float dblMagRef = intCarMagThreshold[Carrier];
	float intMag;
	float dblAlpha = 0.1f;	 // need to optimize  values .1 to .3 appear to be good for WGN  .5 to .8 for flat fading
	float dblMagDecisionThreshold;  

	if (CarrierOk[Carrier])		// Already decoded this carrier?
	{
		intPhasesLen += intNumOfSymbols;
		return intSampPerSym * intNumOfSymbols;
	}

	dblMagDecisionThreshold = (1 + dbl16APSK_8_8_CarRatio) * 0.5; // mid way between nominal mag values values

	for (i = 0; i < intNumOfSymbols; i++)
	{
		if (intBaud < 200)
			GoertzelRealImagHann120(intFilteredMixedSamples, Start, intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);
		else
			GoertzelRealImag(intFilteredMixedSamples, Start, intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);
	
		intMags[Carrier][intPhasesLen] = intMag = sqrtf(powf(dblReal, 2) + powf(dblImag, 2));
		dblPhase_0[Carrier] = -atan2f(dblImag, dblReal);

		dblPhaseSymbol = dblPhase_0[Carrier] - dblPhase_1[Carrier];

		if (dblPhaseSymbol > M_PI)
			dblPhaseSymbol += -dbl2Pi;

		else if (dblPhaseSymbol < -M_PI)
			dblPhaseSymbol += dbl2Pi;
		
		intPhases[Carrier][intPhasesLen] = 1000 * dblPhaseSymbol; //????
 
		
		// Need to work on magnitude ref tracking

		if (intMag >= dblMagRef)
			dblMagRef = (1 - dblAlpha) * dblMagRef + dblAlpha * (dblMagDecisionThreshold * intMag); // Exponential averager (TODO optimize alpha for averager) 
		else
			dblMagRef = (1 - dblAlpha) * dblMagRef + (dblAlpha * (dblMagDecisionThreshold * intMag / dbl16APSK_8_8_CarRatio)); //TODO  Optimize alpha value 
	
	
		// Now MagToIQ_8_8 generate soft IQ amplitude values for bytSoftIQ based on inner to outer Gray scale decoding


		// Positive dblPhaseSymbols inner and outer rings

		if (0 <= dblPhaseSymbol && dblPhaseSymbol < dblPi_4)
		{
			// Sym 0 to Pi/4

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);  // Sets Car Mag IQ value to range of 28 to 228
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 28;
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 28;
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 28 + 200 * (dblPhaseSymbol / dblPi_4);
		}
		else if (dblPi_4 <= dblPhaseSymbol && dblPhaseSymbol < dblPi_2)
		{
			// Sym Pi/4 to Pi/2 

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 28;
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 28 + (200 * (dblPhaseSymbol - dblPi_4) / dblPi_4);
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 228;
		}
		
		else if (dblPi_2 <= dblPhaseSymbol && dblPhaseSymbol < dblPi_3_4)
		{

			// Sym Pi/2 to 3Pi/4

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 28;
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 228;
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 228 - (200 * (dblPhaseSymbol - dblPi_2) / dblPi_4);
		}
		else if (dblPi_3_4 <= dblPhaseSymbol && dblPhaseSymbol <= M_PI)
		{
			// Sym 3Pi/4 to Pi

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 28 + (200 * (dblPhaseSymbol - dblPi_3_4) / dblPi_4);
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 228;
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 28;
		}
		
		//	Negative dblPhaseSymbols inner and outer rings

		else if (-M_PI <= dblPhaseSymbol && dblPhaseSymbol < -dblPi_3_4)
		{
			// Sym -Pi to -3Pi/4

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 228;
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 228;
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 28 + 200 * (dblPhaseSymbol + M_PI) / dblPi_4;
		}
		else if (-dblPi_3_4 <= dblPhaseSymbol && dblPhaseSymbol < -dblPi_2)
		{
			// Sym -3Pi/4 to -Pi/2 

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 228;
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 228 - (200 * (dblPhaseSymbol + dblPi_3_4) / dblPi_4);
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 228;
		}
		else if (-dblPi_2 <= dblPhaseSymbol && dblPhaseSymbol < -dblPi_4)
		{
			// Sym -Pi/2 to -Pi/4

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 228;
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 28;
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 228 - (200 * (dblPhaseSymbol + dblPi_2) / dblPi_4);
		}
		else 
		{
			// Sym -Pi/4 to 0

			bytSoftIQ[Carrier][4 * intPhasesLen] = MagToIQ_8_8(intMag, dblMagRef);   // 228
			bytSoftIQ[Carrier][4 * intPhasesLen + 1] = 228 - 200 * (dblPhaseSymbol + dblPi_4) / dblPi_4;
			bytSoftIQ[Carrier][4 * intPhasesLen + 2] = 28;
			bytSoftIQ[Carrier][4 * intPhasesLen + 3] = 28;
		}

		dblPhase_1[Carrier] = dblPhase_0[Carrier];		//

		intPhasesLen++;
		Start += intSampPerSym;
	}
	if (AccumulateStats)
		intPSKSymbolCnt += intNumOfSymbols;

	intCarMagThreshold[Carrier] = dblMagRef;

	return (Start - origStart);	// Symbols we've consumed
}

// Function to demodulate one carrier for all PSK frame types
int Demod1CarPSKChar(int Start, int Carrier)
{
	// Converts intSample to an array of differential phase and magnitude values for the Specific Carrier Freq
	// intPtr should be pointing to the approximate start of the first reference/training symbol (1 of 3) 
	// intPhase() is an array of phase values (in milliradians range of 0 to 6283) for each symbol 
	// intMag() is an array of Magnitude values (not used in PSK decoding but for constellation plotting or QAM decoding)
	// Objective is to use Minimum Phase Error Tracking to maintain optimum pointer position
    // note: Hann windowing possible on PSK modes if carrier spacing is 2 x baud rate. 

	//	This is called for one DMA buffer of samples (normally 1200)

	float dblReal, dblImag;
	int intMiliRadPerSample = intCarFreq * M_PI / 6;
	int i;
	int intNumOfSymbols = intPSKMode;
	int origStart = Start;;

	if (intNumOfSymbols == 1)
		intNumOfSymbols = 8;

	if (CarrierOk[Carrier])		// Already decoded this carrier?
	{
		intPhasesLen += intNumOfSymbols;
		return intSampPerSym * intNumOfSymbols;
	}

	for (i = 0; i <  intNumOfSymbols; i++)
	{
		GoertzelRealImagHann120(intFilteredMixedSamples, Start, intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);
	
		intMags[Carrier][intPhasesLen] = sqrtf(powf(dblReal, 2) + powf(dblImag, 2));
		intPSKPhase_0[Carrier] = -1000 * atan2f(dblImag, dblReal);
		intPhases[Carrier][intPhasesLen] = (ComputeAng1_Ang2(intPSKPhase_0[Carrier], intPSKPhase_1[Carrier]));

/*
		if (Carrier == 0)
		{
			Corrections = Track1CarPSK(intCarFreq, strMod, atan2f(dblImag, dblReal), FALSE);

			if (Corrections != 0)
			{
				Start += Corrections;

				if (intCP[i] == 0)
					GoertzelRealImagHanning(intFilteredMixedSamples, Start, intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);
				else
					GoertzelRealImag(intFilteredMixedSamples, Start + intCP[Carrier], intNforGoertzel[Carrier], dblFreqBin[Carrier], &dblReal, &dblImag);

				intPSKPhase_0[Carrier] = 1000 * atan2f(dblImag, dblReal);
			}
		}
*/
		intPSKPhase_1[Carrier] = intPSKPhase_0[Carrier];
		intPhasesLen++;
		Start += intSampPerSym;
	
	}
	if (AccumulateStats)
		intPSKSymbolCnt += intNumOfSymbols;

	return (Start - origStart);	// Symbols we've consumed
}


//	Function to average two angles using magnitude weighting

short WeightedAngleAvg(short intAng1, short intAng2)
{
	// Ang1 and Ang 2 are in the range of -3142 to + 3142 (miliradians)
	// works but should come up with a routine that avoids Sin, Cos, Atan2
    // Modified in Rev 0.3.5.1 to "weight" averaging by intMag1 and intMag2 (why!!!)

	float dblSumX, dblSumY;

	dblSumX = cosf(intAng1 / 1000.0) + cosf(intAng2 / 1000.0);
	dblSumY = sinf(intAng1 / 1000.0) + sinf(intAng2 / 1000.0);
        
	return (1000 * atan2f(dblSumY, dblSumX));
}

#ifdef MEMORYARQ

void SaveQAMSamples(int i)
{
	int m;

	if (intSumCounts[i] == 0)
	{
		// First try - initialize Sum counts Phase average and Mag Average 

		for (m = 0; m < intPhasesLen; m++)
		{
			intCarPhaseAvg[i][m] = intPhases[i][m];
			intCarMagAvg[i][m] = intMags[i][m];
		}
	}
	else
	{
		for (m = 0; m < intPhasesLen; m++)
		{
			intCarPhaseAvg[i][m] = WeightedAngleAvg(intCarPhaseAvg[i][m], intPhases[i][m]);
			intPhases[i][m] = intCarPhaseAvg[i][m];
			// Use simple weighted average for Mags 
			intCarMagAvg[i][m] = (intCarMagAvg[i][m] * intSumCounts[i] + intMags[i][m]) / (intSumCounts[i] + 1);		
			intMags[i][m] = intCarMagAvg[i][m];
		}
	}
	intSumCounts[i]++;
}

void SavePSKSamples(int i)
{
	int m;

	if (intSumCounts[i] == 0)
	{
		// First try - initialize Sum counts Phase average and Mag Average 

		for (m = 0; m < intPhasesLen; m++)
		{
			intCarPhaseAvg[i][m] = intPhases[i][m];
		}
	}
	else
	{
		for (m = 0; m < intPhasesLen; m++)
		{
			intCarPhaseAvg[i][m] = WeightedAngleAvg(intCarPhaseAvg[i][m], intPhases[i][m]);
			intPhases[i][m] = intCarPhaseAvg[i][m];
		}
	}
	intSumCounts[i]++;
}			

#endif
