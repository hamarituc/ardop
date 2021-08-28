
// "ACKbyCarrier" support for ARDOP

#include "ARDOPC.h"

//	Terminology

//	Frame - Info sent as one transmission. Consists of one or more Packets on one or more carriers
//	Packet - Info sent on one carrier. Each Packet has a Packet Sequence Number (PSN)
//	Carrier - Also info sent on one carrier, but Packets can be assigned to any carrier


void Flush();
int Get4PSKSym(UCHAR * bytSymbols28_228);
int Get16APSKSym_8_8(UCHAR * Sym, float * dblMag);
BOOL SetCodeRate(char * strRate);
void SendLeaderAndSYNC(UCHAR * bytEncodedBytes, int intLeaderLen);
int SqLawCompressor(float Sample);
VOID Encode4PSKFrameType(UCHAR bytFrameCode, UCHAR bytSessionID, UCHAR * bytReturn);
int CorrectRawDataWithRS(UCHAR * bytRawData, UCHAR * bytCorrectedData, int intDataLen, int intRSLen, int bytFrameType, int Carrier);
VOID Decode1CarPSK(int Carrier);
void DrawTXFrame(const char * Frame);
VOID Gearshift(float AckPercent, BOOL SomeAcked);
void CorrectPhaseForTuningOffset(short * intPhase, int intPhaseLength, char * strMod);
UCHAR GenCRC6(int Value);



extern unsigned char encstate;
extern unsigned char encstates[10];
extern float dblEx;
extern float PeakValue;
extern int intAmp;
extern BOOL SendingHeader200;	// Set when sending header in 200 Hz Modes
extern UCHAR bytSessionID;
extern int dttTimeoutTrip;

extern char strMod[16];

extern char CarrierOk[10];		// RS OK Flags per carrier
extern char Good[10];			// All Set for comparing with CarrierOK

extern short intPhases[10][1000];	// We will decode as soon as we have 4 or 8 depending on mode
								//	(but need one set per carrier)
								// 652 is 163 * 4 (16QAM 100 Baud)

extern int intPhasesLen;
extern int intPSKMode;

extern UCHAR bytData[MAXDATALEN * WINDOW];
extern int frameLen;
extern int intNumCar;

extern unsigned short  ModeHasWorked[16];		// used to attempt to make gear shift more stable.
extern unsigned short  ModeHasBeenTried[16];
extern unsigned short  ModeNAKS[16];
extern unsigned short  CarrierACKS[16];
extern unsigned short  CarrierNAKS[16];
extern int intFrameTypePtr;
extern int modmaxSample;

extern UCHAR * bytFrameTypesForBW;		// Holds the byte array for Data modes for a session bandwidth.  First is most robust, last is fastest
extern int * bytFrameLengthsForBW;	// Holds the byte count for Data modes for a session bandwidth.

extern int intRptInterval;		// Puncturing 2 for R=1/2 or 6 for R=3/4

extern int TempMode;			// Used to shift to more robust mode for short frames
extern int TempModeRetries;

int lastTXPtr = 0;

int NextPSN = 1;				// Next PSN to be allocated
int LastPSNAcked = WINDOW - 1;
int unackedByteCount = 0;

// Stats for gear shift.

int CarrierAcks = 0;
int CarrierNaks = 0;
int TotalCarriersSent = 0;

float RollingAverage = 0.0f;


// List of PSN's with unacked data

// We never send zero length data frames, so can use Len= 0 to
// indicate empty slot. We keep data here and modulate directly
// (I hope) so don't need bytEncodedBytes
// Or if simpler keep byte list of allocated PSN's

//	By saving data here we can speed up resending, as block with
//	RS is already built, but uses more space. May have to rethink if
//	Teensy is short of memory

UCHAR UnackedPSNList[WINDOW][MAXCARRIERLEN];

// Actually can't use length field of above, as packets can be acked but still need to be requeued -
// Only in sequence acked PSN's are passed to host, so we need a separate list of acked PSNs

UCHAR AckedPSNList[WINDOW]; 



int SentPSNList[10];			// PSN's allocated to current frame (in PSN order)

UCHAR Frameheader[2];			// Frame header, sent before stuff in SentPSNListByCarrier

int SentPSNListByCarrier[10];	// PSN's allocated to current frame (in carrier order)

BOOL DontSendNewData = FALSE;

int CarriersSent = 0;			// Carriers in TX'd frame - used in ACK processing

// Receive Code

// All packets go in here but only Packets received out of sequence
// remain - others are passed to host

// I don't think we ever need ReceivedPSNList and UnackedPSNList at
// the same time so could overlay to save RAM (4400) on Teensy

//#define ReceivedPSNList UnackedPSNList

UCHAR ReceivedPSNList[WINDOW][MAXCARRIERLEN];

int NextPSNToHost = 1;

int LastPSN;		// Save last one decoded (for FEC Test)

VOID ResetRXState()
{
	// New frame type. Reset RX fields

	int i;

	for (i = 0; i < WINDOW; i++)
	{
		ReceivedPSNList[i][1] = 0;
	}

	NextPSNToHost = 1;
}

VOID ResetTXState()
{
	// New frame type. Reset TX fields

	int i;

	for (i = 0; i < WINDOW; i++)
	{
		UnackedPSNList[i][1] = 0;
	}

	memset(SentPSNListByCarrier, 0, sizeof(SentPSNListByCarrier));
	memset(AckedPSNList, 0, sizeof(AckedPSNList));

	NextPSN = 1;				// Rick doesn't use zero
	LastPSNAcked = WINDOW - 1;
	DontSendNewData = FALSE;
	unackedByteCount = 0;

	// Reset Gear Shift Stuff

	RollingAverage = 50;
}

VOID ResetPSNState()
{
	// Reset all control variables at start of session

	ResetTXState();
	ResetRXState();

	CarrierAcks = CarrierNaks = TotalCarriersSent = 0;

	RollingAverage = 50.0f;
}

int GetNextFreePSN()
{
	// Returns the number to allocate to the next block of data

	if (NextPSN == WINDOW)
		NextPSN = 1;

	if (NextPSN == LastPSNAcked)	// unacked, so about to wrap sequence
		return -1;

	AckedPSNList[NextPSN] = 0;		// Make sure not flagged as acked

	return(NextPSN++);
}

int GetNextPSNToResend(int Start)
{
	// Returns the next PSN to retransmit
	// Packets are always resent from start of list, but
	// will be randomly allocated to carriers

	// send outstanding carriers backwards

	while ((UnackedPSNList[Start][1] == 0 || AckedPSNList[Start] == 1) && Start >= 0)
		Start--;
	
	return(Start);			// Will be -1 if nothing to resend
}

void GenCRC16Normal(char * Data, int Length)
{
	unsigned int CRC = GenCRC16(Data, Length);
//	CRC = compute_crc(Data, Length);



	// Put the two CRC bytes after the stop index

	Data[Length++] = (CRC >> 8);	 // MS 8 bits of Register
	Data[Length] = (CRC & 0xFF);	 // LS 8 bits of Register
}


// I think EncodeData sends repeats then anything that will fit from bytDataToSend

BOOL EncodeData(UCHAR bytFrameType)
{
	//	Build next Frame to Transmit

	//  Output is a byte array which includes:
	//  1) A 2 byte Header which include the Frame ID.  This will be sent using 4FSK at 50 baud. It will include the Frame ID and ID Xored by the Session bytID.
	//  2) n sections one for each carrier that will include all data (with FEC appended) for the entire frame. Each block will be identical in length.

	//  Each carrier starts with an 8 bit block number, which may not be sequential (each carrier is ack'ed separately)
	//  and may be repeated (for redundancy)

	int intNumCar, intBaud, intDataLen, intRSLen;

	int Length = bytDataToSendLength;
	UCHAR * Data  = bytDataToSend;

	BOOL blnOdd;
	char strMod[16];
	BOOL blnFrameTypeOK;
	int  totSymbols;
	int i, j;
	UCHAR * bytToRS = &bytEncodedBytes[2]; 
	int RepeatIndex = 0;			// used to duplicate data if too short to fill frame
	int PSN;
	BOOL MoretoResend = 1;			//	Set if nothing (more) to resend 
	int UnAckedBlockPtr;

	blnFrameTypeOK = FrameInfo(bytFrameType, &blnOdd, &intNumCar, strMod, &intBaud, &intDataLen, &intRSLen, &totSymbols);

	if (!blnFrameTypeOK || intDataLen == 0 )
	{
		WriteDebugLog(LOGDEBUG, "EncodeData Bad Frame Type %d ", bytFrameType);
		return 0;
	}

	WriteDebugLog(LOGDEBUG, "EncodeData Bytes to Send %d unackedByteCount %d", Length, unackedByteCount);

	if (Length == 0 && unackedByteCount == 0)
		return 0;

	//	Try not to let unacked count get too high when average ack count
	//	is getting near shift down threshold

	// this needs more real world data to optimise

//	if (RollingAverage < 65 && unackedByteCount > intDataLen * intNumCar)
//		DontSendNewData = 1;
//	else
//		DontSendNewData = 0;

	lastTXPtr = intFrameTypePtr;

	CarriersSent = intNumCar;
	TotalCarriersSent += intNumCar;
	
	//	Generate the 2 bytes for the frame type data

	Encode4PSKFrameType(bytFrameType, bytSessionID, Frameheader);

	UnAckedBlockPtr = WINDOW - 1;		// We send unacked blocks backwards

	WriteDebugLog(LOGDEBUG, "EncodeData Bytes to Send %d DontSendNewData %d", Length, DontSendNewData);

	// Often the first carrier is the only one missed, and if we repeat it first it will always
	// fail. So it would be good if logical block number 0 isn't always sent on carrier 0
	
	// The carrier number must match the block number so we can ack it. 

	for (i = 0; i < intNumCar; i++)		//  across all carriers
	{
		//	First Resend unacked data

		char * bytToRS;

repeatblocks:

		if (MoretoResend == 1)
		{
			PSN = GetNextPSNToResend(UnAckedBlockPtr);

			if (PSN != -1)		// Something to resend
			{
				SentPSNList[i] = PSN;
				UnAckedBlockPtr = PSN - 1;

				// As toggle could have changed must redo CRC and RS

				bytToRS = &UnackedPSNList[PSN][0];

				// Data + RS + 1 byte byteCount + 1 byte blockno + 2 Byte CRC

				GenCRC16FrameType(bytToRS, intDataLen + 2, bytFrameType); // calculate the CRC on the PRN + byte count + data bytes	
				RSEncode(bytToRS, bytToRS + intDataLen + 4, intDataLen + 4, intRSLen);  // Generate the RS encoding

				continue;			// Do next carrier
			}
			MoretoResend = 0;
		}

		// Send New Data

		if (DontSendNewData || Length == 0)
		{	
			UnAckedBlockPtr = WINDOW - 1;		// We send unacked blocks backwards
			MoretoResend = 1;		
			goto repeatblocks;
		}

		PSN = GetNextFreePSN();

		if (PSN == -1)
		{
			// Won't fit in Windows
				
			UnAckedBlockPtr = WINDOW - 1;	// Send Unacked again
			MoretoResend = 1;		
			goto repeatblocks;
		}

		SentPSNList[i] = PSN;

		// Build Frame

		bytToRS = &UnackedPSNList[PSN][0];

		if (bytSessionID == 0x3F)			// FEC, so set top bit of PSN
			PSN |= 0x80;

		bytToRS[0] = PSN;

		if (Length > intDataLen)
		{
			// Won't all fit, so send full length packet 
	
			bytToRS[1] = intDataLen;
			memcpy(&bytToRS[2], Data, intDataLen);
			unackedByteCount += intDataLen;
			BytesSent += intDataLen;
		}
		else
		{
			// Last bit

			bytToRS[1] = Length;

			memset(&bytToRS[2], 0, intDataLen);
			memcpy(&bytToRS[2], Data, Length);
			unackedByteCount += Length;
			BytesSent += Length;
		}

		// Data + RS + 1 byte byteCount + 1 byte blockno + 2 Byte CRC

		GenCRC16FrameType(bytToRS, intDataLen + 2, bytFrameType); // calculate the CRC on the PRN + byte count + data bytes	
		RSEncode(bytToRS, bytToRS + intDataLen + 4, intDataLen + 4, intRSLen);  // Generate the RS encoding
	
		// We could remove bytDataToSend here, but better to wait till all carriers built

		Data += bytToRS[1];		// Used
		Length -= bytToRS[1];	// Still to send
	}

	//	Remove anything sent from bytDataToSend. We have to put it back
	//	if we do a mode shift with data outstanding

	//	Data points to the next byte to send

	RemoveDataFromQueue(Data - bytDataToSend);

	//	We now have list of PSN's to send in SentPSNList
	
	// Randomly allocate to carriers

	j = rand() % intNumCar;

	for (i = 0; i < intNumCar; i++)
	{
		if (j >= intNumCar)
			j = 0;

		WriteDebugLog(LOGDEBUG, "Sending Carrier %d PSN %d Len %d", i, 
			UnackedPSNList[SentPSNList[j]][0],
			UnackedPSNList[SentPSNList[j]][1]);

		SentPSNListByCarrier[i] = SentPSNList[j++];

	}
	
	WriteDebugLog(LOGDEBUG, "unackedByteCount %d Next PSN %d", unackedByteCount, NextPSN);
	return TRUE;
}

void ModCarrierSet(int intLeaderLen)
{
	// Send Carrier Set in SentPSNListByCarrier

	int intNumCar, intBaud, intDataLen, intRSLen, intDataPtr, intSampPerSym, intDataBytesPerCar, totSymbols;
	BOOL blnOdd;

	int Type = Frameheader[0];

	float intSample;
	float maxSample = 0;
	int Sample;
	int prefixOffset  = 0;			// Set to 48 for "C" Modes

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

	// Viterbi encoded symbols. Because of puncturing we need to work in 3 byte chunks

	UCHAR bytViterbiSymbols[10][48];

	UCHAR bytLastSym[11]; // = {0}; // Holds the last symbol sent (per carrier). bytLastSym(4) is 1500 Hz carrier (only used on 1 carrier modes) 
 
	if (!FrameInfo(Type >> 2, &blnOdd, &intNumCar, strMod, &intBaud, &intDataLen, &intRSLen,  &totSymbols))
		return;

	intDataBytesPerCar = intDataLen + intRSLen + 4;		// 4 = PSN Len CRC

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
		else if (intNumCar == 2)
			dblEx = 0.707f;  // Change compression on 2 Car 16APSK
		else if (intNumCar == 4)
			dblEx = 0.6f;  // Change compression on 4 Car 16APSK
		else if (intNumCar == 10)
			dblEx = 0.6f;  // Change compression on 10 Car 16APSK
	}

	
	switch(intNumCar)
	{
	case 1:
		intCarStartIndex = 5;
		PeakValue = 30000;
		break;
	case 2:
		intCarStartIndex = 4;
		PeakValue = 30000 * 2;
		break;
	case 4:
		intCarStartIndex = 3;
		PeakValue = 30000 * 4;
		break;
	case 10:
		PeakValue = 30000 * 10;
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
	else if (intNumCar == 4)
		initFilter(2500,1500);			// 1000 Hz filter suspect
	else
		initFilter(2500,1500);

	if (intLeaderLen == 0)
		intLeaderLenMS = LeaderLength;
	else
		intLeaderLenMS = intLeaderLen;
	
	// Create the leader

	SendLeaderAndSYNC(Frameheader, intLeaderLen);
	SendingHeader200 = FALSE;

	bytLastSym[0] = 0;

	//	Send Frame Type. Don't Compress

	for (j = -1; j <  4 * 2; j++) // 4PSK so 4 symbols per byte + reference
	{
		if (j == -1)
			bytSymToSend = 0;	// Reference symbol 
		else
		{
			if (j % 4 == 0)
				bytMask = 0xC0;

			// Differential phase encocoding (add to last symbol sent Mod 4)  statement below confirmed 1/29/19

			bytSymToSend = (bytLastSym[0] + ((Frameheader[j / 4] & bytMask) >> (2 * (3 - (j % 4))))) % 4; // Horrible!!
		}		

		for (n = 0; n < 240; n++)  // all the samples of a symbols 
		{		
			if (bytSymToSend < 2)   // This uses only 90 degree values in the 16APSK table and the symmetry of the symbols to reduce the table size by a factor of 2
				intSample =  int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * bytSymToSend][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
			else	// for Sym values 2, 3
				intSample = -int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * (bytSymToSend - 2)][n % 120]; // double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
	
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

	if (strcmp(strMod, "4PSKC") == 0 || strcmp(strMod, "4PSKCR") == 0)
		prefixOffset = 48;

	intCarIndex = intCarStartIndex; // initialize the carrrier index

	for (n = 0; n < intSampPerSym; n++)  // Sum for all the samples of a symbols 
	{
		intSample = 0;
		intCarIndex = intCarStartIndex;  // initialize to correct starting carrier

		for (i = 0; i < intNumCar; i++)	// across all carriers
		{
			bytSymToSend = 0;  //  using non 0 causes error on first data byte 12/8/2014   ...Values 0-3  not important (carries no data).   (Possible chance for Crest Factor reduction?)       
			bytLastSym[intCarIndex] = 0;

			intSample += int16APSK_8_8_50bdCarTemplate[intCarIndex][0][(n + prefixOffset) % 120];
	
			intCarIndex ++;
			if (intCarIndex == 5)
				intCarIndex = 6;	// skip over 1500 Hz for multi carrier modes (multi carrier modes all use even hundred Hz tones)
		}

		Sample = SqLawCompressor(intSample);

		if (Sample > maxSample)
			maxSample = Sample;

		SampleSink(Sample);
	}
      
	// End of reference phase generation

	// Unlike ARDOP_WIN we send samples as they are created,
	// so we loop through carriers, then data bytes

	modmaxSample = 0;


	if (memcmp(strMod, "4PSK", 4) == 0)
	{
		for (j = 0; j < intDataBytesPerCar; j += 3)	//  for each set of 3 data symbols
		{
			int ViterbiSymbols;

			// Viterbi encode data for each carrier. We need to end up with an even number
			// and the same for each carrier, so if we are puncturing we must work in multiples of
			// 3 bytes (48 symbols) , which will puncture to 32 symbols (remove every 3d byte).
			
			// but beware of sending one or two xtra bytes which will
			// mess up response timing

			int actualSymbols = 48;

			if (intRptInterval == 6)	// Puncturing
				actualSymbols = 32;

			if ((j + 3) > intDataBytesPerCar)
			{
				if (intRptInterval == 6)	// Puncturing
					actualSymbols = (intDataBytesPerCar - j) * 11;
				else
					actualSymbols = (intDataBytesPerCar - j) * 16;
			}				

			for (i = 0; i < intNumCar; i++)
			{
				encstate = encstates[i];


				// Try to track down corruption on retry

				if (j == 0)
				{
					if (!CheckCRC16FrameType(&UnackedPSNList[SentPSNListByCarrier[i]][0], intDataLen + 2, Type >> 2))
						WriteDebugLog(LOGDEBUG, "Sending Carrier %d PSN %d Bad CRC", i, UnackedPSNList[SentPSNListByCarrier[i]][0]);

//					WriteDebugLog(LOGDEBUG, "Sending PSN %d\n%s", 
//						UnackedPSNList[SentPSNListByCarrier[i]][j],
//						&UnackedPSNList[SentPSNListByCarrier[i]][2]);
				}

				ViterbiSymbols = ViterbiEncode(
					&UnackedPSNList[SentPSNListByCarrier[i]][j],
					&bytViterbiSymbols[i][0], 3);

				encstates[i] = encstate;
			}

			// Send the Viterbi encoded symbols

			for (k = 0; k < actualSymbols; k += 2)
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
							intSample += int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * bytSymToSend][(n + prefixOffset) % 120]; //  double the symbol value during template lookup for 4PSK. (skips over odd PSK 8 symbols)
						else if (bytSymToSend < 4)
							intSample -= int16APSK_8_8_50bdCarTemplate[intCarIndex][2 * (bytSymToSend - 2)][(n + prefixOffset) % 120]; // subtract 2 from the symbol value before doubling and subtract value of table 
		
	
						if (n == intSampPerSym - 1)		// Last sample?
							bytLastSym[intCarIndex] = bytSymToSend;	
	
						intCarIndex += 1;
						if (intCarIndex == 5)
							intCarIndex = 6;  // skip over 1500 Hz carrier for multicarrier modes
				
					}	// Carriers
      	
					// Done all carriers - send sample

					Sample = SqLawCompressor(intSample);

					if (Sample > maxSample)
						maxSample = Sample;

					SampleSink(Sample);

				}	// Samples

			}  // 8 Viterbi symbols per byte

			// Done all samples for this symbol

		}
	}
	else if (memcmp(strMod, "16APSK", 5) == 0)
	{
		int ViterbiSymbols;

		for (j = 0; j < intDataBytesPerCar; j += 3)	//  for each data symbol 
		{ 		
			// Viterbi encode data for each carrier. We need to end up with an even number
			// and the same for each carrier, so if we are puncturing we must work in multiples of
			// 3 bytes (48 symbols) , which will puncture to 32 symbols (remove every 3d byte).

			int actualSymbols = 48;

			if ((j + 3) > intDataBytesPerCar)
				actualSymbols = (intDataBytesPerCar - j) * 16;

			for (i = 0; i < intNumCar; i++)
			{
				encstate = encstates[i];

				ViterbiSymbols = ViterbiEncode(
					&UnackedPSNList[SentPSNListByCarrier[i]][j],
					&bytViterbiSymbols[i][0], 3);

				encstates[i] = encstate;
			}

			for (k = 0; k < actualSymbols; k += 4)
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

					Sample = SqLawCompressor(intSample);

					if (intSample > maxSample)
						maxSample = intSample;

					SampleSink(Sample);

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

	WriteDebugLog(LOGDEBUG, "ModMax Sample %d ", modmaxSample);
}

VOID AckCarrier(int Carrier)
{
	// Mark the PSN corresponding to Carrier as acked

	int PSN = SentPSNListByCarrier[Carrier];

//	UnackedPSNList[PSN][1] = 0;

	AckedPSNList[PSN] = 1;

	WriteDebugLog(LOGDEBUG, "Acking Carrier %d PSN %d", Carrier, PSN);
}

#define Alpha 75.0f		// % for smoothed average

BOOL ProcessMultiACK(UCHAR * Msg)
{
	int i, Acks = 0, PSN;
	int bitmask = Msg[1] | Msg[0] << 8;
	UCHAR CRC = bitmask & 0x3F;			// bottom 6 bits;
	char Display[32] = "AckByCar ";

	bitmask = bitmask >> 6;

	if (CRC != GenCRC6(bitmask))
	{
		WriteDebugLog(LOGDEBUG, "MultiACK CRC Error");
		DrawRXFrame(2, "AckByCar");

		return FALSE;
	}

	switch(CarriersSent)
	{
	case 1:

		if ((bitmask & 0x1FF))
		{
			WriteDebugLog(LOGDEBUG, "Invalid MultiACK %x with CarriersSent = %d", bitmask, CarriersSent);
			DrawRXFrame(2, "AckByCar");
			return FALSE;
		}
		break;

	case 2:

		if ((bitmask & 0xFF))
		{
			WriteDebugLog(LOGDEBUG, "Invalid MultiACK %x with CarriersSent = %d", bitmask, CarriersSent);
			DrawRXFrame(2, "AckByCar");
			return FALSE;
		}
		break;

	case 4:

		if ((bitmask & 0x3F))
		{
			WriteDebugLog(LOGDEBUG, "Invalid MultiACK %x with CarriersSent = %d", bitmask, CarriersSent);
			DrawRXFrame(2, "AckByCar");
			return FALSE;
		}
		break;

	case 10:
		break;

	default:
		
		WriteDebugLog(LOGDEBUG, "Invalid MultiACK %x with CarriersSent = %d", bitmask, CarriersSent);
		DrawRXFrame(2, "AckByCar");
		return FALSE;
	} 

	for (i = 0; i < CarriersSent; i++)
	{
		if (bitmask & 0x200)
		{
			AckCarrier(i);
			Acks++;
			CarrierAcks++;
			strcat(Display, "1");
			CarrierACKS[lastTXPtr]++;
		}
		else
		{
			CarrierNaks++;
			strcat(Display, "0");	
			CarrierNAKS[lastTXPtr]++;
		}

		bitmask <<= 1;
	}

	DrawRXFrame(1, Display);


	// Mark of any sequential acked frames - IRS will pass these to host on next new frame

	PSN = LastPSNAcked;
	PSN++;

	if (PSN == WINDOW)
		PSN = 1;

	while (PSN != NextPSN)
	{
		if (AckedPSNList[PSN] == 1)
		{
			// In sequence frame

			WriteDebugLog(LOGDEBUG, "Removing PSN %d", PSN);
			AckedPSNList[PSN] = 0;
			unackedByteCount -= UnackedPSNList[PSN][1];

			// Also remove from Unacked list so we don't resend or requeue

			UnackedPSNList[PSN][1] = 0;

			LastPSNAcked =  PSN;

			PSN++;

			if (PSN == WINDOW)
				PSN = 1;
		}
		else
			break;					// All in sequence removed
	}

	if (Acks)
		ModeHasWorked[lastTXPtr]++;	// Used to stop hunting to impossible mode
	else
		ModeNAKS[lastTXPtr]++;

	// Calculate rolling average

	if (TempMode != -1)
	{
		// if at least one acked reset TempModeRetries
		
		if (Acks)
			TempModeRetries = 0;

		return TRUE;		// Don't include short frames in Gearshift
	}

	RollingAverage = (RollingAverage * Alpha) / 100.0f;
	
	RollingAverage += (Acks * (100.0f - Alpha)) / CarriersSent;

	Gearshift(RollingAverage, Acks);

 	return TRUE;
}

int GetUnackedDataCount()
{
	int totalBytes = 0;
	int PSN = NextPSN;     // is this right - loopthrough all not passed to host;

	PSN++;				// First Unacked

	if (PSN == WINDOW)
		PSN = 1;		// Skip zero 

	while (PSN != NextPSN)
	{
		totalBytes += UnackedPSNList[PSN][1];
		PSN++;
		if (PSN == WINDOW)
			PSN = 1;
	}
	
	return totalBytes;
}

VOID RequeueData()
{
	// Requeue data that has been send and not passed to host. Used
	// on mode shift

	// To make reasonably quick with small memory requirement
	// get total length to requeue, move data back down buffer
	// them fill in from front

	int totalBytes =  GetUnackedDataCount();
	UCHAR * ptr = bytDataToSend;
	int PSN;
	
	if (totalBytes == 0)
		return;				// nothing doing

	WriteDebugLog(LOGDEBUG, "Gear Shift - requeue %d Bytes", totalBytes);
	WriteDebugLog(LOGDEBUG, "bytDataToSendLength was %d", bytDataToSendLength);

	if (bytDataToSendLength)	// if anything queued
		memmove(&bytDataToSend[totalBytes], &bytDataToSend[0], bytDataToSendLength);

//	Move unacked data to front of bytDataToSend

	PSN = NextPSN;

	PSN++;				// First Unacked

	if (PSN == WINDOW)
		PSN = 1;

	while (PSN != NextPSN)
	{
		if (UnackedPSNList[PSN][1])
		{
			WriteDebugLog(LOGDEBUG, "Gear Shift - requeue PSN %d Len %d",
				PSN, UnackedPSNList[PSN][1]);
			
			memcpy(ptr, &UnackedPSNList[PSN][2], UnackedPSNList[PSN][1]);
			ptr += UnackedPSNList[PSN][1];
		}

		PSN++;
		if (PSN == WINDOW)
			PSN = 1;
	}

	bytDataToSendLength += totalBytes;
	WriteDebugLog(LOGDEBUG, "bytDataToSendLength now %d", bytDataToSendLength);

	unackedByteCount = 0;
}



BOOL CheckAndCorrectCarrier(char * bytFrameData, int intDataLen, int intRSLen, int intFrameType, int Carrier)
{
	unsigned char PSN;

	// Data is corrected im situe, so no need to copy
	int decodeLen = CorrectRawDataWithRS(bytFrameData, NULL , intDataLen, intRSLen, intFrameType, Carrier);

	// if decode fails try with a tuning offset correction 

//	if (CarrierOk[Carrier] == 0)
//	{
//		CorrectPhaseForTuningOffset(&intPhases[Carrier][0], intPhasesLen, strMod);
	
//		Decode1CarPSK(Carrier);
	
//		decodeLen = CorrectRawDataWithRS(bytFrameData, NULL , intDataLen + 1, intRSLen, intFrameType, Carrier);
//	}

	if (CarrierOk[Carrier] == 0)
		return FALSE;				//Still bad

	PSN = bytFrameData[0];

	// Rick uses PSN's above 128 for FEC

	// I don't think it is safe at the moment to pass monitored
	// ARQ Data to host ?? crashes Pi

	if (PSN < 129 && bytSessionID == 0x3F)		// FEC decode of ARQ Data
		return FALSE;

	if (PSN > 128 && (ProtocolMode == FEC || ((ProtocolState == DISC) && Monitor)))
	{
		PSN &= 0x7F;
	}

	// I don't think it is safe at the moment to pass monitored
	// ARQ Data to host


	// CRC check isn't perfect. At least we can check that PSN and Length
	// are reasonable

	if (PSN > 0 && PSN < WINDOW && decodeLen <=  intDataLen)
	{
		// copy data to correct place is RX PSN list

		LastPSN = PSN;

//		CarriersDecoded[RXOFDMMode]++;

		if (ReceivedPSNList[PSN][1] == 0)		// not got this one?
		{
			memcpy(&ReceivedPSNList[PSN][0], bytFrameData, decodeLen + 2);
		}
		return TRUE;
	}
	return FALSE;
}

int LastFECPSN = 1;

VOID PassGoodDataToHost(UCHAR Type)
{
	// Pass data to host till we find a missing PSN

	// Should this be different for FEC? - only send to host if 
	// all acked. Send rest as ERR frame on next toggle change

	// Best to stick with Rick's idea and only send on
	// receiving OVER (or toggle change??)

	UCHAR * ptr = bytData;
	int frameLen = 0;
	int len;
	
	if (bytSessionID == 0x3F)		// FEC
	{
		// Rick starts an FEC sequence at 1 and ends with OVER.

		if (Type == OVER || IsDataFrame(Type))
		{
			// Either a new type or end of block

			// Pass everything received to host

			int First = 1;
			int Last = MAXCAR;

			while (First != Last)
			{
				len = ReceivedPSNList[First][1];

				if (len)
				{
					memcpy(ptr, &ReceivedPSNList[First][2], len);
					ReceivedPSNList[First][1] = 0;		// Processed
					frameLen += len;
					ptr += len;
				}
				First++;
			}

			if (frameLen)
				AddTagToDataAndSendToHost(bytData, "FEC", frameLen); // only correct data in proper squence passed to host   
		}
		return;
	}
			
	// ARQ


	len = ReceivedPSNList[NextPSNToHost][1];

	WriteDebugLog(LOGDEBUG, "PassGoodDataToHost - NextPSNToHost %d Len %d", NextPSNToHost, len);

	while (len)				// Len nonzero
	{
//		WriteDebugLog(LOGDEBUG, "Sending to Host PSN %d Len %d", NextPSNToHost, len);
		memcpy(ptr, &ReceivedPSNList[NextPSNToHost][2], len);
		ptr += len;
		frameLen += len;
		BytesReceived += len;

		ReceivedPSNList[NextPSNToHost][1] = 0;			// Clear length on old frame

		NextPSNToHost++;
		if (NextPSNToHost == WINDOW)
			NextPSNToHost = 1;
	
		len = ReceivedPSNList[NextPSNToHost][1];
	}

	if (frameLen)
	{
		bytData[frameLen] = 0;
//		WriteDebugLog(LOGDEBUG, "Sending to Host %s", bytData);

		AddTagToDataAndSendToHost(bytData, "ARQ", frameLen); // only correct data in proper squence passed to host   
	}
}

// Compute a 6 bit CRC value  (Used in AckByCar to generate and check)

UCHAR GenCRC6(int Value)
{
	// Status April 29, 2019  Checked and confirmed OK on AckByCar at S:N of -9 dB without any failures.
	// For  CRC-t-ITU =    x^6 + x +1
	// Data is a 32 bit integer only the 10 LSBs are used
	// Returns the CRC6 value as the lower 6 bits in byte

	int intPoly = 0x21;		//  This implements the CRC polynomial  x^6 + x + 1
	int intRegister = 0x3F;
	int i;
        
	Value &= 0x3FF;			// Mask to 10 bits

	for (i = 9; i >= 0; i--)		//  for each bit processing MS bit first
	{
		int Bit = Value & 0x200;		// Top Bit

		if (intRegister & 0x20) // the MSB of the register is set
		{
			// Shift left, place data bit as LSB, then divide
			// Register := shiftRegister left shift 1
			// Register := shiftRegister xor polynomial

			if (Bit)
				intRegister = ((intRegister << 1) | 1) & 0x3f;
			else
				intRegister = ((intRegister << 1)) & 0x3f;

			intRegister = intRegister ^ intPoly;
		}
		else // the MSB is not set
		{
			// Register is not divisible by polynomial yet.
			//  Just shift left and bring current data bit onto LSB of shiftRegister
  			if (Bit)
				intRegister = ((intRegister << 1) | 1) & 0x3f;
			else
				intRegister = ((intRegister << 1)) & 0x3f;
		}
		Value <<= 1;
	}

	return intRegister;
}

VOID EncodeAndSendMulticarrierACK(UCHAR bytSessionID, int LeaderLength)
{
	int i;
	char Frame[16];
	unsigned int val = 0;
	unsigned int CRC;

	// Rick uses 6 bit CRC in bottom 6 bits
	// Top bit for carrier 0

	//	Generate the 2 bytes for the frame type data

	Encode4PSKFrameType(MultiACK, bytSessionID, Frame);

	for (i = 0; i < MAXCAR; i++)
	{
		val <<= 1;

		if (CarrierOk[i])
			val |= 1;	
	}

	if (val)
		dttTimeoutTrip = Now;					// Kick watchdog if we have any valid data
// ??	else
// ??	ModeNAKS[intFrameTypePtr]++;

	CRC = GenCRC6(val);

	val = CRC | val << 6;			// to upper 6 bits

	Frame[2] = val >> 8;
	Frame[3] = val;


	Mod1Car50Bd4PSK(Frame, 4, LeaderLength);		// only returns when all sent
               
}

