//	ARDOP Modem Decode Sound Samples

#include "ARDOPC.h"

extern BOOL blnAbort;

extern int NextPSN;

int intFECFramesSent;
int FECRepeatsSent;

UCHAR bytFrameType;
BOOL blnSendIDFrame;
extern BOOL NeedID;		// SENDID Command Flag
extern int intRepeatCount;
extern int unackedByteCount;

extern int intLastFrameIDToHost;
int bytFailedDataLength; 
extern int intLastFailedFrameID;
int crcLastFECDataPassedToHost;

UCHAR bytFailedData[1600];		// do we rally need that much ????

extern int intNumCar;
extern int intBaud;
extern int intDataLen;
extern int intRSLen;
extern int intSampleLen;
extern int intDataPtr;
extern int intSampPerSym;
extern int intDataBytesPerCar;
extern BOOL blnOdd;
extern char strType[18];
extern char strMod[16];
extern UCHAR bytMinQualThresh;

UCHAR bytLastFECDataFrameSent;

char strCurrentFrameFilename[16];

unsigned int dttLastFECIDSent;

extern int intCalcLeader;        // the computed leader to use based on the reported Leader Length

extern UCHAR UnackedPSNList[WINDOW][MAXCARRIERLEN];
extern int SentPSNListByCarrier[10];	// PSN's allocated to current frame (in carrier order)

// Function to start sending FEC data 

BOOL StartFEC(UCHAR * bytData, int Len, char * strDataMode, int intRepeats, BOOL blnSendID)
{
	// Return True if OK false if problem

	BOOL blnModeOK = FALSE;
	int i;

	FECRepeats = intRepeats;

	if (ProtocolState == FECSend)	// If already sending FEC data simply add to the OB queue
	{
		AddDataToDataToSend(bytData, Len);	// add new data to queue

		WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.StartFEC] %d bytes received while in FECSend state...append to data to send.", Len);
		return TRUE;
	}
	else
		dttLastFECIDSent = Now;
	
	//	Check to see that there is data in the buffer.

	if (Len == 0 && bytDataToSendLength == 0)
	{
		WriteDebugLog(LOGDEBUG, "[ARDOPprotocol.StartFEC] No data to send!");
		return FALSE;
	}

	// Check intRepeates
	
	if (intRepeats < 0 || intRepeats > 5)
	{
        //    Logs.Exception("[ARDOPprotocol.StartFEC] Repeats out of range: " & intRepeats.ToString)
		return FALSE;
	}
	
	//	check call sign
	
	if (!CheckValidCallsignSyntax(Callsign))
	{
     //       Logs.Exception("[ARDOPprotocol.StartFEC] Invalid call sign: " & MCB.Callsign)
		return FALSE;
	}
	
	//	Check to see that strDataMode is correct
    
	for (i = 0;  i < strAllDataModesLen; i++)
	{
		if (strcmp(strDataMode, strAllDataModes[i]) == 0)
		{
			strcpy(strFECMode, strDataMode);
			blnModeOK = TRUE;
			break;
		}
	}

	if (blnModeOK == FALSE)
	{
		//Logs.Exception("[ARDOPprotocol.StartFEC] Illegal FEC mode: " & strDataMode)
		return FALSE;
	}
        

	blnAbort = FALSE;

	intFrameRepeatInterval = 400;	// should be a safe number for FEC...perhaps could be shortened down to 200 -300 ms
 
	AddDataToDataToSend(bytData, Len);	// add new data to queue

	SetARDOPProtocolState(FECSend);		// set to FECSend state

	blnSendIDFrame = blnSendID;

	if (blnSendID)
	{
		NeedID = TRUE;
	}
	else
	{
		// Cant we just call GetNextFECFrame??

//		GetNextFECFrame();		// Use timer to start so cmd response is immediate
/*
			Dim bytFrameData(-1) As Byte
            strFrameComponents = strFECMode.Split(".")
            bytFrameType = objFrameInfo.FrameCode(strFECMode & ".E")
            If bytFrameType = bytLastFECDataFrameSent Then ' Added 0.3.4.1 
                bytFrameType = bytFrameType Xor &H1 ' insures a new start is on a different frame type. 
            End If
            objFrameInfo.FrameInfo(bytFrameType, blnOdd, intNumCar, strMod, intBaud, intDataLen, intRSLen, bytQualThres, strType)
            GetDataFromQueue(bytFrameData, intDataLen * intNumCar)
            ' If bytFrameData.Length < (intDataLen * intNumCar) Then ReDim Preserve bytFrameData((intDataLen * intNumCar) - 1)
            'Logs.WriteDebug("[ARDOPprotocol.StartFEC]  Frame Data (string) = " & GetString(bytFrameData))
            If strMod = "4FSK" Then
                bytFrameData = objMain.objMod.EncodeFSKData(bytFrameType, bytFrameData, strCurrentFrameFilename)
                intCurrentFrameSamples = objMain.objMod.Mod4FSKData(bytFrameType, bytFrameData)
            ElseIf strMod = "16FSK" Then
                bytFrameData = objMain.objMod.EncodeFSKData(bytFrameType, bytFrameData, strCurrentFrameFilename)
                intCurrentFrameSamples = objMain.objMod.Mod16FSKData(bytFrameType, bytFrameData)
            ElseIf strMod = "8FSK" Then
                bytFrameData = objMain.objMod.EncodeFSKData(bytFrameType, bytFrameData, strCurrentFrameFilename)
                intCurrentFrameSamples = objMain.objMod.Mod8FSKData(bytFrameType, bytFrameData)
            Else
                bytFrameData = objMain.objMod.EncodePSK(bytFrameType, bytFrameData, strCurrentFrameFilename)
                intCurrentFrameSamples = objMain.objMod.ModPSK(bytFrameType, bytFrameData)
            End If
            bytLastFECDataFrameSent = bytFrameType
            objMain.SendFrame(intCurrentFrameSamples, strCurrentFrameFilename)
            intFECFramesSent = 1
*/
	}
	return TRUE;
}
     
// Function to get the next FEC data frame 

BOOL GetNextFECFrame()
{
	int Len;
	int intNumCar, intBaud, intDataLen, intRSLen;
	BOOL blnOdd;
//    char strType[18] = "";
    char strMod[16] = "";
	int totSymbols;

	if (blnAbort)
	{
		ClearDataToSend();

		WriteDebugLog(LOGDEBUG, "[GetNextFECFrame] FECAbort. Going to DISC state");
		KeyPTT(FALSE);  // insurance for PTT off
		SetARDOPProtocolState(DISC);
		blnAbort = FALSE;
		return FALSE;
	}
	
	if (intFECFramesSent == -1)
	{
		WriteDebugLog(LOGDEBUG, "[GetNextFECFrame] intFECFramesSent = -1.  Going to DISC state");
		
		SetARDOPProtocolState(DISC);
		KeyPTT(FALSE); // insurance for PTT off
		return FALSE;
	}
	
	if (bytDataToSendLength == 0 && FECRepeatsSent >= FECRepeats && ProtocolState == FECSend)
	{
		// All sent. Send Over

		WriteDebugLog(LOGDEBUG, "[GetNextFECFrame] All data and repeats sent.  Send Over and go to DISC state");
         
		txSleep(400);
		EncodeAndSend4FSKControl(OVER, 0x3F, LeaderLength);

		SetARDOPProtocolState(DISC);
		blnEnbARQRpt = FALSE;
		KeyPTT(FALSE); // insurance for PTT of

		return FALSE;
	}

	if (ProtocolState == DISC && intPINGRepeats > 0)
	{
		intRepeatCount++;
		if (intRepeatCount <= intPINGRepeats && blnPINGrepeating)
		{
			dttLastPINGSent = Now;
			return TRUE;				// continue PING
		}
		
		intPINGRepeats = 0;
		blnPINGrepeating = False;
        return FALSE;
	}

	if (ProtocolState != FECSend)
		return FALSE;

	if (intFECFramesSent == 0)
	{
		// Initialize the first FEC Data frame (even) from the queue

		char FullType[18];

		strcpy(FullType, strFECMode);
		strcat(FullType, ".E");

		bytFrameType = FrameCode(FullType);
 
		FrameInfo(bytFrameType, &blnOdd, &intNumCar, strMod, &intBaud, &intDataLen, &intRSLen,  &totSymbols);

		Len = intDataLen * intNumCar;

		if (Len > bytDataToSendLength)
			Len = bytDataToSendLength;

		bytLastFECDataFrameSent = bytFrameType;

sendit:
		if (FECRepeats)
			blnEnbARQRpt = TRUE;
		else
			blnEnbARQRpt = FALSE;

		intFrameRepeatInterval = 400;	// should be a safe number for FEC...perhaps could be shortened down to 200 -300 ms

		FECRepeatsSent = 0;

		intFECFramesSent += 1;

		bytFrameType = bytLastFECDataFrameSent;

		EncodeData(bytFrameType);
		ModCarrierSet(intCalcLeader);

		// No ACKS, So ACK all sent carriers and reset PSN to 0
		
		while (intNumCar)
			AckCarrier(--intNumCar);

		unackedByteCount = 0;

		NextPSN = 1;

		return TRUE;
	}
	
	// Not First

	if (FECRepeatsSent >= FECRepeats)
	{
		// Send New Data

		//	Need to add pause  

		txSleep(400);

		if ((Now - dttLastFECIDSent) > 600000)		// 10 Mins
		{
			// Send ID every 10 Mins

			unsigned char bytEncodedBytes[16];

			EncLen = EncodePSKIDFrame(Callsign, GridSquare, bytEncodedBytes, 0x3F);
			Mod1Car50Bd4PSK(&bytEncodedBytes[0], 16, 0);		// only returns when all sent

			dttLastFECIDSent = Now;
			return TRUE;
		}

		FrameInfo(bytLastFECDataFrameSent, &blnOdd, &intNumCar, strMod, &intBaud, &intDataLen, &intRSLen,  &totSymbols);

		Len = intDataLen * intNumCar;

		if (Len > bytDataToSendLength)
			Len = bytDataToSendLength;

		bytLastFECDataFrameSent = bytLastFECDataFrameSent ^ 1;
		goto sendit;
	}
		
	// just a repeat of the last frame so no changes to samples...just inc frames Sent

	FECRepeatsSent++;
	intFECFramesSent++;

	if (FECRepeatsSent >= FECRepeats)
		blnEnbARQRpt = FALSE;				// Stop repeats

	return TRUE;
}

extern int frameLen;

 //	Subroutine to process Received FEC data 

void ProcessRcvdFECDataFrame(int intFrameType, UCHAR * bytData, BOOL blnFrameDecodedOK)
{
	// Determine if this frame should be passed to Host.

	return;			// I think data has to be forwarded in PassGoodDataToHost

	if (blnFrameDecodedOK)
	{
		int CRC = GenCRC16(bytData, frameLen);
		
		if (intFrameType == intLastFrameIDToHost && CRC == crcLastFECDataPassedToHost)
		{
			if (CommandTrace) WriteDebugLog(LOGINFO, "[ARDOPprotocol.ProcessRcvdFECDataFrame] Same Frame ID: %s and matching data, not passed to Host", Name(intFrameType));
			return;
		}
		
		if (bytFailedDataLength > 0 && intLastFailedFrameID != intFrameType)
		{
			AddTagToDataAndSendToHost(bytFailedData, "ERR", bytFailedDataLength);
			if (CommandTrace) WriteDebugLog(LOGINFO, "[ARDOPprotocol.ProcessRcvdFECDataFrame] Pass failed frame ID %s to Host (%d bytes)", Name(intFrameType), bytFailedDataLength);
			bytFailedDataLength = 0;
			intLastFailedFrameID = -1;
		}


		AddTagToDataAndSendToHost(bytData, "FEC", frameLen);
		crcLastFECDataPassedToHost = CRC;
		intLastFrameIDToHost = intFrameType;

		if (intLastFailedFrameID == intFrameType)
		{
			bytFailedDataLength = 0;
			intLastFailedFrameID = -1;
		}

		if (CommandTrace) WriteDebugLog(LOGINFO, "[ARDOPprotocol.ProcessRcvdFECDataFrame] Pass good data frame  ID %s to Host (%d bytes)", Name(intFrameType), frameLen);
	}
	else
	{
		// Bad Decode
		
		if (bytFailedDataLength > 0 && intLastFailedFrameID != intFrameType)
		{
			AddTagToDataAndSendToHost(bytFailedData, "ERR", bytFailedDataLength);
			if (CommandTrace) WriteDebugLog(LOGINFO, "[ARDOPprotocol.ProcessRcvdFECDataFrame] Pass failed frame ID %s to Host (%d bytes)", Name(intFrameType), bytFailedDataLength);
			bytFailedDataLength = 0;
			intLastFrameIDToHost = intLastFailedFrameID;
			if (CommandTrace) WriteDebugLog(LOGINFO, "[ARDOPprotocol.ProcessRcvdFECDataFrame] Pass failed frame ID %s to Host (%d bytes)", Name(intFrameType), bytFailedDataLength);
		}
		memcpy(bytFailedData, bytData, frameLen);	// ' capture the current data and frame type 
		bytFailedDataLength = frameLen;
		intLastFailedFrameID = intFrameType;
	}
}
