//
// This code calculates and writes to files the templates
// used to generate modulation samples.

// Rick's code gererates them dynamically as program start, but
// that measns they have to be in RAM. By pregenerating and 
// compliling them they can be placed in program space
// This is necessary with the small RAM space of embedded CPUs

// This only needs to be run once to generate the source files

// Keep code in case we need to change, but don't compile

#if 0

#include "ARDOPC.h"

#pragma warning(disable : 4244)		// Code does lots of int float to int

static int intAmp = 30000;	   // Selected to have some margin in calculations with 16 bit values (< 32767) this must apply to all filters as well. 

void Generate50BaudTwoToneLeaderTemplate()
{
	int i;
	float x, y, z;
	int line = 0;

	FILE * fp1;

	char msg[80];
	int len;

	fp1 = fopen("d:\\leadercoeffs.txt", "wb");

	for (i = 0; i < 240; i++)
	{
		x = 
		int50BaudTwoToneLeaderTemplate[i] = intAmp * 0.5 * ((sin(((1500.0 - 25) / 1500) * (i / 8.0 * 2 * M_PI))) - (sin(((1500.0 + 25) / 1500) * (i / 8.0 * 2 * M_PI))));

		if ((i - line) == 9)
		{
			// print the last 10 values

			len = sprintf(msg, "\t%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
				int50BaudTwoToneLeaderTemplate[line],
				int50BaudTwoToneLeaderTemplate[line + 1],
				int50BaudTwoToneLeaderTemplate[line + 2],
				int50BaudTwoToneLeaderTemplate[line + 3],
				int50BaudTwoToneLeaderTemplate[line + 4],
				int50BaudTwoToneLeaderTemplate[line + 5],
				int50BaudTwoToneLeaderTemplate[line + 6],
				int50BaudTwoToneLeaderTemplate[line + 7],
				int50BaudTwoToneLeaderTemplate[line + 8],
				int50BaudTwoToneLeaderTemplate[line + 9]);

			line = i + 1;

			fwrite(msg, 1, len, fp1);
		}
	}		
	fclose(fp1);
}

//	Subroutine to create the PSK symbol templates for 8 tones and 8 phases at 200 baud
float round(float x);

VOID Generate16APSK_8_8Templates()
{

	// Generate templates of 120 samples (each template = 20 ms) for each of the 11 possible carriers used in 16QAM modulation in a 12,4 circle. 
	// Used to speed up computation of PSK frames and reduce use of Sin functions.
	// Amplitude values will have to be scaled based on the number of Active Carriers (2 or 8) initial values should be OK for 1 carrier
	// Tone values 

	// the carrier frequencies in Hz
	
	float dblCarFreq[] = {600, 800, 1000, 1200, 1400, 1500, 1600, 1800, 2000, 2200, 2400};

	// for 1 carrier mode (200 Hz) use index 5 (1500 Hz)
	// for 2 carrier (500 Hz) modes use indexes 4, 6(1400 and 1600 Hz)
	// for 10 carrier modes use index 0-4, 6-10  (600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400 Hz) 
 
	float dblCarPhaseInc[11];	// the phase inc per sample based on frequency
	float dblAngle;				// Angle in radians
	int i, j, k;


	char msg[256];
	int len;
	int line = 0;
	FILE * fp1;

	
	// Compute the phase inc per sample
	
	for (i = 0; i < 11; i++)
	{
		dblCarPhaseInc[i] = 2 * M_PI * dblCarFreq[i] / 12000.0f;
	}
	
	//Now compute the templates: ( 32 bit values total)  
	
	for (i = 0; i < 11; i++) // across 11 tones
	{
		for (j = 0; j < 4; j++) //  using only half the phase values (0, 45, 90, 135)  (use sign compliment for the opposit phase) 
		{
			dblAngle = j * M_PI / 4;
			
			for (k = 0; k < 120; k++) // for 120 samples (one 100 baud symbol also used to generate 50 baud using mod 120) 
			{
				int16APSK_8_8_50bdCarTemplate[i][j][k] = intAmp * sinf(dblAngle);  // with no envelope control
				
				dblAngle += dblCarPhaseInc[i];
				if (dblAngle >= 2 * M_PI)
					dblAngle -= 2 * M_PI;
			}
		}
	}

	
	fp1 = fopen("d:\\PSKcoeffs.txt", "wb");

	len = sprintf(msg, "\tCONST short int16APSK_8_8_50bdCarTemplate[11][4][120]; = \r\n");
	fwrite(msg, 1, len, fp1);

	len = sprintf(msg, "\t{{{\r\n");
	fwrite(msg, 1, len, fp1);

	for (i = 0; i <= 10; i++)		// across 11 tones
	{
		for (j = 0; j <= 3; j++)
		{
			line = 0;
		
			len = sprintf(msg, "\r\n// Carrier %d Phase %d\r\n", i, j);

			fwrite(msg, 1, len, fp1);
		

			for (k = 0; k <= 119; k++) // for 120 samples (one 100 baud symbol, 200 baud modes will just use half of the data)
			{
				if ((k - line) == 9)
				{
					// print 10 to line

					len = sprintf(msg, "\t%d, %d, %d, %d, %d, %d, %d, %d, %d, %d,\n",
					int16APSK_8_8_50bdCarTemplate[i][j][line],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 1],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 2],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 3],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 4],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 5],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 6],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 7],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 8],
					int16APSK_8_8_50bdCarTemplate[i][j][line + 9]);

					line = k + 1;

					if (k == 119)
					{
						len += sprintf(&msg[len-2], "},\r\n\t{");
						len -=2;
					}
					fwrite(msg, 1, len, fp1);
				}
			}
//			len = sprintf(msg, "\t}{\r\n");
//			fwrite(msg, 1, len, fp1);
		}
		len = sprintf(msg, "\t}}{{\r\n");
		fwrite(msg, 1, len, fp1);
	}

	len = sprintf(msg, "\t}}};\r\n");
	fwrite(msg, 1, len, fp1);

	fclose(fp1);

}  


#endif


