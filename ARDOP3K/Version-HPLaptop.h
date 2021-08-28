const char ProductName[] = "ARDOP TNC";
const char ProductVersion[] = "3.0.1.18fv-BPQ";

// i revert 2500 levels add check for two tones
// j maintain AcquireFrameSyncRSBAvg state over samplee buffers

// Suspect tuning isn't accurate enough for psk frame type

// k add CorrectPhaseForTuningOffset for all types, including frame type
// iss sends break insted of ack at start
// speed up IDLE repeat (3 > 2 secs)

// l Try replying to IDLE with data (auto IRS/ISS switch without using BREAK)
// Now no need to send BREAK at start

// m Fix FEC
// fix repeat error if first frame after auto ISS swich id repeated

// n Sync Diagnostics
// o ditto

// p. Require 3 good s/n in searchforleader
//	  More diagostics

// q change noise bins

// r Fix Windows input buffer count

// s Fix possible short leader on data frames

// t Revert p and q

// u Fix envelope correlator. 
//   More logging
//	 increase envelope correlator to 1.5 symbols starting at 360

// v Use Ardop2OFDM sync code