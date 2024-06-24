/*
 * WAV.h
 *
 *  Created on: 3 apr 2024
 *      Author: fabry
 */

#ifndef WAV_H_
#define WAV_H_


#include "Arduino.h"
#include "FS.h"
#include "SD_MMC.h"


#define WAV_SERIAL_DEBUG 1
#define NUM_BYTES_TO_READ_FROM_FILE 2048



      struct WavHeader_Struct
      {
          //   RIFF Section
          char RIFFSectionID[4];      // Letters "RIFF"
          uint32_t Size;              // Size of entire file less 8
          char RiffFormat[4];         // Letters "WAVE"

          //   Format Section
          char FormatSectionID[4];    // letters "fmt"
          uint32_t FormatSize;        // Size of format section less 8
          uint16_t FormatID;          // 1=uncompressed PCM
          uint16_t NumChannels;       // 1=mono,2=stereo
          uint32_t SampleRate;        // 44100, 16000, 8000 etc.
          uint32_t ByteRate;          // =SampleRate * Channels * (BitsPerSample/8)
          uint16_t BlockAlign;        // =Channels * (BitsPerSample/8)
          uint16_t BitsPerSample;     // 8,16,24 or 32

          // Data Section
          char DataSectionID[4];      // The letters "data"
          uint32_t DataSize;          // Size of the data that follows
      };

      // The data for one particular wav file
      struct Wav_Struct
      {
        File WavFile;                               // Object for accessing the opened wavfile
        uint32_t DataSize;                          // Size of wav file data
        bool Playing=false;                         // Is file playing
        bool Repeat;                                // If true, when wav ends, it will auto start again
        byte Samples[NUM_BYTES_TO_READ_FROM_FILE];  // Buffer to store data red from file
        uint32_t TotalBytesRead=0;                  // Number of bytes read from file so far
        uint16_t LastNumBytesRead;                  // Num bytes actually read from the wav file which will either be
                                                    // NUM_BYTES_TO_READ_FROM_FILE or less than this if we are very
                                                    // near the end of the file. i.e. we can't read beyond the file.

      };


      Wav_Struct Wav1;                              // Hum
      Wav_Struct Wav2;                              // Swing H
      Wav_Struct Wav3;                              // Swing L
      Wav_Struct Wav4; // Power on/power off

      void PlaySwingH()
      {
    	  if (!Wav2.Playing)
    	  {
        	  Wav2.Playing=true;
        	  Wav2.WavFile.seek(44);                                 // Reset to start of wav data
        	  Wav2.TotalBytesRead=0;                         // Clear to no bytes read in so far
    	  }

      }
      void PlaySwingL()
      {

    	  Wav3.Playing=true;
    	  Wav3.WavFile.seek(44);                                 // Reset to start of wav data
    	  Wav3.TotalBytesRead=0;                         // Clear to no bytes read in so far
      }

      void PlayPowerOn()
      {
    	  Wav4.Playing=true;
    	  Wav4.WavFile.seek(44);                                 // Reset to start of wav data
    	  Wav4.TotalBytesRead=0;                         // Clear to no bytes read in so far
      }

      float VolumeMaster = 1.0;
      float VolumePowerOnOff = 1.0;
      float VolumeHum = 0.05;
      float VolumeSwingH = 0.05;
      float VolumeSwingL = 0.05;
//------------------------------------------------------------------------------------------------------------------------

	  #define MAX(a, b) ((a) > (b) ? (a) : (b))
	  #define MAX4(a, b, c, d) (MAX(MAX(a, b), MAX(c, d)))


      void PlayWavs();
      uint16_t MixWavs(byte* Samples);
      bool InitWavFiles();
      void ReadFile(Wav_Struct *Wav);
      void ReadFiles();
      bool LoadWavFileHeader(String FileName, Wav_Struct* Wav);
      bool FillI2SBuffer(byte* Samples,uint16_t BytesInBuffer);
      bool ValidWavData(WavHeader_Struct* Wav);
      void DumpWAVHeader(WavHeader_Struct* Wav);
      void PrintData(const char* Data,uint8_t NumBytes);

//------------------------------------------------------------------------------------------------------------------------

      void PlayWavs()
      {
        static bool ReadingFile=true;                       // True if reading files from SD. false if filling I2S buffer
        static byte Samples[NUM_BYTES_TO_READ_FROM_FILE];   // Memory allocated to store the data read in from the wav files
        static uint16_t BytesReadFromFile;                  // Max Num bytes actually read from the wav files which will either be
                                                            // NUM_BYTES_TO_READ_FROM_FILE or less than this if we are very
                                                            // near the end of all files.

        if(ReadingFile)                                     // Read next chunk of data in from files
        {
          ReadFiles();                                      // Read data into the wavs own buffers
          BytesReadFromFile=MixWavs(Samples);                       // Mix the samples together and store in the samples buffer
          ReadingFile=false;                                // Switch to sending the buffer to the I2S
        }
        else
         ReadingFile=FillI2SBuffer(Samples,BytesReadFromFile);   // We keep calling this routine until it returns true, at which point
                                                                  // this will swap us back to Reading the next block of data from the file.
                                                                  // Reading true means it has managed to push all the data to the I2S
                                                                  // Handler, false means there still more to do and you should call this
                                                                  // routine again and again until it returns true.

      }


      uint16_t MixWavs(byte* Samples)
      {
        // Mix all playing wavs together, returns the max bytes that are in the buffer, usually this would be the full buffer but
        // in rare cases wavs may be close to the end of the file and thus not fill the entire buffer

        uint16_t Wav1Idx,Wav2Idx,Wav3Idx,Wav4Idx;           // Index into the wavs sample data
        int16_t Sample;                                     // The mixed sample
        uint16_t i;                                         // index into main samples buffer
        uint16_t MaxBytesInBuffer;                          // Max bytes of data in buffer, most of time buffer will be full

        Wav1Idx=0;
        Wav2Idx=0;
        Wav3Idx=0;
        Wav4Idx=0;

        while((Wav1Idx<Wav1.LastNumBytesRead)|(Wav2Idx<Wav2.LastNumBytesRead)|(Wav3Idx<Wav3.LastNumBytesRead)|(Wav4Idx<Wav4.LastNumBytesRead))
        {
          Sample=0;
          if(Wav1.Playing)         Sample+=*((int16_t *)(Wav1.Samples+Wav1Idx)) * VolumeHum ;
          if(Wav2.Playing)         Sample+=*((int16_t *)(Wav2.Samples+Wav2Idx)) * VolumeSwingH ;
          if(Wav3.Playing)         Sample+=*((int16_t *)(Wav3.Samples+Wav3Idx)) * VolumeSwingL ;
          if(Wav4.Playing)         Sample+=*((int16_t *)(Wav4.Samples+Wav4Idx)) * VolumePowerOnOff;

          *((int16_t *)(Samples+i))=Sample;

          Wav1Idx+=2;
          Wav2Idx+=2;
          Wav3Idx+=2;
          Wav4Idx+=2;

          i+=2;
        }

       MaxBytesInBuffer=MAX4(Wav1.LastNumBytesRead,
              Wav2.LastNumBytesRead,
Wav3.LastNumBytesRead,
Wav4.LastNumBytesRead);



      //  for(i=0;i<MaxBytesInBuffer;i+=2)  // We step 2 bytes at a time as we're using 16bits per channel
      //    *((int16_t *)(Samples+i))=(*((int16_t *)(Samples+i)))* VolumeMaster;

        return MaxBytesInBuffer;
      }





      bool InitWavFiles()
      {
        // initialise wav files
        if(LoadWavFileHeader("/HUM_0_22050.wav",&Wav1))
        if(LoadWavFileHeader("/SSH1_22k.wav",&Wav2))
        if(LoadWavFileHeader("/SSL1_22k.wav",&Wav3))
        return LoadWavFileHeader("/POWERON_0_22050.wav",&Wav4);             // only bother trying to load this if first loads ok
        return false;
      }


      void ReadFiles()
      {
        // Read in all files samples into their buffers
        if(Wav1.Playing)    ReadFile(&Wav1);
        if(Wav2.Playing)    ReadFile(&Wav2);
        if(Wav3.Playing)    ReadFile(&Wav3);
        if(Wav4.Playing)    ReadFile(&Wav4);
      }

      void ReadFile(Wav_Struct *Wav)
      {
          uint16_t i;                                         // loop counter
          int16_t SignedSample;                               // Single Signed Sample

          if(Wav->TotalBytesRead+NUM_BYTES_TO_READ_FROM_FILE>Wav->DataSize)   // If next read will go past the end then adjust the
            Wav->LastNumBytesRead=Wav->DataSize-Wav->TotalBytesRead;                    // amount to read to whatever is remaining to read
          else
            Wav->LastNumBytesRead=NUM_BYTES_TO_READ_FROM_FILE;                          // Default to max to read

          Wav->WavFile.read(Wav->Samples,Wav->LastNumBytesRead);                  // Read in the bytes from the file
          Wav->TotalBytesRead+=Wav->LastNumBytesRead;                        // Update the total bytes red in so far

          if(Wav->TotalBytesRead>=Wav->DataSize)              // Have we read in all the data?
          {
            if(Wav->Repeat)
            {
              Wav->WavFile.seek(44);                                 // Reset to start of wav data
              Wav->TotalBytesRead=0;                         // Clear to no bytes read in so far
            }
            else
              Wav->Playing=false;                                    // Flag that wav has completed
          }
      }

      bool LoadWavFileHeader(String FileName, Wav_Struct* Wav)
      {
        // Load wav file, if all goes ok returns true else false
        WavHeader_Struct WavHeader;

        Wav->WavFile = SD_MMC.open(FileName);                  // Open the wav file
        if(Wav->WavFile==false)
        {
#if WAV_SERIAL_DEBUG
        Serial.print("Could not open :");
        Serial.println(FileName);
#endif
            return false;
        }
        else
        {
          Wav->WavFile.read((byte *) &WavHeader,44);        // Read in the WAV header, which is first 44 bytes of the file.
                                                            // We have to typecast to bytes for the "read" function
          if(ValidWavData(&WavHeader))
          {
            DumpWAVHeader(&WavHeader);                      // Dump the header data to serial, optional!
            Serial.println();
            Wav->DataSize=WavHeader.DataSize;                    // Copy the data size into our wav structure
            return true;
          }
          else
            return false;
        }
      }



      bool FillI2SBuffer(byte* Samples,uint16_t BytesInBuffer)
      {
          // Writes bytes to buffer, returns true if all bytes sent else false, keeps track itself of how many left
          // to write, so just keep calling this routine until returns true to know they've all been written, then
          // you can re-fill the buffer

          size_t BytesWritten;                        // Returned by the I2S write routine,
          static uint16_t BufferIdx=0;                // Current pos of buffer to output next
          uint8_t* DataPtr;                           // Point to next data to send to I2S
          uint16_t BytesToSend;                       // Number of bytes to send to I2S

          // To make the code easier to understand I'm using to variables to some calculations, normally I'd write this calcs
          // directly into the line of code where they belong, but this make it easier to understand what's happening

          DataPtr=Samples+BufferIdx;                               // Set address to next byte in buffer to send out
          BytesToSend=BytesInBuffer-BufferIdx;                     // This is amount to send (total less what we've already sent)
          i2s_write(I2S_NUM_0,DataPtr,BytesToSend,&BytesWritten,1);  // Send the bytes, wait 1 RTOS tick to complete
          BufferIdx+=BytesWritten;                                 // increasue by number of bytes actually written

          if(BufferIdx>=BytesInBuffer)
          {
            // sent out all bytes in buffer, reset and return true to indicate this
            BufferIdx=0;
            return true;
          }
          return false;       // Still more data to send to I2S so return false to indicate this
      }

      bool ValidWavData(WavHeader_Struct* Wav)
      {

        if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0)
        {

          #if WAV_SERIAL_DEBUG
            Serial.print("Invalid data - Not RIFF format");
          #endif
          return false;
        }
        if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
        {
  #if WAV_SERIAL_DEBUG
        Serial.print("Invalid data - Not Wave file");
  #endif
          return false;
        }
        if(memcmp(Wav->FormatSectionID,"fmt",3)!=0)
        {
#if WAV_SERIAL_DEBUG
        Serial.print("Invalid data - No format section found");
#endif
        return false;
        }
        if(memcmp(Wav->DataSectionID,"data",4)!=0)
        {
 #if WAV_SERIAL_DEBUG
        Serial.print("Invalid data - data section not found");
 #endif
          return false;
        }
        if(Wav->FormatID!=1)
        {
 #if WAV_SERIAL_DEBUG
        Serial.print("Invalid data - format Id must be 1");
#endif
          return false;
        }
        if(Wav->FormatSize!=16)
        {
 #if WAV_SERIAL_DEBUG
        Serial.print("Invalid data - format section size must be 16.");
 #endif
          return false;
        }
        if((Wav->NumChannels!=1)&(Wav->NumChannels!=2))
        {
 #if WAV_SERIAL_DEBUG
        Serial.print("Invalid data - only mono or stereo permitted.");
 #endif
          return false;
        }
        if(Wav->SampleRate>48000)
        {
 #if WAV_SERIAL_DEBUG
Serial.print("Invalid data - Sample rate cannot be greater than 48000");
       #endif
          return false;
        }
        if((Wav->BitsPerSample!=8)& (Wav->BitsPerSample!=16))
        {
#if WAV_SERIAL_DEBUG
 Serial.print("Invalid data - Only 8 or 16 bits per sample permitted.");
#endif
          return false;
        }
        return true;
      }


void DumpWAVHeader(WavHeader_Struct* Wav)
{
        if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0)
        {
          #if WAV_SERIAL_DEBUG
 Serial.print("Not a RIFF format file - ");
 #endif
          PrintData(Wav->RIFFSectionID,4);
          return;
        }
        if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
        {
#if WAV_SERIAL_DEBUG
Serial.print("Not a WAVE file - ");
#endif
        PrintData(Wav->RiffFormat,4);
          return;
        }
        if(memcmp(Wav->FormatSectionID,"fmt",3)!=0)
        {
#if WAV_SERIAL_DEBUG
Serial.print("fmt ID not present - ");
#endif
        PrintData(Wav->FormatSectionID,3);
          return;
        }
        if(memcmp(Wav->DataSectionID,"data",4)!=0)
        {
#if WAV_SERIAL_DEBUG
Serial.print("data ID not present - ");
#endif
        PrintData(Wav->DataSectionID,4);
        return;
        }
        // All looks good, dump the data
#if WAV_SERIAL_DEBUG
        Serial.print("Total size :");Serial.println(Wav->Size);
        Serial.print("Format section size :");Serial.println(Wav->FormatSize);
        Serial.print("Wave format :");Serial.println(Wav->FormatID);
        Serial.print("Channels :");Serial.println(Wav->NumChannels);
        Serial.print("Sample Rate :");Serial.println(Wav->SampleRate);
        Serial.print("Byte Rate :");Serial.println(Wav->ByteRate);
        Serial.print("Block Align :");Serial.println(Wav->BlockAlign);
        Serial.print("Bits Per Sample :");Serial.println(Wav->BitsPerSample);
        Serial.print("Data Size :");Serial.println(Wav->DataSize);
#endif
}

void PrintData(const char* Data,uint8_t NumBytes)
{
#if WAV_SERIAL_DEBUG
     for(uint8_t i=0;i<NumBytes;i++)
            Serial.print(Data[i]);
            Serial.println();
#endif
}



#endif /* WAV_H_ */

