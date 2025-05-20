/*
 *  MainAudioAI.ino - Voice command sample using spectrograms
 *  Copyright 2022 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <Audio.h>
#include <FFT.h>

#define FFT_LEN 512

// Initialize FFT with stereo, 512 samples
FFTClass<AS_CHANNEL_MONO, FFT_LEN> FFT;

AudioClass* theAudio = AudioClass::getInstance();

#include <SDHCI.h>
SDClass SD;
File myFile;;

#include <float.h> // Header defining FLT_MAX, FLT_MIN
#include <DNNRT.h>
#define NNB_FILE "model.nnb"
DNNRT dnnrt;

#include <SoftwareSerial.h>

const byte rxPin = 2;
const byte txPin = 3;
static byte ID = 1;

// Set up a new SoftwareSerial object   
SoftwareSerial UNO (rxPin, txPin);


void setup() {
  Serial.begin(115200);
  UNO.begin(9600);
  while (!SD.begin()) {Serial.println("insert SD card");}  
  File nnbfile = SD.open(NNB_FILE);
  if (!nnbfile) {
    Serial.println(String(NNB_FILE) + " not found");
    return;
  }
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("DNNRT initialization error");
    return;
  }

  // Hamming window, stereo, 50% overlap  
  FFT.begin(WindowHamming, AS_CHANNEL_MONO, (FFT_LEN/2));
  
  Serial.println("Init Audio Recorder");
  theAudio->begin();
  // Set input to microphone
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC);
  // Recording settings: format is PCM (16-bit RAW data),
  // Specify the location of the DSP codec (BIN directory on the SD card),
  // Sampling rate 16000Hz, stereo input   
  int err = theAudio->initRecorder(AS_CODECTYPE_PCM, 
    "/mnt/sd0/BIN", AS_SAMPLINGRATE_16000, AS_CHANNEL_MONO);
    if (err != AUDIOLIB_ECODE_OK) {
    Serial.println("Recorder initialize error");
    while(1);
  }

  Serial.println("Start Recorder");                                 
  theAudio->startRecorder(); // Start recording
}


void loop() {
  static const uint32_t buffering_time = 
      FFT_LEN*1000/AS_SAMPLINGRATE_16000;
  static const uint32_t buffer_size = 
      FFT_LEN*sizeof(int16_t)*AS_CHANNEL_MONO;
  static char buff[buffer_size]; // Buffer to store audio data
  static float pDst[FFT_LEN];  // Buffer to store the difference between MIC-A and MIC-B
  uint32_t read_size; 
  int ret;

  // Store the requested data in buff with buffer_size
  // The amount of data that could be read is set in read_size
  ret = theAudio->readFrames(buff, buffer_size, &read_size);
  if (ret != AUDIOLIB_ECODE_OK && 
      ret != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA) {
    Serial.println("Error err = " + String(ret));
    theAudio->stopRecorder();
    while(1);
  }
  
  // If the read size is less than buffer_size
  if (read_size < buffer_size) {
    delay(buffering_time); // Wait until data is accumulated
    return;
  } 
  
  FFT.put((q15_t*)buff, FFT_LEN); // Execute FFT
  FFT.get(pDst, 0);  // Data for MIC-A (for voice) is at index 0
  
  averageSmooth(pDst); // Smooth the data

  // Buffer for sound pressure data histogram
  static const int frames =  32;
  static float hist[frames];
  // Buffer for spectrogram
  static const int fft_samples = 96; // 3000Hz
  static float spc_data[frames*fft_samples];
   
  // Shift the data for histogram and spectrogram
  for (int t = 1; t < frames; ++t) {
    float* sp0 = spc_data+(t-1)*fft_samples;
    float* sp1 = spc_data+(t  )*fft_samples;
    memcpy(sp0, sp1, fft_samples*sizeof(float));
    hist[t-1] = hist[t];
  } 

  // Total sound pressure level after background noise reduction
  float sound_power_nc =  0; 
  for (int f = 0; f < FFT_LEN; ++f) {
    sound_power_nc += pDst[f];
  }
  
  // Add the latest sound pressure level data to the histogram
  hist[frames-1] = sound_power_nc;  
  // Add the latest FFT result to the spectrogram
  float* sp_last = spc_data + (frames-1)*fft_samples;
  memcpy(sp_last, pDst, fft_samples*sizeof(float));

  // Set sound threshold and silence threshold
  const float sound_th = 1000;
  float pre_area = 0;
  float post_area = 0;
  float target_area = 0;
  // Sum the sound pressure levels for the first 250ms, middle 500ms, and last 250ms
  for (int t = 0; t < frames; ++t) {
    if (t < frames/4) pre_area += hist[t];
    else if (t >= frames/4 && t < frames*3/4) target_area += hist[t];
    else if (t >= frames*3/4) post_area += hist[t];
  }

  int index = -1; // Recognition result to be passed to the subcore
  float value = -1; // Confidence of the recognition result to be passed to the subcore
  // Check if the first and last 250ms are below the silence threshold
  // and the middle 500ms is above the sound threshold
  if (target_area >= sound_th) {
    // Reset the histogram to avoid multiple detections
    memset(hist, 0, frames*sizeof(float)); 

    // Label text
    static const char label[2][15] = {"explosion", "non-explosion"};
    // Buffer for DNNRT input data
    DNNVariable input(frames/2*fft_samples/2);
    
    // Calculate the maximum and minimum values for normalization
    float spmax = FLT_MIN;
    float spmin = FLT_MAX;
    for (int n = 0; n < frames*fft_samples; ++n) {
      if (spc_data[n] > spmax) spmax = spc_data[n];
      if (spc_data[n] < spmin) spmin = spc_data[n];
    }
 
    // Convert from horizontal axis (frequency) x vertical axis (time) to horizontal axis (time) x vertical axis (frequency)
    float* data = input.data();
    int bf = fft_samples/2-1;
    for (int f = 0; f < fft_samples; f += 2) {
      int bt = 0;
      // Extract only the voice part
      for (int t = frames/4; t < frames*3/4; ++t) {
        // Normalize with the minimum and maximum values of the spectrogram
        float val0 = (spc_data[fft_samples*t+f] - spmin)/(spmax - spmin);
        float val1 = (spc_data[fft_samples*t+f+1] - spmin)/(spmax - spmin);
        float val = (val0 + val1)/2;  // Average reduction
        val = val < 0. ? 0. : val;
        val = val > 1. ? 1. : val;
        data[frames/2*bf+bt] = val;
        ++bt;
      }
      --bf;
    }

    // Temporarily stop the recorder for recognition processing
    theAudio->stopRecorder();  
    // Recognition processing
    dnnrt.inputVariable(input, 0);
    dnnrt.forward();
    DNNVariable output = dnnrt.outputVariable(0);
    // Output the result
    index = output.maxIndex();
    value = output[index];

    if (value > 0.9) {
      // Harcoded coordinates
      UNO.write("{\"id\":145,\"latitude\":14.648696,\"longitude\":121.068517}");

      // Wait for 10 seconds
      delay(10000);
    }

    theAudio->startRecorder(); // Resume the recorder
  }
}


void averageSmooth(float* dst) {
  const int array_size = 4;
  static float pArray[array_size][FFT_LEN/2];
  static int g_counter = 0;
  if (g_counter == array_size) g_counter = 0;
  for (int i = 0; i < FFT_LEN/2; ++i) {
    pArray[g_counter][i] = dst[i];
    float sum = 0;
    for (int j = 0; j < array_size; ++j) {
      sum += pArray[j][i];
    }
    dst[i] = sum / array_size;
  }
  ++g_counter;
}
