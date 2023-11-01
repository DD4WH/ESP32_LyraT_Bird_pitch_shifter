# ESP32-LyraT Bird Song Pitch Shifter

Inspired by the "Hear birds again project" I built this device for my father.

Many people suffer from high frequency hearing loss, especially people above the age of 50. Lang Elliott had the idea of lowering the pitch of bird songs with a special device in order to help bird enthusiasts with high frequency hearing loss to hear those higher-pitched bird songs [https://hearbirdsagain.org/]. These higher-pitched songs would normally be outside of their hearing ranges. His idea motivated me to try this on the ESP32 LyraT platform. This is my very first attempt to use the ESP32 LyraT platform.

The hardware uses:

* 2x AOM5024 low noise high SNR electret microphones
* audio codec to amplify and digitize the incoming audio
* ESP32 processor to do Digital Signal Processing
* the processor aquires the audio in real time and implements the pitch shifting algorithm and outputs the audio to the headphones
 
Thanks to Harold Mills I was able to implement **pitch shifting in the time domain** . The algorithm is based on an idea by Lang Elliott & Herb Susmann for the "Hear birds again"-project, specifically for the now deprecated SongFinder pitch shifter units. The algorithm can shift the audio down by a factor of two (one octave), three (1.5 octaves) or four (two octaves). Here is a graph showing my implementation for the case of downshifting by 4. Many thanks go to Harold Mills & Lang Elliott for explaining this algorithm to me and answering my questions ! :-)

![grafik](https://user-images.githubusercontent.com/14326464/194013110-f01d8397-0838-47c0-8373-3df8eebc1835.png)



![grafik](https://user-images.githubusercontent.com/14326464/192025404-fd7cb0a5-075f-4cdd-96d4-60ec2c061aa3.png)
My self-built binaural headset with Koss KSC75 headset and two low-noise AOM5024 electret mics fixed on the earbuds. It works very well and provides very low noise signals and enables the user to spatially locate the transposed audio like you would do with the original audio signal. See HERE (https://hearbirdsagain.org/binaural-headset/) for an explanation of the binaural headset and the building instructions. 

The LyraT board has a construction error in the Line input. I modified the line inputs, so that both inputs now have a Mic bias and are separated from each other:



Based on:

# ESP32-LyraT Audio Passthrough
### Short Description
This [esp-idf](https://github.com/espressif/esp-idf)-based application for the [ESP32-LyraT](https://www.espressif.com/en/products/hardware/esp32-lyrat) reads stereo samples from the board's analog "AUX IN" input jack and outputs them through the "PHONE JACK" analog output again. It does so without all the overhead that [esp-adf](https://github.com/espressif/esp-adf) brings at the cost of not being as portable as [esp-adf's pipeline_passthrough](https://github.com/espressif/esp-adf/tree/master/examples/audio_processing/pipeline_passthru).

By default, this application uses a word length of 16 bits (bits per channel) and a sample rate of 44100kHz. It first configures the ES8388 audio codec chip on the ESP32-LyraT board through I²C and then transfers digital audio samples bidirectionally over I²S.

This project can be used as a starting point for implementing **digital filters** such as FIR / IIR / biquad filters and other digital signal processing (DSP) applications on the ESP32 plattform. The ESP32's Tensilica Xtensa processor architecture is well-know for its DSP capabilities. For instance, FIR filters can be efficiently implemented on the ESP32 thanks to support for special 16-bit multiply-accumulate instructions with optional parallel loading (**MAC16** option for Tensilica Xtensa LX6).. See [esp-dsp](https://github.com/espressif/esp-dsp) and the Xtensa Instruction Set Architecture (ISA) Reference Manual for more information.

### Build and Flash
Make sure to have a recent [esp-idf](https://github.com/espressif/esp-idf) version with support for cmake-based projects installed. Then compile and flash this project as usual using the following steps:

* `idf.py all` to compile everything
* Put your ESP32-LyraT into download mode: Press and hold `Boot`, press and release `RST`, release `Boot`
* `idf.py -p /dev/ttyUSBX flash`to flash everything
* Reset your ESP32-LyraT by pressing `RST`

### License
This project is licensed under the MIT license. See `LICENSE` for details.
