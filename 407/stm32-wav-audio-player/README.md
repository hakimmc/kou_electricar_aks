# WAV audio player for STM32 F4 discover board

This firmware demonstrates playback of WAV audio files on an STM32F4 discovery board.

## Hardware required

- STM32 F4 discovery board
- SD card module with SPI interface
- earphones or speakers
- USB cable to program and provide power to board

## Audio settings

Currently the audio parameters are hard-coded.

- sample rate: 44.1 kHz
- bit-depth: 16 bits
- channels: 2 (stereo)

Please check your WAV files are formatted for the above.

## Preparation

Place your WAV audio files in a folder called 'audio' on your SD card.

Ensure the filenames - excluding the .wav extension - are no longer than 8 characters long.
E.g., track1.wav is okay, while californication.wav is not.

Connect your earphones to the audio jack on the discovery board.
The volume is set to a sensible level by default - but perhaps double-check before actually playing a track!

You will also need a serial application to actually control the software. See below.

## Usage

Currently the software is controlled via the serial console. To see a full list of commands type 'help' in your console application.

To view the list of tracks, type

> dump track info

To play track 1, type

> play track 1

To set the volume, type

> set volume <volume>

To stop playback, simply type

> stop
