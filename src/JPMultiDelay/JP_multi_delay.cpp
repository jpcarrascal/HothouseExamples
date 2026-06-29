// BasicMultiDelay for Hothouse DIY DSP Platform
// Copyright (C) 2024 Your Name <your@email>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// Ported directly from the 'petal/MultiDelay' example in DaisyExamples. Knobs,
// switches, and pins are directly accessed without enums, so look at hothouse.h
// to decipher the mappings.

// It's fair to call this code 'obfuscated'; it's been left (mostly) as-is.


#include "daisysp.h"
#include "hothouse.h"

//using namespace clevelandmusicco;
//using namespace daisysp;
//using namespace daisy;

#define MAX_DELAY static_cast<size_t>(48000 * 2.0f)

using clevelandmusicco::Hothouse;
using daisy::AudioHandle;
using daisy::Led;
using daisy::Parameter;
using daisy::SaiHandle;
using daisy::System;
using daisysp::DelayLine;
using daisysp::Oscillator;
using daisysp::Svf;
using daisysp::fonepole;
using daisysp::fclamp;

Hothouse hw;
DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delMems[2];

static daisysp::Svf svfFilter[2];
static Oscillator filterLfo;

struct delay {
  DelayLine<float, MAX_DELAY> *del;
  float currentDelay;
  float delayTarget;
  float feedback;
  float delSend;

 
  float Process(float in) {
    // set delay times
    fonepole(currentDelay, delayTarget, 0.0002f);
    del->SetDelay(currentDelay);

    float read = del->Read();
    del->Write((feedback * read) + in * delSend);

    return read;
  }
};

enum outputMode {
  MISO = 0,
  STEREO = 1,
  MONO = 2
};

delay delays[2];
Parameter d_delay, d_feedback, d_level, p_freq, p_res, mod_freq;

// Bypass vars
Led led_bypass;
bool bypass = true;
outputMode currentOutputMode = MISO;

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
                   size_t size) {
  hw.ProcessAllControls();


  float baseCutoff = p_freq.Process();
  float res = p_res.Process();
  float lfoRate = mod_freq.Process();
  filterLfo.SetFreq(lfoRate);

  // knobs
  for (int i = 0; i < 2; i++) {
    delays[i].delayTarget = d_delay.Process();
    delays[i].feedback = d_feedback.Process();
    delays[i].delSend = d_level.Process();
  }

  int filterType = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1);
  static const float lfoDepthValues[] = {.0f, .9f, 2.0f};
  float lfoDepth = lfoDepthValues[hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2)];

  float lfo = filterLfo.Process();
  float modCutoff = baseCutoff * powf(2.0f, lfoDepth * lfo);
  modCutoff = fclamp(modCutoff, 20.0f, 20000.0f);
  

  // footswitch
  bypass ^= hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge();

  for (size_t i = 0; i < size; ++i) {


    // Remove this for full stereo input
    float input[2] = {in[0][i], in[1][i]};
    if(currentOutputMode == MISO) {
      input[1] = in[0][i];
    }
    float mix[2] = {0, 0};
    float sig[2] = {0, 0};

    for (int d = 0; d < 2; d++) {
      if(bypass) {
        delays[d].delSend = 0.0f;
      }
      sig[d] = delays[d].Process(input[d]);

      svfFilter[d].SetFreq(modCutoff);
      svfFilter[d].SetRes(res);
      svfFilter[d].Process(sig[d]);

      switch(filterType) {
        case 0: // LP
          sig[d] = svfFilter[d].Low();
          break;
        case 1: // BP
          sig[d] = svfFilter[d].Band();
          break;
        case 2: // HP
          sig[d] = svfFilter[d].High();
          break;
      }
      mix[d] += sig[d] + input[d];
    }


    out[0][i] = mix[0];
    out[1][i] = mix[1];
  }
}

int main() {
  hw.Init();

  if(hw.switches[Hothouse::FOOTSWITCH_1].Pressed()) {
    currentOutputMode = STEREO;
  }

  hw.SetAudioBlockSize(4);  // Number of samples handled per callback
  hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

  d_delay.Init(hw.knobs[0], hw.AudioSampleRate() * 0.05, MAX_DELAY,
                   Parameter::LOGARITHMIC);
  d_feedback.Init(hw.knobs[1], 0.0f, 1.0f, Parameter::LINEAR);
  d_level.Init(hw.knobs[2], 0.0f, 1.0f, Parameter::LINEAR);
  p_freq.Init(hw.knobs[Hothouse::KNOB_4], 200.0f, 20000.0f,
              Parameter::LOGARITHMIC);
  p_res.Init(hw.knobs[Hothouse::KNOB_5], 0.0f, 0.9f, Parameter::LINEAR);
  mod_freq.Init(hw.knobs[Hothouse::KNOB_6], 0.0f, 100.0f, Parameter::EXPONENTIAL);

  for (int i = 0; i < 2; i++) {
    // Init delays:
    delMems[i].Init();
    delays[i].del = &delMems[i];
    // Init filters: 
    svfFilter[i].Init(hw.AudioSampleRate());
    svfFilter[i].SetFreq(p_freq.Process());
    svfFilter[i].SetRes(p_res.Process());
  }

  filterLfo.Init(hw.AudioSampleRate());
  filterLfo.SetWaveform(Oscillator::WAVE_SIN);
  filterLfo.SetAmp(1.0f);
  filterLfo.SetFreq(mod_freq.Process());

  led_bypass.Init(hw.seed.GetPin(Hothouse::LED_2), false);

  hw.StartAdc();
  hw.StartAudio(AudioCallback);

  while (true) {
    led_bypass.Set(bypass ? 0.0f : 1.0f);
    led_bypass.Update();
    System::Delay(10);
    hw.CheckResetToBootloader();
  }
  return 0;
}