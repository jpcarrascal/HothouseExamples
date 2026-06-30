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
#include <cmath>

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

enum outputMode {
  MISO = 0,
  STEREO = 1,
  MONO = 2
};

// Plays a delay line back-to-front in overlapping grains instead of reading
// it forward. Grain depth must stay <= MAX_DELAY/2: the read sweeps from 0 up
// to 2x grain length, so anything longer would wrap into already-overwritten
// buffer content.
struct GrainReverser {
  float grainLength = 1.0f;
  float phaseA = 0.0f;
  float phaseB = 0.0f;
  float gainA  = 0.0f;
  float gainB  = 0.0f;

  void Advance(float grainLengthTarget) {
    fonepole(grainLength, fclamp(grainLengthTarget, 1.0f, MAX_DELAY / 2 - 1), 0.0002f);
    float cycle = 2.0f * grainLength;

    phaseA += 2.0f;
    if(phaseA >= cycle) phaseA -= cycle;

    phaseB = phaseA + grainLength;
    if(phaseB >= cycle) phaseB -= cycle;

    gainA = sinf(PI_F * phaseA / cycle);
    gainB = sinf(PI_F * phaseB / cycle);
  }

  float Read(DelayLine<float, MAX_DELAY> *del) {
    return gainA * del->Read(phaseA) + gainB * del->Read(phaseB);
  }
};

struct PingPongDelay {
  DelayLine<float, MAX_DELAY> *del1;
  DelayLine<float, MAX_DELAY> *del2;
  float currentDelay;
  float delayTarget;
  float feedback;
  float delaySend;


  std::pair<float, float> Process(float in1, float in2, outputMode currentOutputMode,
                                   bool reverseCompound, GrainReverser *reverser) {
    // set delay times
    fonepole(currentDelay, delayTarget, 0.0002f);

    float read1, read2;
    if(reverseCompound) {
      reverser->Advance(delayTarget);
      read1 = reverser->Read(del1);
      read2 = reverser->Read(del2);
    } else {
      del1->SetDelay(fclamp(currentDelay, 0.0f, MAX_DELAY - 1));
      del2->SetDelay(fclamp(currentDelay, 0.0f, MAX_DELAY - 1));
      read1 = del1->Read();
      read2 = del2->Read();
    }

    if(outputMode::MISO == currentOutputMode) {
      del1->Write((feedback * read2) + in1 * delaySend);
      del2->Write((feedback * read1));
    } else {
      del1->Write((feedback * read2) + in1 * delaySend);
      del2->Write((feedback * read1) + in2 * delaySend);
    }

    return {read1, read2};
  }
};

PingPongDelay ppDelay;
DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS dryCapture[2];
GrainReverser compoundReverser, oneShotReverser;
Parameter d_delay, d_feedback, d_send, f_freq, f_res, mod_freq;

// Bypass vars
Led led_delay, led_tap;
bool delayActive = false;
bool freeze = false;
bool freezeHold = false;

bool fs2LongPressHandled = false;
bool fs2SuppressReleaseToggle = false;

const unsigned int FS2_HOLD_MS = 500;
const unsigned int TAP_MIN_MS = 100;
const unsigned int TAP_TIMEOUT_MS = 2000;

bool tapWaitingForNext = false;
bool tapTempoActive = false;
unsigned int lastTapMs = 0;
unsigned int tapIntervals[3] = {0, 0, 0};
int tapIntervalCount = 0;
float tappedDelaySamples = 0.0f;
float tapDelayKnobBaseline = 0.0f;

const float DELAY_KNOB_TAP_CANCEL_THRESHOLD = 0.005f;

outputMode currentOutputMode = MISO;

void JPCheckResetToBootloader() {
  if(hw.switches[Hothouse::FOOTSWITCH_1].TimeHeldMs() >= 2000 && hw.switches[Hothouse::FOOTSWITCH_2].TimeHeldMs() >= 2000) {
    hw.StopAudio();
    hw.StopAdc();
    Led led1, led2;
    led1.Init(hw.seed.GetPin(Hothouse::LED_1), false);
    led2.Init(hw.seed.GetPin(Hothouse::LED_2), false);
    for(int i = 0; i < 3; i++) 
    {
        led1.Set(1.0f);
        led2.Set(0.0f);
        led1.Update();
        led2.Update();
        System::Delay(100);
        
        led1.Set(0.0f);
        led2.Set(1.0f);
        led1.Update();
        led2.Update();
        System::Delay(100);
    }
    
    System::ResetToBootloader();
  }
}

void UpdateButtons()
{
  unsigned int now = System::GetNow();

  if (tapWaitingForNext && (now - lastTapMs > TAP_TIMEOUT_MS))
  {
    tapWaitingForNext = false;
    tapIntervalCount = 0;
  }

  if (hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge())
  {
    if (!tapWaitingForNext)
    {
      tapWaitingForNext = true;
      tapIntervalCount = 0;
      lastTapMs = now;
    }
    else
    {
      unsigned int interval = now - lastTapMs;

      if (interval >= TAP_MIN_MS && interval <= TAP_TIMEOUT_MS)
      {
        if (tapIntervalCount < 3)
        {
          tapIntervals[tapIntervalCount++] = interval;
        }
        else
        {
          tapIntervals[0] = tapIntervals[1];
          tapIntervals[1] = tapIntervals[2];
          tapIntervals[2] = interval;
        }

        unsigned int total = 0;
        for (int i = 0; i < tapIntervalCount; ++i)
        {
          total += tapIntervals[i];
        }

        float averageTapMs = static_cast<float>(total) / tapIntervalCount;
        tappedDelaySamples = fclamp((averageTapMs * hw.AudioSampleRate()) / 1000.0f,
                      1.0f,
                      static_cast<float>(MAX_DELAY - 1));
        tapTempoActive = true;
        tapDelayKnobBaseline = hw.GetKnobValue(Hothouse::KNOB_1);
        lastTapMs = now;
      }
    }
  }

  // - Short press (on release) toggles delayActive when freezeHold is off
  // - Long press always enters freezeHold
  // - Short press while freezeHold is on exits freezeHold, keeps delayActive unchanged

  if (hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge())
  {
      fs2LongPressHandled = false;

      // Short press while in freezeHold -> exit freezeHold
      if (freezeHold)
      {
          freezeHold = false;
          freeze = false;
          fs2SuppressReleaseToggle = true; // prevent toggle on this release
      }
  }

  if (hw.switches[Hothouse::FOOTSWITCH_2].Pressed()
      && hw.switches[Hothouse::FOOTSWITCH_2].TimeHeldMs() >= FS2_HOLD_MS
      && !fs2LongPressHandled)
  {
      freezeHold = true;
      freeze = true;
      fs2LongPressHandled = true;
  }

  if (hw.switches[Hothouse::FOOTSWITCH_2].FallingEdge())
  {
      if (fs2SuppressReleaseToggle)
      {
          fs2SuppressReleaseToggle = false;
      }
      else if (!fs2LongPressHandled && !freezeHold)
      {
          // Normal short press
          delayActive = !delayActive;
      }

      fs2LongPressHandled = false;
  }
}

void ledBlink(Led& led)
{
    static bool ledState = false;
    static int counter = 0;
    counter++;
    if (counter >= 25) {
        ledState = !ledState;
        led.Set(ledState ? 1.0f : 0.0f);
        counter = 0;
    }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out,
                   size_t size) {
  hw.ProcessAllControls();


  float baseCutoff = f_freq.Process();
  float res = f_res.Process();
  float lfoRate = mod_freq.Process();
  filterLfo.SetFreq(lfoRate);

  // knobs

  // Freeze state follows freezeHold
  freeze = freezeHold;

  // Read controls
  float feedbackFromKnob = d_feedback.Process();
  if (feedbackFromKnob > 0.95f)
  {
      feedbackFromKnob = 1.0f;
  }
  float delaySendFromKnob = d_send.Process();
  float delayKnobPosition = hw.GetKnobValue(Hothouse::KNOB_1);

  if (tapTempoActive
      && std::fabs(delayKnobPosition - tapDelayKnobBaseline) > DELAY_KNOB_TAP_CANCEL_THRESHOLD)
  {
      tapTempoActive = false;
      tapWaitingForNext = false;
      tapIntervalCount = 0;
  }

  float knobDelayTarget = d_delay.Process();
  ppDelay.delayTarget = tapTempoActive ? tappedDelaySamples : knobDelayTarget;

  // Option B: delayActive always controls input feed
  ppDelay.delaySend = delayActive ? delaySendFromKnob : 0.0f;

  // Freeze controls loop behavior
  ppDelay.feedback = freeze ? 1.0f : feedbackFromKnob;

  int filterType = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1);
  static const float lfoDepthValues[] = {.0f, .9f, 2.0f};
  float lfoDepth = lfoDepthValues[hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2)];

  // UP: normal delay. MIDDLE: every repeat keeps re-reversing (reads del1/del2
  // directly). DOWN: only the fresh input is reversed once; repeats echo it forward.
  Hothouse::ToggleswitchPosition reverseSwitch = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_3);
  bool reverseOneShot = reverseSwitch == Hothouse::TOGGLESWITCH_MIDDLE;
  bool reverseCompound = reverseSwitch == Hothouse::TOGGLESWITCH_DOWN;

  float lfo = filterLfo.Process();
  float modCutoff = baseCutoff * powf(2.0f, lfoDepth * lfo);
  modCutoff = fclamp(modCutoff, 20.0f, 12000.0f);
  
  UpdateButtons();

  for (size_t i = 0; i < size; ++i) {


    // Remove this for full stereo input
    float input[2] = {in[0][i], in[1][i]};
    if(currentOutputMode == MISO) {
      input[0] = in[0][i] + in[1][i];
      input[1] = input[0];
    }
    std::pair<float, float> mix = {0, 0};
    std::pair<float, float> sig = {0, 0};

    float delayInputL = input[0];
    float delayInputR = input[1];

    if(!delayActive)
    {
        delayInputL = 0.0f;
        delayInputR = 0.0f;
    }

    // Always keep the capture buffer warm so DOWN mode has real history the
    // instant the switch is flipped, instead of reversing silence.
    dryCapture[0].Write(delayInputL);
    dryCapture[1].Write(delayInputR);

    if(reverseOneShot)
    {
        oneShotReverser.Advance(ppDelay.delayTarget);
        delayInputL = oneShotReverser.Read(&dryCapture[0]);
        delayInputR = oneShotReverser.Read(&dryCapture[1]);
    }

    sig = ppDelay.Process(delayInputL, delayInputR, currentOutputMode,
                          reverseCompound, &compoundReverser);

    if(!std::isfinite(sig.first))  sig.first  = 0.0f;
    if(!std::isfinite(sig.second)) sig.second = 0.0f;

    for (int d = 0; d < 2; d++) {
      svfFilter[d].SetFreq(modCutoff);
      svfFilter[d].SetRes(res);
    }
    svfFilter[0].Process(sig.first);
    svfFilter[1].Process(sig.second);

    switch(filterType) {
      case 0: // LP
        sig = {svfFilter[0].Low(), svfFilter[1].Low()};
        break;
      case 1: // BP
        sig = {svfFilter[0].Band(), svfFilter[1].Band()};
        break;
      case 2: // HP
        sig = {svfFilter[0].High(), svfFilter[1].High()};
        break;
    }

    // Guard filtered wet before mixing
    if(!std::isfinite(sig.first))  sig.first  = 0.0f;
    if(!std::isfinite(sig.second)) sig.second = 0.0f;
    mix = {sig.first + input[0], sig.second + input[1]};


    out[0][i] = mix.first;
    out[1][i] = mix.second;
  }
}

int main() {
  hw.Init();
  led_delay.Init(hw.seed.GetPin(Hothouse::LED_2), false);
  led_tap.Init(hw.seed.GetPin(Hothouse::LED_1), false);

  bool stereo_at_boot = false;
  unsigned int start = System::GetNow();

  while(System::GetNow() - start < 500) // 500 ms detection window
  {
      hw.ProcessDigitalControls();
      if(hw.switches[Hothouse::FOOTSWITCH_1].Pressed())
      {
          stereo_at_boot = true;
          for(int i = 0; i < 3; i++)
          {
              led_tap.Set(1.0f);
              led_tap.Update();
              System::Delay(100);
              led_tap.Set(0.0f);
              led_tap.Update();
              System::Delay(100);
          }
      }
      System::Delay(2);
  }

  if(stereo_at_boot)
  {
      currentOutputMode = STEREO;
  }

  hw.SetAudioBlockSize(4);  // Number of samples handled per callback
  hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

  d_delay.Init(hw.knobs[0], hw.AudioSampleRate() * 0.05, MAX_DELAY,
                   Parameter::LOGARITHMIC);
  d_feedback.Init(hw.knobs[1], 0.0f, 1.0f, Parameter::LINEAR);
  d_send.Init(hw.knobs[2], 0.0f, 1.5f, Parameter::LINEAR);
  f_freq.Init(hw.knobs[Hothouse::KNOB_4], 200.0f, 20000.0f,
              Parameter::LOGARITHMIC);
  f_res.Init(hw.knobs[Hothouse::KNOB_5], 0.0f, 0.9f, Parameter::LINEAR);
  mod_freq.Init(hw.knobs[Hothouse::KNOB_6], 0.0f, 100.0f, Parameter::EXPONENTIAL);

  for (int i = 0; i < 2; i++) {
    // Init delays:
    delMems[i].Init();
    dryCapture[i].Init();
    // Init filters:
    svfFilter[i].Init(hw.AudioSampleRate());
    svfFilter[i].SetFreq(f_freq.Process());
    svfFilter[i].SetRes(f_res.Process());
  }
  ppDelay.del1 = &delMems[0];
  ppDelay.del2 = &delMems[1];
  ppDelay.currentDelay = d_delay.Process();
  tapDelayKnobBaseline = hw.GetKnobValue(Hothouse::KNOB_1);

  filterLfo.Init(hw.AudioSampleRate());
  filterLfo.SetWaveform(Oscillator::WAVE_SIN);
  filterLfo.SetAmp(1.0f);
  filterLfo.SetFreq(mod_freq.Process());

  hw.StartAdc();
  hw.StartAudio(AudioCallback);

  while (true) {
    if (freezeHold)
    {
        ledBlink(led_delay);
    }
    else
    {
        led_delay.Set(delayActive ? 1.0f : 0.0f);
    }
    led_delay.Update();

    if(tapTempoActive)
    {
        led_tap.Set(1.0f);
    }
    else
    {
        led_tap.Set(0.0f);
    }
    led_tap.Update();

    System::Delay(10);
    JPCheckResetToBootloader(); // Requires both FSs, Less likely to be hit accidentally
  }
  return 0;
}