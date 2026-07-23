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


#include "daisysp.h"
#include "hothouse.h"
#include "util/PersistentStorage.h"
#include <cmath>

#define DELAY_BUFFER_SIZE static_cast<size_t>(48000 * 10.0f)
#define MAX_DELAY_SAMPLES (48000 * 10.0f)
#define PITCH_GRAIN_MULTIPLIER 0.9f  // Grain length as a fraction of sample rate; adjust to taste

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

struct RigSettings {
  bool monoInput;
  bool monoOutput;
  bool operator!=(const RigSettings &o) const {
    return monoInput != o.monoInput || monoOutput != o.monoOutput;
  }
};
daisy::PersistentStorage<RigSettings> storage(hw.seed.qspi);

DelayLine<float, DELAY_BUFFER_SIZE> DSY_SDRAM_BSS delMems[2];

static daisysp::Svf svfFilter[2];
static Oscillator filterLfo;


// Plays a delay line back-to-front in overlapping grains instead of reading
// it forward. Grain depth must stay <= DELAY_BUFFER_SIZE/2: the read sweeps from 0 up
// to 2x grain length, so anything longer would wrap into already-overwritten
// buffer content.
struct GrainReverser {
  float grainLength = 1.0f;
  float phaseA = 0.0f;
  float phaseB = 0.0f;
  float gainA  = 0.0f;
  float gainB  = 0.0f;

  void Advance(float grainLengthTarget) {
    fonepole(grainLength, fclamp(grainLengthTarget, 1.0f, DELAY_BUFFER_SIZE / 2 - 1), 0.0002f);
    float cycle = 2.0f * grainLength;

    phaseA += 2.0f;
    if(phaseA >= cycle) phaseA -= cycle;

    phaseB = phaseA + grainLength;
    if(phaseB >= cycle) phaseB -= cycle;

    gainA = sinf(PI_F * phaseA / cycle);
    gainB = sinf(PI_F * phaseB / cycle);
  }

  float Read(DelayLine<float, DELAY_BUFFER_SIZE> *del) {
    return gainA * del->Read(phaseA) + gainB * del->Read(phaseB);
  }
};

// Plays a delay line faster than real time, transposing pitch upward.
// Phase [0,1) advances at (pitchRatio-1)/grainLength per sample; delay sweeps
// from grainLength down to 0 so buffer depth needed is only grainLength.
struct GrainPitchShifter {
  float grainLength = 1.0f;
  float phase = 0.0f;
  float gainA  = 0.0f;
  float gainB  = 0.0f;
  float delayA = 0.0f;
  float delayB = 0.0f;

  void Advance(float grainLengthTarget, float pitchRatio) {
    fonepole(grainLength, fclamp(grainLengthTarget, 1.0f, MAX_DELAY_SAMPLES), 0.0002f);

    phase += (pitchRatio - 1.0f) / grainLength;
    if(phase >= 1.0f) phase -= 1.0f;

    float phaseB = phase + 0.5f;
    if(phaseB >= 1.0f) phaseB -= 1.0f;

    delayA = grainLength * (1.0f - phase);
    delayB = grainLength * (1.0f - phaseB);
    gainA  = sinf(PI_F * phase);
    gainB  = sinf(PI_F * phaseB);
  }

  float Read(DelayLine<float, DELAY_BUFFER_SIZE> *del) {
    return gainA * del->Read(fclamp(delayA, 0.0f, DELAY_BUFFER_SIZE - 1))
         + gainB * del->Read(fclamp(delayB, 0.0f, DELAY_BUFFER_SIZE - 1));
  }
};

struct PingPongDelay {
  DelayLine<float, DELAY_BUFFER_SIZE> *del1;
  DelayLine<float, DELAY_BUFFER_SIZE> *del2;
  float currentDelay;
  float delayTarget;
  float feedback;
  float delaySend;


  std::pair<float, float> Process(float in1, float in2, bool monoInput,
                                   bool reverseCompound, GrainReverser *reverser,
                                   daisysp::Svf *inLoopFilters = nullptr, int inLoopFilterType = 0) {
    // set delay times
    fonepole(currentDelay, delayTarget, 0.0002f);

    float read1, read2;
    if(reverseCompound) {
      reverser->Advance(delayTarget);
      read1 = reverser->Read(del1);
      read2 = reverser->Read(del2);
    } else {
      del1->SetDelay(fclamp(currentDelay, 0.0f, DELAY_BUFFER_SIZE - 1));
      del2->SetDelay(fclamp(currentDelay, 0.0f, DELAY_BUFFER_SIZE - 1));
      read1 = del1->Read();
      read2 = del2->Read();
    }

    float writeL = (feedback * read2) + in1 * delaySend;
    float writeR = monoInput
                     ? (feedback * read1)
                     : (feedback * read1) + in2 * delaySend;

    if(inLoopFilters) {
      inLoopFilters[0].Process(writeL);
      inLoopFilters[1].Process(writeR);
      switch(inLoopFilterType) {
        case 0: writeL = inLoopFilters[0].Low();  writeR = inLoopFilters[1].Low();  break;
        case 1: writeL = inLoopFilters[0].Band(); writeR = inLoopFilters[1].Band(); break;
        case 2: writeL = inLoopFilters[0].High(); writeR = inLoopFilters[1].High(); break;
      }
    }

    del1->Write(writeL);
    del2->Write(writeR);

    return {read1, read2};
  }
};

PingPongDelay ppDelay;
DelayLine<float, DELAY_BUFFER_SIZE> DSY_SDRAM_BSS dryCapture[2];
DelayLine<float, DELAY_BUFFER_SIZE> DSY_SDRAM_BSS reversedCapture[2];
GrainReverser compoundReverser, oneShotReverser;
GrainPitchShifter pitchShifter;

float storedPitchRatio = 1.0f;
Hothouse::ToggleswitchPosition lastToggle3Pos = Hothouse::TOGGLESWITCH_UP;

enum FilterPosition { FILTER_AFTER, FILTER_BEFORE, FILTER_IN_LOOP };
FilterPosition storedFilterPosition = FILTER_AFTER;
Hothouse::ToggleswitchPosition lastToggle2Pos = Hothouse::TOGGLESWITCH_UP;

enum LofiMode { LOFI_OFF, LOFI_MILD, LOFI_HEAVY };
LofiMode storedLofiMode = LOFI_OFF;
Hothouse::ToggleswitchPosition lastToggle1Pos = Hothouse::TOGGLESWITCH_UP;

volatile int pitchLayerBlinkCount = 0;
Parameter d_delay, d_feedback, d_send, f_freq, f_res, mod_freq;

// Bypass vars
Led led_delay, led_tap;
bool delayActive = false;
bool freeze = false;
bool freezeHold = false;
bool sendGateOpen = false;

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

bool monoInput = false;
bool monoOutput  = false;
bool pendingSave = false;

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

  if (!freezeHold && hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge())
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
                      MAX_DELAY_SAMPLES);
        tapTempoActive = true;
        tapDelayKnobBaseline = hw.GetKnobValue(Hothouse::KNOB_1);
        lastTapMs = now;
      }
    }
  }

  sendGateOpen = freezeHold && hw.switches[Hothouse::FOOTSWITCH_1].Pressed();

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
      && !fs2LongPressHandled
      && delayActive)
  {
      freezeHold    = true;
      freeze        = true;
      sendGateOpen  = false;
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

  // Hidden modifier: hold FS1 and move TOGGLESWITCH_3 to select transpose interval.
  Hothouse::ToggleswitchPosition currentToggle3 =
      hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_3);
  if(currentToggle3 != lastToggle3Pos
     && hw.switches[Hothouse::FOOTSWITCH_1].Pressed()
     && !freezeHold)
  {
    if     (currentToggle3 == Hothouse::TOGGLESWITCH_MIDDLE) storedPitchRatio = 1.5f;
    else if(currentToggle3 == Hothouse::TOGGLESWITCH_DOWN)   storedPitchRatio = 2.0f;
    else                                                      storedPitchRatio = 1.0f;
    tapWaitingForNext    = false;
    tapTempoActive       = false;
    tapIntervalCount     = 0;
    pitchLayerBlinkCount = 6;
  }
  lastToggle3Pos = currentToggle3;

  // Hidden modifier: hold FS1 and move TOGGLESWITCH_2 to select filter position.
  Hothouse::ToggleswitchPosition currentToggle2 =
      hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2);
  if(currentToggle2 != lastToggle2Pos
     && hw.switches[Hothouse::FOOTSWITCH_1].Pressed()
     && !freezeHold)
  {
    if     (currentToggle2 == Hothouse::TOGGLESWITCH_UP)     storedFilterPosition = FILTER_AFTER;
    else if(currentToggle2 == Hothouse::TOGGLESWITCH_MIDDLE) storedFilterPosition = FILTER_BEFORE;
    else                                                      storedFilterPosition = FILTER_IN_LOOP;
    tapWaitingForNext    = false;
    tapTempoActive       = false;
    tapIntervalCount     = 0;
    pitchLayerBlinkCount = 6;
  }
  lastToggle2Pos = currentToggle2;

  // Hidden modifier: hold FS1 and move TOGGLESWITCH_1 to select lo-fi mode.
  Hothouse::ToggleswitchPosition currentToggle1 =
      hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1);
  if(currentToggle1 != lastToggle1Pos
     && hw.switches[Hothouse::FOOTSWITCH_1].Pressed()
     && !freezeHold)
  {
    if     (currentToggle1 == Hothouse::TOGGLESWITCH_UP)     storedLofiMode = LOFI_OFF;
    else if(currentToggle1 == Hothouse::TOGGLESWITCH_MIDDLE) storedLofiMode = LOFI_MILD;
    else                                                      storedLofiMode = LOFI_HEAVY;
    tapWaitingForNext    = false;
    tapTempoActive       = false;
    tapIntervalCount     = 0;
    pitchLayerBlinkCount = 6;
  }
  lastToggle1Pos = currentToggle1;
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

  bool sendBlocked = freeze && !sendGateOpen;
  ppDelay.delaySend = (delayActive && !sendBlocked) ? delaySendFromKnob : 0.0f;

  // Freeze controls loop behavior
  ppDelay.feedback = freeze ? 1.0f : feedbackFromKnob;

  int filterType = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1);
  static const float lfoDepthValues[] = {.0f, .9f, 2.0f};
  float lfoDepth = lfoDepthValues[hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2)];

  // TOGGLESWITCH_3 always selects delay/reverse mode.
  // Hold FS1 and move TOGGLESWITCH_3 to update stored transpose (see UpdateButtons).
  Hothouse::ToggleswitchPosition ts3 = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_3);
  bool reverseOneShot  = (ts3 == Hothouse::TOGGLESWITCH_MIDDLE);
  bool reverseCompound = (ts3 == Hothouse::TOGGLESWITCH_DOWN);

  float lfo = filterLfo.Process();
  float modCutoff = baseCutoff * powf(2.0f, lfoDepth * lfo);
  modCutoff = fclamp(modCutoff, 20.0f, 12000.0f);
  
  UpdateButtons();

  // Lo-fi parameters — precomputed once per block.
  // MILD: N=2 (effective 24kHz), 8-bit. HEAVY: N=4 (effective 12kHz), 4-bit.
  const int   lofiN      = (storedLofiMode == LOFI_HEAVY) ? 4 : 2;
  const float lofiLevels = (storedLofiMode == LOFI_HEAVY) ? 15.0f : 255.0f;

  for (size_t i = 0; i < size; ++i) {


    float input[2] = {in[0][i], in[1][i]};
    if(monoInput) {
      input[0] = in[0][i] + in[1][i];
      input[1] = input[0];
    }
    std::pair<float, float> sig = {0, 0};

    float delayInputL = input[0];
    float delayInputR = input[1];

    if(!delayActive)
    {
        delayInputL = 0.0f;
        delayInputR = 0.0f;
    }

    // dryCapture holds raw dry input for oneShotReverser.
    // reversedCapture holds oneShotReverser output (or dry if not in reverse),
    // so pitchShifter can chain after the reverser in all modes.
    dryCapture[0].Write(delayInputL);
    dryCapture[1].Write(delayInputR);

    if(reverseOneShot)
    {
        oneShotReverser.Advance(ppDelay.delayTarget);
        delayInputL = oneShotReverser.Read(&dryCapture[0]);
        delayInputR = oneShotReverser.Read(&dryCapture[1]);
    }

    reversedCapture[0].Write(delayInputL);
    reversedCapture[1].Write(delayInputR);

    if(storedPitchRatio > 1.0f)
    {
        pitchShifter.Advance(hw.AudioSampleRate() * PITCH_GRAIN_MULTIPLIER, storedPitchRatio);
        delayInputL = pitchShifter.Read(&reversedCapture[0]);
        delayInputR = pitchShifter.Read(&reversedCapture[1]);
    }

    for (int d = 0; d < 2; d++) {
      svfFilter[d].SetFreq(modCutoff);
      svfFilter[d].SetRes(res);
    }

    if(storedFilterPosition == FILTER_BEFORE)
    {
      svfFilter[0].Process(delayInputL);
      svfFilter[1].Process(delayInputR);
      switch(filterType) {
        case 0: delayInputL = svfFilter[0].Low();  delayInputR = svfFilter[1].Low();  break;
        case 1: delayInputL = svfFilter[0].Band(); delayInputR = svfFilter[1].Band(); break;
        case 2: delayInputL = svfFilter[0].High(); delayInputR = svfFilter[1].High(); break;
      }
    }

    sig = ppDelay.Process(delayInputL, delayInputR, monoInput,
                          reverseCompound, &compoundReverser,
                          storedFilterPosition == FILTER_IN_LOOP ? svfFilter : nullptr,
                          filterType);

    if(!std::isfinite(sig.first))  sig.first  = 0.0f;
    if(!std::isfinite(sig.second)) sig.second = 0.0f;

    if(storedLofiMode != LOFI_OFF)
    {
      static int   lofiCounter = 0;
      static float lofiHeldL   = 0.0f;
      static float lofiHeldR   = 0.0f;
      if(++lofiCounter >= lofiN)
      {
        lofiCounter = 0;
        lofiHeldL   = fclamp(sig.first,  -1.0f, 1.0f);
        lofiHeldR   = fclamp(sig.second, -1.0f, 1.0f);
        lofiHeldL   = roundf(lofiHeldL * lofiLevels) / lofiLevels;
        lofiHeldR   = roundf(lofiHeldR * lofiLevels) / lofiLevels;
      }
      sig.first  = lofiHeldL;
      sig.second = lofiHeldR;
    }

    if(storedFilterPosition == FILTER_AFTER)
    {
      svfFilter[0].Process(sig.first);
      svfFilter[1].Process(sig.second);
      switch(filterType) {
        case 0: sig = {svfFilter[0].Low(),  svfFilter[1].Low()};  break;
        case 1: sig = {svfFilter[0].Band(), svfFilter[1].Band()}; break;
        case 2: sig = {svfFilter[0].High(), svfFilter[1].High()}; break;
      }
      if(!std::isfinite(sig.first))  sig.first  = 0.0f;
      if(!std::isfinite(sig.second)) sig.second = 0.0f;
    }

    if(monoOutput)
    {
      float monoWet = (sig.first + sig.second) * 0.5f;
      out[0][i] = monoWet + input[0];
      out[1][i] = monoWet + input[1];
    }
    else
    {
      out[0][i] = sig.first  + input[0];
      out[1][i] = sig.second + input[1];
    }
  }
}

int main() {
  hw.Init();
  led_delay.Init(hw.seed.GetPin(Hothouse::LED_2), false);
  led_tap.Init(hw.seed.GetPin(Hothouse::LED_1), false);

  // FS2 (left/input side) held at boot → stereo input
  // FS1 (right/output side) held at boot → mono output
  // Both held → stereo input + mono output
  bool monoInput_at_boot = false;
  bool monoOutput_at_boot  = false;
  unsigned int start = System::GetNow();

  while(System::GetNow() - start < 500)
  {
      hw.ProcessDigitalControls();
      if(hw.switches[Hothouse::FOOTSWITCH_2].Pressed() && !monoInput_at_boot)
      {
          monoInput_at_boot = true;
          for(int i = 0; i < 3; i++)
          {
              led_delay.Set(1.0f); led_delay.Update();
              System::Delay(100);
              led_delay.Set(0.0f); led_delay.Update();
              System::Delay(100);
          }
      }
      if(hw.switches[Hothouse::FOOTSWITCH_1].Pressed() && !monoOutput_at_boot)
      {
          monoOutput_at_boot = true;
          for(int i = 0; i < 3; i++)
          {
              led_tap.Set(1.0f); led_tap.Update();
              System::Delay(100);
              led_tap.Set(0.0f); led_tap.Update();
              System::Delay(100);
          }
      }
      System::Delay(2);
  }


  lastToggle3Pos = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_3);
  lastToggle2Pos = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2);
  lastToggle1Pos = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1);

  hw.SetAudioBlockSize(4);  // Number of samples handled per callback
  hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

  d_delay.Init(hw.knobs[0], hw.AudioSampleRate() * 0.05, MAX_DELAY_SAMPLES,
                   Parameter::LOGARITHMIC);
  d_feedback.Init(hw.knobs[1], 0.0f, 1.0f, Parameter::LINEAR);
  d_send.Init(hw.knobs[2], 0.0f, 1.5f, Parameter::LINEAR);
  f_freq.Init(hw.knobs[Hothouse::KNOB_4], 200.0f, 20000.0f,
              Parameter::LOGARITHMIC);
  f_res.Init(hw.knobs[Hothouse::KNOB_5], 0.0f, 0.9f, Parameter::LINEAR);
  mod_freq.Init(hw.knobs[Hothouse::KNOB_6], 0.0f, 100.0f, Parameter::CUBE);

  for (int i = 0; i < 2; i++) {
    // Init delays:
    delMems[i].Init();
    dryCapture[i].Init();
    reversedCapture[i].Init();
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

  // Load persisted rig settings, falling back to defaults on first boot.
  // Boot gestures toggle the stored value so repeated holds cycle the setting.
  RigSettings defaults{false, false};
  storage.Init(defaults);
  RigSettings &saved = storage.GetSettings();
  monoInput  = saved.monoInput;
  monoOutput = saved.monoOutput;

  if(monoInput_at_boot)  { monoInput  = !monoInput;  pendingSave = true; }
  if(monoOutput_at_boot) { monoOutput = !monoOutput; pendingSave = true; }

  // Prevent FS2 held during boot from triggering freeze or bypass toggle
  // when audio starts and UpdateButtons() first runs.
  if(monoInput_at_boot) {
    fs2LongPressHandled      = true;
    fs2SuppressReleaseToggle = true;
  }

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

    if(freezeHold)
    {
        led_tap.Set(sendGateOpen ? 1.0f : 0.0f);
    }
    else if(pitchLayerBlinkCount > 0)
    {
        static int blinkTick = 0;
        if(++blinkTick >= 10)
        {
            blinkTick = 0;
            --pitchLayerBlinkCount;
            led_tap.Set((pitchLayerBlinkCount % 2 == 0) ? 0.0f : 1.0f);
        }
    }
    else if(tapTempoActive)
    {
        led_tap.Set(1.0f);
    }
    else
    {
        led_tap.Set(0.0f);
    }
    led_tap.Update();

    if(pendingSave)
    {
      RigSettings &s = storage.GetSettings();
      s.monoInput = monoInput;
      s.monoOutput  = monoOutput;
      storage.Save();
      pendingSave = false;
    }

    System::Delay(10);
    JPCheckResetToBootloader(); // Requires both FSs, Less likely to be hit accidentally
  }
  return 0;
}