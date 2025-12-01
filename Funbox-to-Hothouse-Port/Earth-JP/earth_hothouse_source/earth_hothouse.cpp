// Earth Reverbscape - Hothouse Port
// Original by Keith Bloemer (GuitarML) for Funbox
// Ported to Hothouse by Chris Brandt, September 5, 2025
// 
// Multi-effect: Dattorro Reverb + Octave Generator + Overdrive
// Hardware: Hothouse DIY DSP Platform (Daisy Seed based)
// License: MIT (see LICENSE file)

#include "daisy_seed.h"
#include "daisysp.h"
#include "hothouse.h"
#include "expressionHandler.h"

#include "Dattorro/Dattorro.hpp"

#include <q/support/literals.hpp>
#include <q/fx/biquad.hpp>
#include "Util/Multirate.h"
#include "Util/OctaveGenerator.h"
namespace q = cycfi::q;
using namespace q::literals;

using namespace daisy;
using namespace daisysp;
using namespace clevelandmusicco;

// Declare hardware
Hothouse hw;
float pdamp, pmix, pdecay, pmoddepth, pmodspeed, ppredelay;
bool reverbActive;
bool driveActive;;
Led led1, led2;

float dryMix = 0.5f;
float wetMix = 0.5f;

// Expression
ExpressionHandler expHandler;
bool expression_pressed;

// Midi
bool midi_control[6];

// Control Values
float knobValues[6];
int toggleValues[3];
int prev_toggleValues[3] = {-1, -1, -1};
bool first_start;

float pknobValues[6];

Dattorro reverb(48000, 16, 4.0);
int reverbSampleRate = 48000;
int effect_mode = 0;
bool fw1_held = false;
bool freeze = false;
bool freezeHold = false;
bool reverbStateBeforeFreeze = false;
bool justExitedFreezeHold = false;

static Decimator2 decimate;
static Interpolator interpolate;
static const auto sample_rate_temp = 48000;
static OctaveGenerator octave(sample_rate_temp / resample_factor);
static q::highshelf eq1(-20, 100_Hz, sample_rate_temp);
static q::lowshelf eq2(5, 160_Hz, sample_rate_temp);
static q::lowshelf eq3(-11, 160_Hz, sample_rate_temp);
float buff[6];
float buff_out[6];
int bin_counter = 0;
int octaveEQ = 0;
int addOctave = 0;

float current_predelay, current_moddepth, current_modspeed, current_ODswell, current_freezeDecay;
float setTimeScale, current_timeScale, setOD;

Overdrive overdrivePre;
Overdrive overdrivePre2;
Overdrive overdrivePost;
Overdrive overdrivePost2;
bool odOn = false;

void ledBlink(Led& led)
{
    static bool ledState = false;
    static int counter = 0;
    counter++;
    if (counter >= 100) { // Blink every 200 calls (~200ms if called every 1ms)
        ledState = !ledState;
        led.Set(ledState ? 1.0f : 0.0f);
        counter = 0;
    }
}

bool knobMoved(float old_value, float new_value)
{
    float tolerance = 0.005;
    if (new_value > (old_value + tolerance) || new_value < (old_value - tolerance)) {
        return true;
    } else {
        return false;
    }
}

void updateSwitch1()
{
    if (toggleValues[0] == 0) {
        setTimeScale = 1.0;
    } else if (toggleValues[0] == 2) {
        setTimeScale = 4.0;
    } else {
        setTimeScale = 2.0;
    }
    reverb.setTimeScale(setTimeScale);
}

void updateSwitch2() 
{
    if (toggleValues[1] == 0) {

    } else if (toggleValues[1] == 1) {

    } else {

    }
}

void updateSwitch3() // What will we do with this one?
{
    if (toggleValues[2] == 0) {
        addOctave = 0;
    } else if (toggleValues[2] == 1) {
        addOctave = 1;
    } else {
        addOctave = 3;
    }
}

void UpdateButtons()
{
    // Footswitch 1:
    // * Switch reverb on/off with short press (after releasing switch)  
    // * Hold to freeze reverb and enter freezeHold mode
    // * freezeHold persists after button release with continuous freeze
    // * Short press while in freezeHold exits freezeHold (maintaining current reverb state)

    if(hw.switches[Hothouse::FOOTSWITCH_1].RisingEdge())
    {
        if(!freezeHold) {
            // Normal press - start freeze (don't save state yet)
            freeze = true;
        } else {
            // Short press while in freezeHold - exit freezeHold, stop freeze, restore saved reverb state
            freezeHold = false;
            freeze = false;
            reverbActive = reverbStateBeforeFreeze;
            led1.Set(reverbActive ? 1.0f : 0.0f);
            justExitedFreezeHold = true;  // Prevent FallingEdge from toggling
        }
    }

    if(hw.switches[Hothouse::FOOTSWITCH_1].TimeHeldMs() >= 1000 && !freezeHold) {
        // Save reverb state when entering freezeHold
        reverbStateBeforeFreeze = reverbActive;
        freezeHold = true;
    }

    if(hw.switches[Hothouse::FOOTSWITCH_1].FallingEdge())
    {
        if(justExitedFreezeHold) {
            // Just exited freezeHold, don't toggle
            justExitedFreezeHold = false;
        } else if(!freezeHold) {
            // Short press - toggle reverb on/off normally
            reverbActive = !reverbActive;
            led1.Set(reverbActive ? 1.0f : 0.0f);
            freeze = false;
        }
        // If freezeHold is true, do nothing on release - keep freeze active
    }

    // During freezeHold, maintain freeze and blink LED
    if(freezeHold) {
        freeze = true;  // Keep freeze active during freezeHold
        ledBlink(led1);
    }

    // Footswitch 2:
    if(hw.switches[Hothouse::FOOTSWITCH_2].RisingEdge())
    {
        driveActive = !driveActive;
        led2.Set(driveActive ? 1.0f : 0.0f);
    }
    
}

void UpdateSwitches()
{
    toggleValues[0] = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_1);
    toggleValues[1] = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_2);
    toggleValues[2] = hw.GetToggleswitchPosition(Hothouse::TOGGLESWITCH_3);
    
    if (toggleValues[0] != prev_toggleValues[0] || first_start) {
        prev_toggleValues[0] = toggleValues[0];
        updateSwitch1();
    }
    
    if (toggleValues[1] != prev_toggleValues[1] || first_start) {
        prev_toggleValues[1] = toggleValues[1];
        updateSwitch2();
    }
    
    if (toggleValues[2] != prev_toggleValues[2] || first_start) {
        prev_toggleValues[2] = toggleValues[2];
        updateSwitch3();
    }
    
    first_start = false;
}

void processSmoothedParameters()
{
    fonepole(current_predelay, ppredelay, .0002f);
    reverb.setPreDelay(current_predelay);

    fonepole(current_moddepth, pmoddepth, .0002f);
    reverb.setTankModDepth(current_moddepth * 8);

    fonepole(current_modspeed, pmodspeed, .0002f);
    reverb.setTankModSpeed(0.3 + current_modspeed * 15);

    if (odOn) {
        fonepole(current_ODswell, setOD, .000015f);
        overdrivePre.SetDrive(current_ODswell);
        overdrivePre2.SetDrive(current_ODswell);
        if (current_ODswell < 0.41 && !fw1_held) {
            odOn = false;
        }
    }

    if (freeze) {
        fonepole(current_freezeDecay, 1.0, .0002f); 
    } else {
        fonepole(current_freezeDecay, pdecay, .0002f); 
    }
    reverb.setDecay(current_freezeDecay);
}

static void AudioCallback(AudioHandle::InputBuffer in,
                          AudioHandle::OutputBuffer out,
                          size_t size)
{
    hw.ProcessAllControls();
    UpdateButtons();
    UpdateSwitches();
    led1.Update();
    led2.Update();

    // Read knobs
    float newExpressionValues[6];
    
    for(int i = 0; i < 6; i++) {
        float raw_knob = hw.GetKnobValue(static_cast<Hothouse::Knob>(i));
        if (!midi_control[i]) {
            pknobValues[i] = knobValues[i] = raw_knob;
        } else if (knobMoved(pknobValues[i], raw_knob)) {
            midi_control[i] = false;
        }
    }

    float vexpression = 0.5f;
    expHandler.Process(vexpression, knobValues, newExpressionValues);
  
    float vpredelay = newExpressionValues[0];
    float vmix = newExpressionValues[1];
    float vdecay = newExpressionValues[2];
    //float vmoddepth = newExpressionValues[3];
    //float vmodspeed = newExpressionValues[3];
    float overdrivePostAmount = newExpressionValues[3]; // HERE:
    float overdrivePreAmount = newExpressionValues[4]; // HERE:
    float vdamp = newExpressionValues[5];

    if (pmix != vmix) {
        if (knobMoved(pmix, vmix)) {
            float x2 = 1.0 - vmix;
            float A = vmix*x2;
            float B = A * (1.0 + 1.4186 * A);
            float C = B + vmix;
            float D = B + x2;

            wetMix = C * C;
            dryMix = D * D;
            pmix = vmix;
        }
    }

    if (knobMoved(ppredelay, vpredelay)) {
        ppredelay = vpredelay;
    }

    if (knobMoved(pdecay, vdecay)) {
        pdecay = vdecay;
    }

    /*if (knobMoved(pmoddepth, vmoddepth)) {
        pmoddepth = vmoddepth;
    }

    if (knobMoved(pmodspeed, vmodspeed)) {
        pmodspeed = vmodspeed;
    }*/

    if (knobMoved(pdamp, vdamp)) {
        reverb.setInputFilterHighCutoffPitch(10. - (7. * vdamp));
        pdamp = vdamp;
    }

    reverb.enableInputDiffusion(true);

    float inputL;
    float inputR;

    for (size_t i = 0; i < size; i++)
    {
        processSmoothedParameters();
        inputL = in[0][i];
        inputR = in[1][i];
        // Remove this for full stereo input
        inputL = inputR = in[0][i];
        
        float   reverb_in_L = inputL,
                reverb_in_R = inputR;


        // Overdrive is always on, but amount controlled by knob 4
        float drivePreValue = .3f + overdrivePreAmount * 0.5f;
        float drivePreLevelCompensantion = 1.0f - drivePreValue;
        overdrivePre.SetDrive(drivePreValue);
        overdrivePre2.SetDrive(drivePreValue);

        float drivePostValue = .3f + overdrivePostAmount * 0.5f;
        float drivePostLevelCompensantion = 1.0f - drivePostValue;
        overdrivePost.SetDrive(drivePostValue);
        overdrivePost2.SetDrive(drivePostValue);

        float octaveOutL = 0.0f;
        float octaveOutR = 0.0f;
        float effectLeftOut = 0.0f;
        float effectRightOut = 0.0f;
        float leftOutput = 0.0f;
        float rightOutput = 0.0f;

        if(driveActive == true) {
            reverb_in_L = overdrivePre.Process(  (inputL) ) * drivePreLevelCompensantion;
            reverb_in_R = overdrivePre2.Process( (inputR) ) * drivePreLevelCompensantion;
        } else {
            reverb_in_L = inputL;
            reverb_in_R = inputR;
        }

        /*--------- Octave START: ---------*/
        if(reverbActive == true) {
            if (addOctave > 0) {
                // Buffer input for octave processing
                buff[bin_counter] = inputL * 0.5f + inputR * 0.5f;
            
                // Process octave every 6 samples
                if (bin_counter == 5) {
                    std::span<const float, resample_factor> in_chunk(&(buff[0]), resample_factor);
                    const auto sample = decimate(in_chunk); 
                    octave.update(sample);
                    float octave_mix = octave.up1() * 2.0f * addOctave;
                    
                    auto out_chunk = interpolate(octave_mix);
                    for (size_t j = 0; j < out_chunk.size(); ++j) {
                        float mix = eq2(eq1(out_chunk[j]));
                        float dryLevel = 0.75f;
                        mix += dryLevel * buff[j];
                        buff_out[j] = mix;
                    }
                }

                if(bin_counter < 6) {
                    octaveOutL = buff_out[bin_counter];
                    octaveOutR = buff_out[bin_counter];
                }
            }
        }
        
        /*--------- Octave END. ---------*/

        if((reverbActive == true || driveActive == true)) {
            inputL = reverb_in_L;
            inputR = reverb_in_R;
        }
        if(reverbActive == false) {
            reverb_in_L = 0.0f;
            reverb_in_R = 0.0f;
        }
        reverb.process(reverb_in_L + octaveOutL, reverb_in_R + octaveOutR);
        effectLeftOut = reverb.getLeftOutput();  
        effectRightOut = reverb.getRightOutput();
        effectLeftOut =  overdrivePost.Process(  eq3(effectLeftOut) )  * drivePostLevelCompensantion;
        effectRightOut = overdrivePost2.Process( eq3(effectRightOut) ) * drivePostLevelCompensantion;

        if(reverbActive == true) {
            leftOutput =  inputL * dryMix + effectLeftOut * wetMix;
            rightOutput = inputR * dryMix + effectRightOut * wetMix;
        } else {
            leftOutput =  inputL + effectLeftOut * wetMix;
            rightOutput = inputR + effectRightOut * wetMix;
        }
        
        
        out[0][i] = leftOutput;
        out[1][i] = rightOutput;
        
        // Update bin counter
        bin_counter++;
        if (bin_counter >= 6) {
            bin_counter = 0;
        }
    }
}

void HandleMidiMessage(MidiEvent m)
{
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();
            if(m.data[1] != 0)
            {
                p = m.AsNoteOn();
            }
        }
        break;
        case ControlChange:
        {
            ControlChangeEvent p = m.AsControlChange();
            switch(p.control_number)
            {
                case 14:
                    midi_control[0] = true;
                    knobValues[0] = ((float)p.value / 127.0f);
                    break;
                case 15:
                    midi_control[1] = true;
                    knobValues[1] = ((float)p.value / 127.0f);
                    break;
                case 16:
                    midi_control[2] = true;
                    knobValues[2] = ((float)p.value / 127.0f);
                    break;
                case 17:
                    midi_control[3] = true;
                    knobValues[3] = ((float)p.value / 127.0f);
                    break;
                case 18:
                    midi_control[4] = true;
                    knobValues[4] = ((float)p.value / 127.0f);
                    break;
                case 19:
                    midi_control[5] = true;
                    knobValues[5] = ((float)p.value / 127.0f);
                    break;
                default: break;
            }
            break;
        }
        default: break;
    }
}

int main(void)
{
    float samplerate;

    hw.Init();
    hw.SetAudioBlockSize(48);
    samplerate = hw.AudioSampleRate();

    reverb.setSampleRate(samplerate);
    reverb.setTimeScale(2.0);
    reverb.setPreDelay(0.0);
    reverb.setInputFilterLowCutoffPitch(0.0);
    reverb.setInputFilterHighCutoffPitch(10.0);
    reverb.enableInputDiffusion(true);
    reverb.setDecay(0.5);
    reverb.setTankDiffusion(0.7);
    reverb.setTankFilterLowCutFrequency(0.0);
    reverb.setTankFilterHighCutFrequency(10.0);
    reverb.setTankModSpeed(1.0);
    reverb.setTankModDepth(0.0);
    reverb.setTankModShape(0.5);
    reverb.clear();

    for (int j = 0; j < 6; ++j) {
        buff[j] = 0.0;
        buff_out[j] = 0.0;
    }

    overdrivePre.Init();
    overdrivePre.SetDrive(0.4);
    overdrivePre2.Init();
    overdrivePre2.SetDrive(0.4);

    overdrivePost.Init();
    overdrivePost.SetDrive(0.4);
    overdrivePost2.Init();
    overdrivePost2.SetDrive(0.4);


    odOn = false;

    pdamp = 0.5f;
    pmix = 0.5f;
    pdecay = 0.5f;
    pmoddepth = 0.1f;
    pmodspeed = 0.5f;
    ppredelay = 0.5f;
    
    for(int i = 0; i < 6; i++) {
        knobValues[i] = 0.5f;
        pknobValues[i] = 0.5f;
    }

    current_predelay = 0.5f;
    current_moddepth = 0.1f;
    current_modspeed = 0.5f;
    current_freezeDecay = 0.5f;
    current_ODswell = 0.4f;
    setOD = 0.4f;

    expHandler.Init(6);
    expression_pressed = false;

    for(int i = 0; i < 6; ++i) 
        midi_control[i] = false;

    first_start = true;

    led1.Init(hw.seed.GetPin(Hothouse::LED_1), false);
    led1.Update();
    reverbActive = false;
    driveActive = false;

    led2.Init(hw.seed.GetPin(Hothouse::LED_2), false);
    led2.Update();

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    
    while(1)
    {
        // Check for bootloader reset in main loop
        // JP: Less easy to trigger accidentally if both footswitches are involved:
        if(hw.switches[Hothouse::FOOTSWITCH_1].TimeHeldMs() >= 2000 && hw.switches[Hothouse::FOOTSWITCH_2].TimeHeldMs() >= 2000)
        {
            hw.StopAudio();
            hw.StopAdc();
            
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
        
        System::Delay(100);
    }
}
