/*
 * led_rgb.h
 *
 *  Created on: 31 рту. 2014 у.
 *      Author: Kreyl
 */

#pragma once

#include "hal.h"
#include "color.h"
#include "ChunkTypes.h"
#include "uart.h"
#include "kl_lib.h"

#if 1 // =========================== Common auxilary ===========================
// TimeToWaitBeforeNextAdjustment = SmoothVar / (N+4) + 1, where N - current LED brightness.
static inline uint32_t ICalcDelay(uint32_t CurrentBrightness, uint32_t SmoothVar) { return (uint32_t)((SmoothVar / (CurrentBrightness+4)) + 1); }
#endif

#if 0 // ========================= Single LED blinker ==========================
#define LED_RGB_BLINKER

class LedBlinker_t : public BaseSequencer_t<BaseChunk_t> {
protected:
    PinOutputPushPull_t IChnl;
    void ISwitchOff() { Off(); }
    SequencerLoopTask_t ISetup() {
        IChnl.Set(IPCurrentChunk->Value);
        IPCurrentChunk++;   // Always increase
        return sltProceed;  // Always proceed
    }
public:
    LedBlinker_t(const PinOutputPushPull_t AChnl) : BaseSequencer_t(), IChnl(AChnl) {}
    void Init() {
        IChnl.Init();
        Off();
    }
    void Off() { IChnl.Set(0); }
    void On()  { IChnl.Set(1); }
};
#endif


#if 0 // ======================== Single Led Smooth ============================
#define LED_TOP_VALUE       255
#define LED_INVERTED_PWM    invInverted

class LedSmooth_t : public BaseSequencer_t<LedSmoothChunk_t> {
private:
    PinOutputPWM_t<LED_TOP_VALUE, LED_INVERTED_PWM> IChnl;
    uint8_t ICurrentBrightness;
    void ISwitchOff() { SetBrightness(0); }
    SequencerLoopTask_t ISetup() {
        if(ICurrentBrightness != IPCurrentChunk->Brightness) {
            if(IPCurrentChunk->Value == 0) {     // If smooth time is zero,
                SetBrightness(IPCurrentChunk->Brightness); // set color now,
                ICurrentBrightness = IPCurrentChunk->Brightness;
                IPCurrentChunk++;                // and goto next chunk
            }
            else {
                if     (ICurrentBrightness < IPCurrentChunk->Brightness) ICurrentBrightness++;
                else if(ICurrentBrightness > IPCurrentChunk->Brightness) ICurrentBrightness--;
                SetBrightness(ICurrentBrightness);
                // Check if completed now
                if(ICurrentBrightness == IPCurrentChunk->Brightness) IPCurrentChunk++;
                else { // Not completed
                    // Calculate time to next adjustment
                    uint32_t Delay = ICalcDelay(ICurrentBrightness, IPCurrentChunk->Value);
                    SetupDelay(Delay);
                    return sltBreak;
                } // Not completed
            } // if time > 256
        } // if color is different
        else IPCurrentChunk++; // Color is the same, goto next chunk
        return sltProceed;
    }
public:
    LedSmooth_t(const PinOutputPWM_t<LED_TOP_VALUE, LED_INVERTED_PWM> AChnl) :
        BaseSequencer_t(), IChnl(AChnl), ICurrentBrightness(0) {}
    void Init() {
        IChnl.Init();
        SetBrightness(0);
    }
    void SetBrightness(uint8_t ABrightness) { IChnl.Set(ABrightness); }
};
#endif


#if 0 // ==================== RGB blinker (no smooth switch) ===================
#define LED_RGB_BLINKER
class LedRgbBlinker_t : public BaseSequencer_t<LedRGBChunk_t> {
protected:
    PinOutputPushPull_t R, G, B;
    void ISwitchOff() { SetColor(clBlack); }
    SequencerLoopTask_t ISetup() {
        SetColor(IPCurrentChunk->Color);
        IPCurrentChunk++;   // Always increase
        return sltProceed;  // Always proceed
    }
public:
    LedRgbBlinker_t(const PinOutputPushPull_t ARed, const PinOutputPushPull_t AGreen, const PinOutputPushPull_t ABlue) :
        BaseSequencer_t(), R(ARed), G(AGreen), B(ABlue) {}
    void Init() {
        R.Init();
        G.Init();
        B.Init();
        SetColor(clBlack);
    }
    void SetColor(Color_t AColor) {
        R.Set(AColor.R);
        G.Set(AColor.G);
        B.Set(AColor.B);
    }
};
#endif

#if 1 // ============================== LedRGB =================================
#define LED_RGB
#define LED_RGB_TOP_VALUE   255 // Intencity 0...255
#define LED_RGB_INVERTED    invNotInverted

class LedRGB_t : public BaseSequencer_t<LedRGBChunk_t> {
private:
    PinOutputPWM_t<LED_RGB_TOP_VALUE, LED_RGB_INVERTED, omPushPull>  R, G, B;
    Color_t ICurrColor;
    void ISwitchOff() { SetColor(clBlack); }
    SequencerLoopTask_t ISetup() {
        if(ICurrColor != IPCurrentChunk->Color) {
            if(IPCurrentChunk->Value == 0) {     // If smooth time is zero,
                SetColor(IPCurrentChunk->Color); // set color now,
                ICurrColor = IPCurrentChunk->Color;
                IPCurrentChunk++;                // and goto next chunk
            }
            else {
                ICurrColor.Adjust(&IPCurrentChunk->Color);
                SetColor(ICurrColor);
                // Check if completed now
                if(ICurrColor == IPCurrentChunk->Color) IPCurrentChunk++;
                else { // Not completed
                    // Calculate time to next adjustment
                    uint32_t DelayR = (ICurrColor.R == IPCurrentChunk->Color.R)? 0 : ICalcDelay(ICurrColor.R, IPCurrentChunk->Value);
                    uint32_t DelayG = (ICurrColor.G == IPCurrentChunk->Color.G)? 0 : ICalcDelay(ICurrColor.G, IPCurrentChunk->Value);
                    uint32_t DelayB = (ICurrColor.B == IPCurrentChunk->Color.B)? 0 : ICalcDelay(ICurrColor.B, IPCurrentChunk->Value);
                    uint32_t Delay = DelayR;
                    if(DelayG > Delay) Delay = DelayG;
                    if(DelayB > Delay) Delay = DelayB;
                    SetupDelay(Delay);
                    return sltBreak;
                } // Not completed
            } // if time > 256
        } // if color is different
        else IPCurrentChunk++; // Color is the same, goto next chunk
        return sltProceed;
    }
public:
    LedRGB_t(
            const PinOutputPWM_t<LED_RGB_TOP_VALUE, LED_RGB_INVERTED, omPushPull> ARed,
            const PinOutputPWM_t<LED_RGB_TOP_VALUE, LED_RGB_INVERTED, omPushPull> AGreen,
            const PinOutputPWM_t<LED_RGB_TOP_VALUE, LED_RGB_INVERTED, omPushPull> ABlue) :
        BaseSequencer_t(), R(ARed), G(AGreen), B(ABlue) {}
    void Init() {
        R.Init();
        G.Init();
        B.Init();
        SetColor(clBlack);
    }
    void SetColor(Color_t AColor) {
        R.Set(AColor.R);
        G.Set(AColor.G);
        B.Set(AColor.B);
    }
};
#endif
