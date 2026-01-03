/*
 * =====================================================================
 *                    RP2040 DUB SIREN
 *              with Real-time Waveform Display
 *              RP2040-Zero + ST7789 170x320 TFT
 * =====================================================================
 * 
 * HARDWARE:
 * Waveshare RP2040-Zero
 * ST7789 170x320 TFT Display
 * 4 Potentiometers (10k recommended)
 * 2 Buttons (momentary)
 * Audio out via PWM
 * 
 * WIRING:
 * TFT Display:
 *   SCL -> GP6, SDA -> GP7
 *   RST -> GP8, DC -> GP10, CS -> GP13
 * 
 * Potentiometers (to 3.3V and GND):
 *   Pot 1 (Pitch)         -> GP26 (ADC0)
 *   Pot 2 (Speed)         -> GP27 (ADC1)
 *   Pot 3 (Modulation)    -> GP28 (ADC2)
 *   Pot 4 (Echo Feedback) -> GP29 (ADC3)
 * 
 * Buttons:
 *   Trigger Button (momentary) -> GP3
 *   Delay Time Button          -> GP5 (cycles through 5 delay times)
 *   Mode Button                -> GP4 (cycles through synthesis modes)
 * 
 * Sync Input:
 *   External Sync (3-5V)       -> GP2 (triggers sound when HIGH)
 * 
 * Audio Output:
 *   PWM Audio -> GP15 -> 100Ω resistor -> amplifier input
 * =====================================================================
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>

// =====================================================================
// PIN DEFINITIONS
// =====================================================================

// Display pins
#define TFT_CS        13
#define TFT_RST       8
#define TFT_DC        10
#define TFT_MOSI      7
#define TFT_SCK       6

// Potentiometer pins (ADC)
#define POT_PITCH     26  // ADC0
#define POT_SPEED     27  // ADC1
#define POT_MOD       28  // ADC2 - Modulation depth
#define POT_FEEDBACK  29  // ADC3 - Echo feedback

// Button pins
#define BTN_TRIGGER   3
#define BTN_DELAY     5   // Cycles through delay times
#define BTN_MODE      4   // Cycles through synthesis modes
#define BTN_FUNCTION  1   // Multi-function button (gate length, etc)
#define BTN_MENU      0   // MENU BUTTON - cycles instruments
#define BTN_MUTATE    9   // MUTATE BUTTON - cycles mutation modes
#define BTN_FUNC_SHIFT 21 // FUNCTION SHIFT - hold for alternative key functions

// Sync input
#define SYNC_IN       2   // External sync input (3-5V triggers sound)

// Keyboard pins (8 keys)
#define KEY_1         11  // Root note
#define KEY_2         12  // 2nd degree
#define KEY_3         14  // 3rd degree
#define KEY_4         16  // 4th degree
#define KEY_5         17  // 5th degree
#define KEY_6         18  // 6th degree
#define KEY_7         19  // 7th degree
#define KEY_8         20  // Octave/8th degree

// Octave shift button
#define BTN_OCTAVE    23  // Cycles octave shift

// Audio pin
#define AUDIO_PIN     15

// Display dimensions
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 170

// Colors
#define COLOR_BG      0x0000
#define COLOR_WAVE    0x07E0  // Green
#define COLOR_GRID    0x2104  // Dark green
#define COLOR_TEXT    0xFFFF
#define COLOR_ACTIVE  0xF800  // Red
#define COLOR_ECHO    0x07FF  // Cyan
#define COLOR_MOD     0xFFE0  // Yellow

// =====================================================================
// AUDIO CONFIGURATION
// =====================================================================

#define SAMPLE_RATE 22050
#define PWM_WRAP 255

// Echo buffer - 500ms max delay at 22050 Hz
#define MAX_DELAY_SAMPLES 11025
int16_t echoBuffer[MAX_DELAY_SAMPLES];
uint16_t echoWritePos = 0;

// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================

/**
 * Draw a labeled parameter value with color-coded label
 * @param x X position
 * @param y Y position
 * @param width Width of clear area
 * @param label Label text (will be colored MOD color)
 * @param value Value to display
 * @param suffix Optional suffix (Hz, ms, %, etc)
 * @param valueColor Color for value text (default: TEXT)
 */

void drawParameter(int x, int y, int width, const char* label, const char* value, const char* suffix = "", uint16_t valueColor = COLOR_TEXT);
void drawParameter(int x, int y, int width, const char* label, int value, const char* suffix = "", uint16_t valueColor = COLOR_TEXT);
void drawSubHeading(int y, const char* label, const char* value, uint16_t valueColor = COLOR_TEXT);
void drawSubHeadingOctave(int y, int8_t octaveValue);
void drawSubHeadingGatePercent(int y, uint8_t percentage);
void drawWaveform(bool forceRedraw = false);
void drawParameters(bool forceFullRedraw = false);
void updateKeyboardFrequencies();
void generateMelody();
float quantizePitchToScale(float baseFreq, uint8_t scaleStep);

// =====================================================================
// GLOBAL OBJECTS
// =====================================================================

Adafruit_ST7789 display = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, TFT_RST);
uint slice_num;

// =====================================================================
// INSTRUMENT TYPES
// =====================================================================

enum InstrumentType {
    INST_DUB_SIREN = 0,
    INST_RAY_GUN,
    INST_LEAD_SYNTH,
    INST_DISCO,
    INST_COUNT
};

const char* instrumentNames[INST_COUNT] = {
    "DUB SIREN",
    "RAY GUN",
    "LEAD SYNTH",
    "DISCO"
};

// =====================================================================
// SYNTHESIS MODES
// =====================================================================

enum SynthMode {
    MODE_CLASSIC_DUB = 0,
    MODE_DEEP_SUB,
    MODE_SQUARE_WAVE,
    MODE_LOFI_CRUSH,
    MODE_RING_MOD,
    MODE_PORTAMENTO,
    MODE_COUNT
};

const char* modeNames[MODE_COUNT] = {
    "CLASSIC DUB",
    "DEEP SUB",
    "SQUARE WAVE",
    "LO-FI CRUSH",
    "RING MOD",
    "PORTAMENTO"
};

// Ray Gun sub-modes
enum RayGunMode {
    RAYGUN_ZAP = 0,      // Quick upward sweep
    RAYGUN_LASER,        // Downward sweep
    RAYGUN_BLASTER,      // Short burst
    RAYGUN_PHASER,       // Modulated pulse
    RAYGUN_MODE_COUNT
};

const char* rayGunModeNames[RAYGUN_MODE_COUNT] = {
    "ZAP",
    "LASER",
    "BLASTER",
    "PHASER"
};

// Lead Synth sub-modes
enum LeadSynthMode {
    LEAD_SEQUENCE = 0,    // Auto-sequenced melody
    LEAD_ARPEGGIO,        // Arpeggiator
    LEAD_EUCLIDEAN,       // Euclidean rhythm patterns
    LEAD_GENERATIVE,      // Generative melody
    LEAD_MODE_COUNT
};

const char* leadSynthModeNames[LEAD_MODE_COUNT] = {
    "SEQUENCE",
    "ARPEGGIO",
    "EUCLIDEAN",
    "GENERATIVE"
};

// Disco sub-modes
enum DiscoMode {
    DISCO_ORCHESTRA_HIT = 0,  // Punchy orchestral stab
    DISCO_STRING_SWEEP,       // Rising string machine
    DISCO_FUNK_BLAST,         // Synth brass stab
    DISCO_SPACE_WHOOSH,       // Noise sweep (spaceship)
    DISCO_BUBBLE_POP,         // Percussive bubble sound
    DISCO_LASER_SWEEP,        // Musical laser sweep
    DISCO_MODE_COUNT
};

const char* discoModeNames[DISCO_MODE_COUNT] = {
    "ORCH HIT",
    "STRINGS",
    "FUNK BLAST",
    "WHOOSH",
    "BUBBLE",
    "PAD CHORD"
};

// Mutation modes
enum MutationMode {
    MUTATE_OFF = 0,
    MUTATE_AMOUNT,
    MUTATE_RHYTHM,
    MUTATE_CUTOFF,
    MUTATE_MODE_COUNT
};

const char* mutationModeNames[MUTATE_MODE_COUNT] = {
    "OFF",
    "MUTATE",
    "RHYTHM",
    "CUTOFF"
};

// Loop/Gate pattern modes
enum LoopPattern {
    LOOP_OFF = 0,
    LOOP_TRIPLET,    // ███░░░███░░░ (triplet feel - 3 against 2)
    LOOP_SWING,      // ██░██░░██░ (swing/shuffle - classic dub feel)
    LOOP_STACCATO,   // █░░█░░█░░ (short stabs)
    LOOP_GATE,       // ████░░░░ (gate on first half, off second)
    LOOP_RATCHET,    // █░█░█░█░█░█░ (fast machine gun)
    LOOP_DOTTED,     // ██░█░██░█░ (dotted eighth feel)
    LOOP_COUNT
};

const char* loopPatternNames[LOOP_COUNT] = {
    "OFF",
    "TRIPLET",
    "SWING",
    "STACCATO",
    "GATE",
    "RATCHET",
    "DOTTED"
};

// Loop pattern state
struct LoopState {
    bool active;                  // Is loop mode active?
    uint8_t pattern;              // Current loop pattern
    unsigned long patternStartTime; // When current pattern cycle started
    uint16_t patternTempo;        // Pattern speed in ms (full cycle time)
    bool gateState;               // Current gate on/off state
};

LoopState loopState = {
    false,        // active
    LOOP_OFF,     // pattern
    0,            // patternStartTime
    480,          // patternTempo (480ms = 125 BPM, classic dub tempo)
    false         // gateState
};

// =====================================================================
// SEQUENCER STATE
// =====================================================================

struct SequencerState {
    bool active;              // Is sequencer mode active?
    bool running;             // Is sequencer playing?
    bool stepEnabled[8];      // Which steps are on/off
    float stepFrequencies[8]; // Frequency for each step
    uint8_t currentStep;      // Current playback position (0-7)
    unsigned long lastStepTime; // Timing for steps
    uint16_t stepInterval;    // ms between steps (from Pot 2)
    bool noteActive;          // Is current note playing
    uint32_t sequencerPhase;  // Sequencer oscillator phase
    uint8_t gateLength;       // Gate length percentage (20-100%)
    unsigned long noteStartTime; // When current note started
    uint16_t filterCutoff;    // Filter cutoff frequency (200-4000)
    int8_t octaveShift;       // Octave shift for sequencer (-2 to +2)
    uint8_t soundMode;        // Sound variation mode
};

SequencerState sequencer = {
    false,                    // active
    false,                    // running
    {true, true, true, true, true, true, true, true}, // All steps enabled initially
    {0,0,0,0,0,0,0,0},       // stepFrequencies (will be set from keyboard)
    0,                        // currentStep
    0,                        // lastStepTime
    250,                      // stepInterval (250ms default)
    false,                    // noteActive
    0,                        // sequencerPhase
    80,                       // gateLength (80% default)
    0,                        // noteStartTime
    1200,                     // filterCutoff (1200 default - warm tone)
    0,                        // octaveShift (start at 0)
    0                         // soundMode (SAW mode default)
};

// Sequencer sound modes
enum SequencerSoundMode {
    SEQ_SOUND_SAW = 0,     // Sawtooth/square mix (current)
    SEQ_SOUND_PULSE,       // Pulse width modulation
    SEQ_SOUND_FILTERED,    // Heavy filtered bass
    SEQ_SOUND_PLUCK,       // Plucked envelope
    SEQ_SOUND_CHORD,       // Add 5th harmonic
    SEQ_SOUND_COUNT
};

const char* seqSoundModeNames[SEQ_SOUND_COUNT] = {
    "SAW",
    "PULSE",
    "FILTERED",
    "PLUCK",
    "CHORD"
};

// Musical scales for pitch quantization (frequency ratios)
const float minorPentatonic[] = {1.0f, 1.189f, 1.335f, 1.498f, 1.782f, 2.0f}; // Root, m3, 4, 5, m7, octave
const uint8_t minorPentatonicSize = 6;

const float majorScale[] = {1.0f, 1.122f, 1.260f, 1.335f, 1.498f, 1.682f, 1.888f, 2.0f}; // Major scale
const uint8_t majorScaleSize = 8;

const float bluesScale[] = {1.0f, 1.189f, 1.260f, 1.335f, 1.498f, 1.782f, 2.0f}; // Blues scale
const uint8_t bluesScaleSize = 7;

const float minorScale[] = {1.0f, 1.122f, 1.189f, 1.335f, 1.498f, 1.587f, 1.782f, 2.0f}; // Natural minor
const uint8_t minorScaleSize = 8;

const float chromaticScale[] = {1.0f, 1.059f, 1.122f, 1.189f, 1.260f, 1.335f, 1.414f, 1.498f, 1.587f, 1.682f, 1.782f, 1.888f, 2.0f}; // All 12 notes
const uint8_t chromaticScaleSize = 13;

// Musical scales for mutation
enum MutationScale {
    SCALE_MINOR_PENT = 0,
    SCALE_MAJOR,
    SCALE_BLUES,
    SCALE_MINOR,
    SCALE_CHROMATIC,
    SCALE_COUNT
};

const char* scaleNames[SCALE_COUNT] = {
    "MIN PENT",
    "MAJOR",
    "BLUES",
    "MINOR",
    "CHROMATIC"
};

// Mutation state
struct MutationState {
    uint8_t mode;           // Current mutation mode
    uint8_t pitchStep;      // Current step in scale
    uint8_t speedStep;      // Current speed quantization step
    float mutationProbability; // Chance of mutation (0.0-1.0)
    unsigned long lastMutateTime; // Last time mutation occurred
    uint8_t currentScale;   // Current scale being used
    
    // Parameter locking
    uint16_t lockedProbabilityRaw;  // Locked pot value for probability
    uint16_t lockedScaleRaw;        // Locked pot value for scale
    uint16_t lockedCutoffRaw;       // Locked pot value for cutoff
    bool probabilityLocked;         // Is probability parameter locked?
    bool scaleLocked;               // Is scale parameter locked?
    bool cutoffLocked;              // Is cutoff parameter locked?
    bool fullyDisabled;             // Are all mutations disabled? (ADD THIS LINE)
};

MutationState mutate = {
    MUTATE_OFF,     // mode
    0,              // pitchStep (start at root)
    2,              // speedStep (start at medium speed)
    0.5f,           // mutationProbability (50% chance)
    0,              // lastMutateTime
    SCALE_MINOR_PENT, // currentScale (minor pentatonic default)
    
    // Initialize locks
    2048,           // lockedProbabilityRaw (middle position)
    2048,           // lockedScaleRaw (middle position)
    2048,           // lockedCutoffRaw (middle position)
    false,          // probabilityLocked
    false,          // scaleLocked
    false,          // cutoffLocked
    false           // fullyDisabled (ADD THIS LINE)
};

// Drone chord modes
enum DroneChordMode {
    DRONE_UNISON = 0,
    DRONE_OCTAVE,
    DRONE_FIFTH,
    DRONE_MAJOR,
    DRONE_MINOR,
    DRONE_CHORD_COUNT
};

const char* droneChordModeNames[DRONE_CHORD_COUNT] = {
    "UNISON",
    "OCTAVE",
    "FIFTH",
    "MAJOR",
    "MINOR"
};

// Drone synth state
struct DroneState {
    bool active;              // Is drone mode active?
    uint8_t playMode;         // Play mode: 0=OFF, 1=KEYS, 2=ON
    float rootFreq;           // Base frequency (20-400Hz)
    float detune;             // Detune amount (0.0-1.0)
    float brightness;         // Timbre control (0.0-1.0)
    float driftSpeed;         // LFO drift rate (0.1-5Hz)
    uint32_t oscPhases[5];    // Phase accumulators for each oscillator
    float oscDetunes[5];      // Detune offset for each osc (-1.0 to +1.0)
    uint32_t driftPhases[5];  // Slow LFOs for drift
    float envelope;           // Fade in/out envelope (0.0-1.0)
    unsigned long fadeStartTime; // When fade started
    uint8_t chordMode;        // Current chord mode
};

DroneState drone = {
    false,      // active
    0,          // playMode (0=OFF to start)
    110.0f,     // rootFreq (A2)
    0.3f,       // detune (30% default)
    0.6f,       // brightness (60% default)
    1.0f,       // driftSpeed (1Hz default)
    {0,0,0,0,0}, // oscPhases
    {-0.02f, -0.01f, 0.0f, 0.01f, 0.02f}, // oscDetunes (slight spread)
    {0,0,0,0,0}, // driftPhases
    0.0f,       // envelope (start silent)
    0,          // fadeStartTime
    0           // chordMode (UNISON default)
};

// Sidechain state
struct SidechainState {
    bool active;              // Is sidechain mode active?
    bool enabled;             // Is sidechain currently ON?
    float depth;              // Ducking amount (0.0-1.0)
    float attackTime;         // Attack time in ms (1-50ms)
    float releaseTime;        // Release time in ms (50-500ms)
    float envelope;           // Current envelope value (0.0-1.0)
    unsigned long lastTriggerTime; // When last trigger occurred
    bool triggered;           // Is currently triggered
};

SidechainState sidechain = {
    false,      // active
    false,      // enabled
    0.9f,       // depth (90% ducking default - MORE OBVIOUS)
    3.0f,       // attackTime (3ms - FASTER attack for punch)
    300.0f,     // releaseTime (300ms - longer release for classic pump)
    1.0f,       // envelope (start at full volume)
    0,          // lastTriggerTime
    false       // triggered
};

// Drone display tracking (global so loop1 can force updates)
uint8_t lastDronePlayMode = 255;
bool forceDisplayUpdate = false;  // Flag to force display redraw

// =====================================================================
// RECORDING STATE
// =====================================================================

#define MAX_RECORD_SAMPLES 110250  // 5 seconds at 22050 Hz

struct RecordState {
    bool active;                    // Is record mode active?
    bool recording;                 // Is currently recording?
    bool hasRecording;              // Is there a recording to play?
    int16_t* buffer;                // Recording buffer (allocated dynamically)
    uint32_t recordPosition;        // Current record/playback position
    uint32_t recordLength;          // Length of recording in samples
    bool playing;                   // Is playback active?
    unsigned long recordStartTime;  // When recording started
    bool loopEnabled;               // Is loop/repeat enabled?
};

RecordState recorder = {
    false,      // active
    false,      // recording
    false,      // hasRecording
    nullptr,    // buffer (will allocate in setup)
    0,          // recordPosition
    0,          // recordLength
    false,      // playing
    0,          // recordStartTime
    false       // loopEnabled
};

// Flag to suppress sound during record mode key presses
bool recordKeySilence = false;

// =====================================================================
// REVERSE STATE
// =====================================================================

#define MAX_REVERSE_SAMPLES 11025  // 500ms at 22050 Hz

struct ReverseState {
    bool active;                    // Is reverse mode active?
    bool reversing;                 // Is reverse currently ON?
    int16_t* buffer;                // Reverse buffer (allocated dynamically)
    uint32_t bufferSize;            // Size in samples
    uint32_t writePos;              // Current write position
    float mixAmount;                // Wet/dry mix (0.0=dry, 1.0=wet)
    int8_t pitchShift;              // Pitch shift (-2 to +2 octaves)
};

ReverseState reverser = {
    false,      // active
    false,      // reversing
    nullptr,    // buffer (will allocate in setup)
    MAX_REVERSE_SAMPLES,  // bufferSize
    0,          // writePos
    1.0f,       // mixAmount (100% wet default)
    -1          // pitchShift (-1 octave default)
};

// =====================================================================
// INFINITE HOLD STATE
// =====================================================================

bool infiniteSustain = false;
bool holdIsOn = false;  // Track if hold is currently ON
float savedFeedback = 0.0f;

// =====================================================================
// LFO 2 STATE
// =====================================================================

struct LFO2State {
    bool active;            // Is LFO 2 mode active?
    bool enabled;           // Is LFO 2 currently ON?
    float rate;             // LFO 2 rate (0.1-20 Hz)
    float depth;            // LFO 2 depth (0.0-1.0)
    uint32_t phase;         // LFO 2 phase accumulator
};

LFO2State lfo2 = {
    false,      // active
    false,      // enabled
    2.0f,       // rate (2 Hz default)
    0.5f,       // depth (50% default)
    0           // phase
};

// =====================================================================
// SYNTH PARAMETERS
// =====================================================================

struct SirenParams {
  // Instrument selection
    uint8_t instrumentType;     // Current instrument
    
    float frequency;      // Current frequency in Hz
    float targetFreq;     // Target frequency
    float baseFreq;       // Base frequency (set by pot)
    float lfoRate;        // LFO speed
    float lfoDepth;       // LFO modulation depth (controlled by pot!)
    uint16_t filterCutoff; // Lowpass filter cutoff (controlled by pot)
    uint16_t echoTime;    // Echo delay time in samples (fixed at 250ms)
    float echoFeedback;   // Echo feedback amount (0.0-0.95)
    float echoMix;        // Wet/dry mix (cycled by button)
    uint8_t echoLevel;    // Current echo level (0-4)
    uint8_t delayIndex;   // Current delay time index (0-4)
    uint8_t mode;         // Current synthesis mode
    float subMix;         // Sub-octave mix (0.0-1.0)
    uint8_t bitDepth;     // Bit crusher depth (1-8 bits)
    float ringModFreq;    // Ring mod frequency
    float glideTime;      // Portamento glide time
    bool triggered;
    
    // Gate length control
    uint8_t gateMode;           // Current gate mode
    unsigned long gateStartTime; // When gate was triggered
    bool gateActive;            // Is gate currently active
    
    // Ray Gun specific parameters
    uint8_t rayGunMode;         // Current ray gun sub-mode
    float sweepSpeed;           // Sweep rate
    float sweepDirection;       // 1.0 = up, -1.0 = down
    float resonance;            // Filter resonance/Q
    uint32_t rayGunPhase;       // Ray gun oscillator phase
    bool sweepActive;           // Is sweep currently active
    
    // Lead Synth parameters
    uint8_t leadSynthMode;      // Current lead synth sub-mode
    uint8_t stepCount;          // Number of steps in sequence (4-16)
    uint8_t currentStep;        // Current step position
    float noteFrequencies[16];  // Sequence of notes
    uint32_t lastStepTime;      // Timing for steps
    uint16_t stepInterval;      // Time between steps (ms)
    float vibratoDepth;         // Vibrato amount
    float vibratoRate;          // Vibrato speed
    uint8_t gateLength;         // Note gate length (0-100%)
    bool noteActive;            // Is current note playing
    uint32_t noteStartTime;     // When current note started
    uint32_t leadPhase;         // Lead synth oscillator phase
    
    // Disco parameters
    uint8_t discoMode;          // Current disco effect mode
    uint32_t discoPhase;        // Disco oscillator phase
    unsigned long discoStartTime; // When disco effect triggered
    float discoSweepStart;      // Starting frequency for sweeps
    float discoSweepEnd;        // Ending frequency for sweeps
    float discoEnvelope;        // Current envelope value (0.0-1.0)
    uint16_t discoNoiseState;   // Noise generator for whoosh/bubble
    float discoBrightness;      // Filter brightness (0.0-1.0)
    float discoCharacter;       // Mode-specific character control
};

SirenParams siren = {
    INST_DUB_SIREN, // instrumentType
    
    440.0f,         // frequency
    440.0f,         // targetFreq
    440.0f,         // baseFreq
    0.5f,           // lfoRate (slower default for classic dub)
    0.8f,           // lfoDepth (wider sweep)
    1200,           // filterCutoff (much lower = warmer, less harsh)
    5512,           // echoTime (250ms)
    0.65f,          // echoFeedback
    0.0f,           // echoMix
    0,              // echoLevel
    0,              // delayIndex
    MODE_CLASSIC_DUB, // mode
    0.5f,           // subMix
    8,              // bitDepth
    100.0f,         // ringModFreq
    0.05f,          // glideTime
    false,          // triggered
    
    0,              // gateMode
    0,              // gateStartTime
    false,          // gateActive
    
    // Ray Gun parameters (MUST match struct order!)
    RAYGUN_ZAP,     // rayGunMode
    2.0f,           // sweepSpeed
    1.0f,           // sweepDirection
    0.8f,           // resonance
    0,              // rayGunPhase
    false,          // sweepActive
    
    // Lead Synth parameters
    LEAD_SEQUENCE,  // leadSynthMode
    8,              // stepCount
    0,              // currentStep
    {440, 554, 659, 880, 659, 554, 440, 330, 0,0,0,0,0,0,0,0}, // noteFrequencies
    0,              // lastStepTime
    250,            // stepInterval
    0.05f,          // vibratoDepth
    5.0f,           // vibratoRate
    80,             // gateLength
    false,          // noteActive
    0,              // noteStartTime
    0,              // leadPhase
    
    // Disco parameters
    DISCO_ORCHESTRA_HIT, // discoMode
    0,              // discoPhase
    0,              // discoStartTime
    440.0f,         // discoSweepStart
    880.0f,         // discoSweepEnd
    0.0f,           // discoEnvelope
    54321,          // discoNoiseState
    0.8f,           // discoBrightness (bright by default)
    0.5f            // discoCharacter (50% default)
};

// =====================================================================
// OSCILLATOR STATE
// =====================================================================

uint32_t phase = 0;
uint32_t lfoPhase = 0;
int32_t filterState = 0;
int16_t lastSample = 0;

// Waveform buffer for display
#define WAVE_BUFFER_SIZE 320
int16_t waveBuffer[WAVE_BUFFER_SIZE];
uint16_t waveBufferPos = 0;

// =====================================================================
// BUTTON DEBOUNCING
// =====================================================================

struct Button {
    uint8_t pin;
    bool lastState;
    bool currentState;
    unsigned long lastDebounceTime;
};

Button triggerBtn = {BTN_TRIGGER, true, true, 0};
Button delayBtn = {BTN_DELAY, true, true, 0};
Button modeBtn = {BTN_MODE, true, true, 0};
Button functionBtn = {BTN_FUNCTION, true, true, 0};
Button menuBtn = {BTN_MENU, true, true, 0};
Button mutateBtn = {BTN_MUTATE, true, true, 0};
Button octaveBtn = {BTN_OCTAVE, true, true, 0};
Button funcShiftBtn = {BTN_FUNC_SHIFT, true, true, 0};

// Keyboard buttons
Button key1 = {KEY_1, true, true, 0};
Button key2 = {KEY_2, true, true, 0};
Button key3 = {KEY_3, true, true, 0};
Button key4 = {KEY_4, true, true, 0};
Button key5 = {KEY_5, true, true, 0};
Button key6 = {KEY_6, true, true, 0};
Button key7 = {KEY_7, true, true, 0};
Button key8 = {KEY_8, true, true, 0};

// Function key state tracking
bool lastFuncKeyStates[8] = {false, false, false, false, false, false, false, false};
unsigned long lastFuncKeyTime[8] = {0,0,0,0,0,0,0,0};

// Trigger button debounce state
bool lastTriggerReading = HIGH;
unsigned long lastTriggerDebounceTime = 0;
bool debouncedTriggerState = HIGH;

// Delay time settings (in samples at 22050 Hz)
// OFF, 50ms, 100ms, 175ms, 250ms, 375ms
const uint16_t delayTimes[6] = {0, 1103, 2205, 3859, 5512, 8269};
const char* delayTimeNames[6] = {"OFF", "50ms", "100ms", "175ms", "250ms", "375ms"};

struct KeyboardState {
    uint8_t currentKey;      // Which key is pressed (0 = none, 1-8 = keys)
    float keyFrequencies[8]; // Frequency for each key
    bool keyPressed;         // Is any key currently pressed
    bool keysHeld[8];        // Track which keys are currently held
    uint8_t numKeysHeld;     // How many keys are held
};

KeyboardState keyboard = {0, {0,0,0,0,0,0,0,0}, false, {false,false,false,false,false,false,false,false}, 0};

// Arpeggiator state
struct ArpState {
    bool enabled;               // Is arpeggiator on?
    uint8_t currentNote;        // Which held key are we playing
    unsigned long lastArpTime;  // Timing for arp steps
    uint16_t arpSpeed;          // ms between notes
    uint8_t heldKeys[8];        // Which keys are held
    uint8_t numHeldKeys;        // How many keys held
};

ArpState arp = {false, 0, 0, 150, {0,0,0,0,0,0,0,0}, 0};

// Octave shift state
int8_t octaveShift = 0;  // -1, 0, +1, +2 octaves

// Gate mode settings
enum GateMode {
    GATE_SHORT = 0,
    GATE_MEDIUM,
    GATE_LONG,
    GATE_EXTRA_LONG,
    GATE_MODE_COUNT
};

const uint16_t gateTimesMs[GATE_MODE_COUNT] = {
    50,     // SHORT
    200,    // MEDIUM
    500,    // LONG
    1000    // EXTRA LONG
};

const char* gateModeNames[GATE_MODE_COUNT] = {
    "SMALL",
    "MED",
    "LARGE",
    "X-LARGE"
};

bool readButton(Button* btn) {
    bool reading = digitalRead(btn->pin);
    
    if (reading != btn->lastState) {
        btn->lastDebounceTime = millis();
    }
    
    if ((millis() - btn->lastDebounceTime) > 50) {
        if (reading != btn->currentState) {
            btn->currentState = reading;
            btn->lastState = reading;
            return !reading; // Return true on press (active low)
        }
    }
    
    btn->lastState = reading;
    return false;
}

bool isButtonHeld(Button* btn) {
    return (btn->currentState == false); // Active low
}

// Check if function shift button is held
bool isFunctionShiftHeld() {
    return isButtonHeld(&funcShiftBtn);
}

// =====================================================================
// AUDIO GENERATION
// =====================================================================

inline int16_t generateSawtooth(uint32_t ph) {
    return (int16_t)(ph >> 16) - 16384;
}

inline int16_t generateSquare(uint32_t ph) {
    return (ph & 0x80000000) ? 16384 : -16384;
}

inline int16_t fastSin(uint32_t ph) {
    uint8_t index = ph >> 24;
    if (index < 128) {
        return (index * 512) - 32768;
    } else {
        return 32768 - ((index - 128) * 512);
    }
}

inline int16_t generateTriangle(uint32_t ph) {
    uint32_t quarter = ph >> 30;
    int16_t val;
    if (quarter == 0) {
        val = (int16_t)((ph >> 15) & 0xFFFF) - 16384;
    } else if (quarter == 1) {
        val = 16384 - (int16_t)((ph >> 15) & 0xFFFF);
    } else if (quarter == 2) {
        val = 16384 - (int16_t)((ph >> 15) & 0xFFFF);
    } else {
        val = (int16_t)((ph >> 15) & 0xFFFF) - 16384;
    }
    return val;
}

inline int16_t applyBitCrusher(int16_t sample, uint8_t bits) {
    if (bits >= 8) return sample; // No crushing at full resolution
    
    int16_t step = 32768 >> bits;
    return (sample / step) * step;
}

inline int16_t applyPortamento(float current, float target, float glide) {
    float diff = target - current;
    if (fabs(diff) < 1.0f) return target;
    return current + (diff * glide);
}

// =====================================================================
// RAY GUN SOUND GENERATION
// =====================================================================

int16_t generateRayGunSample() {
    // Calculate LFO2 modulation factor (same as in generateSample)
    float lfo2Modulation = 1.0f;
    if (lfo2.enabled) {
        float lfo2Value = fastSin(lfo2.phase) / 32768.0f;
        lfo2Modulation = 1.0f + (lfo2Value * lfo2.depth * 0.6f);
    }
    
    // Use LFO modulation like Dub Siren for smooth, musical sweeps
    static uint32_t rayGunLfoPhase = 0;
    static uint32_t noisePhase = 0;
    
    // Update LFO phase
    rayGunLfoPhase += (uint32_t)((siren.sweepSpeed * 4294967296.0f) / SAMPLE_RATE);
    float lfo = fastSin(rayGunLfoPhase) / 32768.0f;
    
    // Mode-specific modulation and waveform
    float modFreq = siren.baseFreq;
    int16_t sample = 0;
    
    switch (siren.rayGunMode) {
        case RAYGUN_ZAP:
            // Classic upward zap - sawtooth wave with upward pitch sweep
            modFreq = siren.baseFreq * (1.0f + (lfo * 0.5f + 0.5f) * siren.resonance * 3.0f);
            {
                float lfo2ModulatedFreq = modFreq * lfo2Modulation;
                uint32_t phaseInc = (uint32_t)((lfo2ModulatedFreq * 4294967296.0f) / SAMPLE_RATE);
                siren.rayGunPhase += phaseInc;
                sample = generateSawtooth(siren.rayGunPhase);
                // Add high-pass character
                sample = sample - (sample >> 2);
            }
            break;
            
        case RAYGUN_LASER:
            // Downward laser blast - square wave with downward sweep and noise
            modFreq = siren.baseFreq * (2.0f - (lfo * 0.5f + 0.5f) * siren.resonance * 1.5f);
            {
                float lfo2ModulatedFreq = modFreq * lfo2Modulation;
                uint32_t phaseInc = (uint32_t)((lfo2ModulatedFreq * 4294967296.0f) / SAMPLE_RATE);
                siren.rayGunPhase += phaseInc;
                sample = generateSquare(siren.rayGunPhase);
                // Add noise burst at start of sweep
                noisePhase = noisePhase * 1664525 + 1013904223; // Linear congruential generator
                int16_t noise = (noisePhase >> 16) - 16384;
                float noiseMix = (lfo > 0.7f) ? 0.3f : 0.0f;
                sample = (int16_t)(sample * (1.0f - noiseMix) + noise * noiseMix);
            }
            break;
            
        case RAYGUN_BLASTER:
            // Rapid fire blaster - triangle wave with fast symmetric modulation
            modFreq = siren.baseFreq * (1.0f + lfo * siren.resonance * 1.5f);
            {
                float lfo2ModulatedFreq = modFreq * lfo2Modulation;
                uint32_t phaseInc = (uint32_t)((lfo2ModulatedFreq * 4294967296.0f) / SAMPLE_RATE);
                siren.rayGunPhase += phaseInc;
                sample = generateTriangle(siren.rayGunPhase);
                // Add pulse-width-like effect
                if (lfo > 0.0f) {
                    sample = sample >> 1;
                }
            }
            break;
            
        case RAYGUN_PHASER:
            // Sci-fi phaser - ring mod effect with harmonic content
            modFreq = siren.baseFreq * (1.0f + lfo * siren.resonance * 2.0f);
            {
                float lfo2ModulatedFreq = modFreq * lfo2Modulation;
                uint32_t phaseInc = (uint32_t)((lfo2ModulatedFreq * 4294967296.0f) / SAMPLE_RATE);
                siren.rayGunPhase += phaseInc;
                // Mix square and modulated square for ring-mod effect
                int16_t square1 = generateSquare(siren.rayGunPhase);
                int16_t square2 = generateSquare(siren.rayGunPhase * 3);
                sample = (square1 >> 1) + (square2 >> 2);
            }
            break;
    }
    
    // Keep frequency in valid range
    if (modFreq < 20.0f) modFreq = 20.0f;
    if (modFreq > 8000.0f) modFreq = 8000.0f;
    
    // Apply resonant filter (more aggressive for ray gun sounds)
    filterState = filterState + (((sample - filterState) * (int32_t)(siren.resonance * 4095)) >> 12);
    int16_t filteredSample = (int16_t)filterState;
    
    // Store for waveform display
    lastSample = filteredSample;
    
    // Apply gate (only sound when triggered)
    if (!siren.triggered) {
        filteredSample = 0;
        siren.rayGunPhase = 0;
        rayGunLfoPhase = 0;
        filterState = 0;
    }
    
    return filteredSample;
}

// Handle alternative functions when Function + Key is pressed
void handleFunctionKey(uint8_t keyNum) {
    Serial.print("Function + Key ");
    Serial.print(keyNum);
    Serial.println(" pressed");
    
    switch (keyNum) {
        case 1:
            // Activate sequencer mode
            // (to exit: double-press Function Shift button)
           if (!sequencer.active) {
    sequencer.active = true;
    Serial.println("-> SEQUENCER MODE ACTIVATED");
    
    // Initialize sequencer with a sensible bass scale
    // Use 110Hz (A2) as the root - nice bass range
    const float baseRoot = 110.0f;  // A2
    
    // Get current scale
    const float* scale;
    uint8_t scaleSize;
    switch (mutate.currentScale) {
        case SCALE_MINOR_PENT: scale = minorPentatonic; scaleSize = minorPentatonicSize; break;
        case SCALE_MAJOR: scale = majorScale; scaleSize = majorScaleSize; break;
        case SCALE_BLUES: scale = bluesScale; scaleSize = bluesScaleSize; break;
        case SCALE_MINOR: scale = minorScale; scaleSize = minorScaleSize; break;
        case SCALE_CHROMATIC: scale = chromaticScale; scaleSize = chromaticScaleSize; break;
        default: scale = minorPentatonic; scaleSize = minorPentatonicSize; break;
    }
    
    // Map 8 steps across one octave of the scale
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t scaleIndex = i % scaleSize;
        uint8_t octave = i / scaleSize;
        sequencer.stepFrequencies[i] = baseRoot * scale[scaleIndex] * (1 << octave);
        
        Serial.print("Step ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(sequencer.stepFrequencies[i]);
        Serial.println("Hz");
    }
    sequencer.running = false;
    sequencer.currentStep = 0;
    
    // Force complete display redraw
    display.fillScreen(COLOR_BG);
    
    // Draw all display elements
    drawParameters();      
    drawWaveform(true);
    
    // CRITICAL: Draw sub-headings AFTER everything else, with a small delay
    delay(50);  // Let display settle
    
    // Force draw sub-headings manually
    display.setTextSize(1);
    
    // GATE
    display.fillRect(245, 8, 75, 8, COLOR_BG);
    display.setCursor(245, 8);
    display.setTextColor(COLOR_MOD);
    display.print("GATE:");
    display.setTextColor(COLOR_TEXT);
    display.print(sequencer.gateLength);
    display.print("%");
    
    // OCTAVE
    display.fillRect(245, 20, 75, 8, COLOR_BG);
    display.setCursor(245, 20);
    display.setTextColor(COLOR_MOD);
    display.print("OCTAVE:");
    display.setTextColor(COLOR_TEXT);
    if (sequencer.octaveShift == 0) {
        display.print("0");
    } else if (sequencer.octaveShift > 0) {
        display.print("+");
        display.print(sequencer.octaveShift);
    } else {
        display.print(sequencer.octaveShift);
    }
    
    // DELAY
    display.fillRect(245, 32, 75, 8, COLOR_BG);
    display.setCursor(245, 32);
    display.setTextColor(COLOR_MOD);
    display.print("DELAY:");
    display.setTextColor(COLOR_TEXT);
    display.print(delayTimeNames[siren.delayIndex]);
}
                
            // Note: Turning OFF is handled in readKeyboard() with hold detection
            break;
            
           case 2:
    // Activate LOOP mode
    if (!loopState.active) {
        loopState.active = true;
        loopState.pattern = LOOP_TRIPLET; // Start with TRIPLET pattern
        loopState.patternStartTime = millis();
        
        Serial.println("-> LOOP MODE ACTIVATED");
        Serial.println("   Press Key 2 to cycle patterns");
        Serial.println("   Double-press Function button to EXIT");
        
        // Force full redraw
        display.fillScreen(COLOR_BG);
        drawParameters(true);
        drawWaveform(true);
    }
    break;
        case 3:
            // Activate RECORD mode
            if (!recorder.active) {
                recordKeySilence = true;  //  silence sound
                recorder.active = true;
                Serial.println("-> RECORD MODE ACTIVATED");
                Serial.println("   Press Key 3 to START recording");
                Serial.println("   Press Key 3 again to STOP recording");
                Serial.println("   Press Key 3 to PLAY recording");
                Serial.println("   Press Key 4 to toggle LOOP on/off");
                Serial.println("   Double-press Function button to EXIT");
                
                // Force full redraw to show RECORD and LOOP sub-headings
                drawParameters(true);  // Force full redraw
                drawWaveform(true);    // Redraw waveform with proper skip zones
            }
            break;
        case 4:
    // Activate REVERSE mode
    if (!reverser.active) {
        reverser.active = true;
        reverser.reversing = false;  // Start with reverse OFF
        reverser.writePos = 0;       // Reset write position
        
        // Don't clear buffer - just let it fill naturally
        
        Serial.println("-> REVERSE MODE ACTIVATED");
        Serial.println("   Press Key 4 to toggle REVERSE on/off");
        Serial.println("   Pot 1: Pitch Shift (-2 to +2 octaves)");
        Serial.println("   Pot 3: Wet/Dry Mix (0-100%)");
        Serial.println("   Double-press Function button to EXIT");
        
        // Just update parameters - no screen clear, no delays
        drawParameters(true);
        drawWaveform(false);
    }
    break;
      case 5:
            // Activate HOLD mode (like REVERSE mode)
            if (!infiniteSustain) {
                infiniteSustain = true;
                
                // Save current feedback to restore when exiting mode
                savedFeedback = siren.echoFeedback;
                
                // Start with hold OFF (user presses Key 5 to turn ON)
                // Keep current feedback value
                
                Serial.println("-> HOLD MODE ACTIVATED");
                Serial.println("   Press Key 5 to toggle HOLD on/off");
                Serial.println("   When ON: Delay feedback set to 100% (infinite loop)");
                Serial.println("   Double-press Function button to EXIT");
                
                // Update display
                drawParameters(true);
            }
            break;
        case 6:
    // Activate LFO 2 mode
    if (!lfo2.active) {
        lfo2.active = true;
        lfo2.enabled = false;  // Start with LFO 2 OFF
        
        Serial.println("-> LFO 2 MODE ACTIVATED");
        Serial.println("   Press Key 6 to toggle LFO 2 on/off");
        Serial.println("   Pot 1: LFO 2 Rate (0.1-20 Hz)");
        Serial.println("   Pot 3: LFO 2 Depth (0-100%)");
        Serial.println("   Double-press Function button to EXIT");
        
        // Force complete display redraw (same as sequencer/loop)
        display.fillScreen(COLOR_BG);
        drawParameters(true);
        drawWaveform(true);
        
        // CRITICAL: Draw LFO2 sub-heading immediately after everything else
        delay(50);
        
        display.setTextSize(1);
        display.fillRect(5, 47, 120, 10, COLOR_BG);
        display.setCursor(5, 47);
        display.setTextColor(COLOR_MOD);
        display.print("LFO2:");
        display.setTextColor(COLOR_GRID);
        display.print("OFF");
    }
    break;
        case 7:
            // Activate DRONE mode
            if (!drone.active) {
                drone.active = true;
                drone.playMode = 0;     // Start with OFF
                drone.envelope = 0.0f;  
                drone.fadeStartTime = millis();
                
                // Initialize oscillator phases
                for (int i = 0; i < 5; i++) {
                    drone.oscPhases[i] = 0;
                    drone.driftPhases[i] = (uint32_t)i * 858993459u;
                }
                
                Serial.println("-> DRONE MODE ACTIVATED");
                Serial.println("   Press TRIGGER to cycle: OFF -> KEYS -> ON");
                Serial.println("   OFF: Silent");
                Serial.println("   KEYS: Sound only when keys pressed");
                Serial.println("   ON: Continuous drone");
                Serial.println("   Pot 1: Root Pitch (20-400Hz)");
                Serial.println("   Pot 2: Detune Amount (0-100%)");
                Serial.println("   Pot 3: Brightness (0-100%)");
                Serial.println("   Pot 4: Drift Speed (0.1-5Hz)");
                Serial.println("   Double-press Function button to EXIT");
                
                // Force complete display redraw
                display.fillScreen(COLOR_BG);
                drawParameters(true);
                drawWaveform(true);
                
                delay(50);
                
                // Draw DRONE sub-heading with initial values
                display.setTextSize(1);
                display.fillRect(5, 47, 138, 10, COLOR_BG);
                display.setCursor(5, 47);
                display.setTextColor(COLOR_MOD);
                display.print("DRONE:");
                display.setTextColor(COLOR_GRID);
                display.print("OFF ");
                display.setTextColor(COLOR_TEXT);
                display.print((int)drone.rootFreq);
                display.print("Hz ");
                display.print((int)(drone.detune * 100));
                display.print("%");
            }
            break;
        case 8:
            // Activate SIDECHAIN mode
            if (!sidechain.active) {
                sidechain.active = true;
                sidechain.enabled = false;  // Start with sidechain OFF
                sidechain.envelope = 1.0f;   // Start at full volume
                
                Serial.println("-> SIDECHAIN MODE ACTIVATED");
                Serial.println("   Press Key 8 to toggle SIDECHAIN on/off");
                Serial.println("   Pot 1: Ducking Depth (0-100%)");
                Serial.println("   Pot 2: Attack Time (1-50ms)");
                Serial.println("   Pot 3: Release Time (50-500ms)");
                Serial.println("   Syncs with LOOP patterns or SYNC input");
                Serial.println("   Double-press Function button to EXIT");
                
                forceDisplayUpdate = true;  // Force UI update
                
                // Force complete display redraw
                display.fillScreen(COLOR_BG);
                drawParameters(true);
                drawWaveform(true);
                
                delay(50);
                
                // Draw SIDECHAIN sub-heading - MATCH the drawParameters() format exactly
                display.setTextSize(1);
                display.fillRect(5, 47, 150, 10, COLOR_BG);  // Wider clear area
                display.setCursor(5, 47);
                display.setTextColor(COLOR_MOD);
                display.print("SIDECHAIN:");
                display.setTextColor(COLOR_GRID);  // Gray when OFF
                display.print("OFF ");
                // Show depth/attack/release immediately
                display.print((int)(sidechain.depth * 100));
                display.print("% ");
                display.print((int)sidechain.attackTime);
                display.print("/");
                display.print((int)sidechain.releaseTime);
                display.print("ms");
            }
            break;
    }
}

// =====================================================================
// LEAD SYNTH SOUND GENERATION
// =====================================================================

int16_t generateLeadSynthSample() {
    // Calculate LFO2 modulation factor (same as in generateSample)
    float lfo2Modulation = 1.0f;
    if (lfo2.enabled) {
        float lfo2Value = fastSin(lfo2.phase) / 32768.0f;
        lfo2Modulation = 1.0f + (lfo2Value * lfo2.depth * 0.6f);
    }
    
    // Always generate preview for waveform display
    static uint32_t previewPhase = 0;
    
    // Update sequence timing
    unsigned long currentTime = millis();
    
    // === ONLY advance sequence when triggered ===
    if (siren.triggered && (currentTime - siren.lastStepTime >= siren.stepInterval)) {
        siren.lastStepTime = currentTime;
        siren.currentStep = (siren.currentStep + 1) % siren.stepCount;
        siren.noteStartTime = currentTime;
        siren.noteActive = true;
        
        // Regenerate pattern for generative mode
        if (siren.leadSynthMode == LEAD_GENERATIVE && siren.currentStep == 0) {
            generateMelody();
        } else if (siren.currentStep == 0) {
            generateMelody();
        }
    }
    
    // Check if note should be gated off
    unsigned long noteElapsed = currentTime - siren.noteStartTime;
    unsigned long noteDuration = (siren.stepInterval * siren.gateLength) / 100;
    if (noteElapsed >= noteDuration) {
        siren.noteActive = false;
    }
    
    // Get current note frequency
    float noteFreq = siren.noteFrequencies[siren.currentStep];
    
    // Apply modulation based on mode
    switch (siren.leadSynthMode) {
        case LEAD_SEQUENCE:
            break;
            
        case LEAD_ARPEGGIO:
            if (siren.currentStep % 4 == 3) {
                noteFreq *= 2.0f;
            }
            break;
            
        case LEAD_EUCLIDEAN:
            if (!isEuclideanHit(siren.currentStep, siren.stepCount, siren.stepCount * 3 / 4)) {
                noteFreq = 0;
            }
            break;
            
        case LEAD_GENERATIVE:
            break;
    }
    
    // Add vibrato (only if depth > 0)
    float finalFreq;
    if (siren.vibratoDepth > 0.001f) {
        float vibratoMod = sin(currentTime * 0.001f * siren.vibratoRate * 6.28318f) * siren.vibratoDepth;
        finalFreq = noteFreq * (1.0f + vibratoMod);
    } else {
        finalFreq = noteFreq; // No vibrato - perfectly in tune
    }
    
    // === ALWAYS generate preview waveform ===
    if (finalFreq > 0) {
        uint32_t previewPhaseInc = (uint32_t)((finalFreq * 4294967296.0f) / SAMPLE_RATE);
        previewPhase += previewPhaseInc;
        
        // Mix saw and square
        int16_t saw = generateSawtooth(previewPhase);
        int16_t square = generateSquare(previewPhase);
        int16_t previewSample = (saw >> 1) + (square >> 2);
        
        // Apply simple envelope
        float envelope = 1.0f;
        if (noteElapsed < 50) {
            envelope = noteElapsed / 50.0f;
        } else if (noteElapsed > noteDuration - 50) {
            envelope = (noteDuration - noteElapsed) / 50.0f;
        }
        
        previewSample = (int16_t)(previewSample * envelope);
        
        // Store for display (quieter if not triggered)
        if (!siren.triggered) {
            lastSample = previewSample >> 2; // Quieter preview
        } else {
            lastSample = previewSample;
        }
    } else {
        lastSample = 0;
        previewPhase = 0;
    }
    
    // === If not triggered, return silence for audio ===
    if (!siren.triggered) {
        siren.leadPhase = 0;
        return 0; // SILENT AUDIO OUTPUT
    }
    
    // === TRIGGERED - Generate actual audio ===
    int16_t outputSample = 0;
    
    if (finalFreq > 0 && siren.noteActive) {
        // Apply LFO2 modulation for audible vibrato effect
        float lfo2ModulatedFreq = finalFreq * lfo2Modulation;
        uint32_t phaseInc = (uint32_t)((lfo2ModulatedFreq * 4294967296.0f) / SAMPLE_RATE);
        siren.leadPhase += phaseInc;
        
        // Mix saw and square
        int16_t saw = generateSawtooth(siren.leadPhase);
        int16_t square = generateSquare(siren.leadPhase);
        outputSample = (saw >> 1) + (square >> 2);
        
        // Apply envelope
        float envelope = 1.0f;
        if (noteElapsed < 50) {
            envelope = noteElapsed / 50.0f;
        } else if (noteElapsed > noteDuration - 50) {
            envelope = (noteDuration - noteElapsed) / 50.0f;
        }
        
        outputSample = (int16_t)(outputSample * envelope);
    } else {
        siren.leadPhase = 0;
    }
    
    return outputSample;
}

// Helper function for Euclidean rhythm
bool isEuclideanHit(uint8_t step, uint8_t steps, uint8_t pulses) {
    if (pulses >= steps) return true;
    if (pulses == 0) return false;
    return (step * pulses) % steps < pulses;
}

// Helper function to generate melodic patterns
void generateMelody() {
    // Generate a new melodic pattern based on a scale
    float rootFreq = siren.baseFreq;
    
    // Minor pentatonic scale intervals
    const float scale[] = {1.0f, 1.2f, 1.33f, 1.5f, 1.78f, 2.0f};
    const uint8_t scaleSize = 6;
    
    for (uint8_t i = 0; i < siren.stepCount; i++) {
        // Random walk through scale
        uint8_t noteIndex = (millis() + i * 37) % scaleSize;
        siren.noteFrequencies[i] = rootFreq * scale[noteIndex];
    }
}

// Calculate keyboard frequencies based on current scale
void updateKeyboardFrequencies() {
    // Use current mutation scale setting - controlled by MUTATE button + RHYTHM mode!
    const float* scale;
    uint8_t scaleSize;
    
    switch (mutate.currentScale) {
        case SCALE_MINOR_PENT: scale = minorPentatonic; scaleSize = minorPentatonicSize; break;
        case SCALE_MAJOR: scale = majorScale; scaleSize = majorScaleSize; break;
        case SCALE_BLUES: scale = bluesScale; scaleSize = bluesScaleSize; break;
        case SCALE_MINOR: scale = minorScale; scaleSize = minorScaleSize; break;
        case SCALE_CHROMATIC: scale = chromaticScale; scaleSize = chromaticScaleSize; break;
        default: scale = minorPentatonic; scaleSize = minorPentatonicSize; break;
    }
    
    // Base frequency from Pot 1 (or baseFreq)
    float rootFreq = siren.baseFreq;
    
    // Map 8 keys across the scale
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t scaleIndex = i % scaleSize;
        uint8_t octave = i / scaleSize;
        
        // Apply octave shift
        int totalOctave = (int)octave + octaveShift;
        if (totalOctave < 0) totalOctave = 0;
        if (totalOctave > 4) totalOctave = 4;
        
        keyboard.keyFrequencies[i] = rootFreq * scale[scaleIndex] * (1 << totalOctave);
    }
}

// Read keyboard and update frequency
void readKeyboard() {
    // Update key frequencies based on current scale
    bool funcShift = isFunctionShiftHeld();

// ===== HOLD MODE KEY HANDLING - MUST BE FIRST =====
    if (infiniteSustain) {
        // Use direct GPIO read for KEY_5 (GP17) toggle
        static bool lastKey5State = true;
        bool key5Pressed = (digitalRead(KEY_5) == LOW);  // KEY_5 is GP17
        
        // DEBUG output
        if (key5Pressed) {
            Serial.println("DEBUG: Key 5 pressed in HOLD mode");
        }
        
        // Detect button press (falling edge)
        if (key5Pressed && lastKey5State) {
            holdIsOn = !holdIsOn;
            
            if (holdIsOn) {
                // Turn ON - set to high feedback (controlled by Pot 1)
                siren.echoFeedback = 1.0f;  // Start at 100% feedback
                Serial.println("HOLD ON - Infinite loop active");
            } else {
                // Turn OFF - restore saved feedback
                siren.echoFeedback = savedFeedback;
                Serial.println("HOLD OFF - Normal feedback restored");
            }
            
            drawParameters();
        }
        
        lastKey5State = key5Pressed;
        
        // Continue to normal keyboard processing - DON'T RETURN
    }

    // ===== LFO 2 MODE KEY HANDLING =====
    if (lfo2.active) {
        // Use direct GPIO read for Key 6 toggle
        static bool lastKey6State = true;
        bool key6Pressed = (digitalRead(KEY_6) == LOW);
        
        // Detect button press (falling edge)
        if (key6Pressed && lastKey6State) {
            lfo2.enabled = !lfo2.enabled;
            Serial.print("LFO 2 ");
            Serial.println(lfo2.enabled ? "ON" : "OFF");
            drawParameters();
        }
        
        lastKey6State = key6Pressed;
        
        // DON'T process Key 6 normally - it's used for LFO2 toggle only
        // But continue to process other keys for normal keyboard functionality
    }


// ===== REVERSE MODE KEY HANDLING =====
    if (reverser.active) {
        // Use direct GPIO read for Key 4 toggle only
        static bool lastKey4State = true;
        bool key4Pressed = (digitalRead(KEY_4) == LOW);
        
        // Detect button press (falling edge)
        if (key4Pressed && lastKey4State) {
            reverser.reversing = !reverser.reversing;
            Serial.print("REVERSE ");
            Serial.println(reverser.reversing ? "ON" : "OFF");
            drawParameters();
        }
        
        lastKey4State = key4Pressed;
 
        // Continue to normal keyboard processing for pitch changes
        // When keys pressed, update drone root frequency in real-time
        for (int8_t i = 7; i >= 0; i--) {
            if (keyboard.keysHeld[i]) {
                drone.rootFreq = keyboard.keyFrequencies[i];
                break; // Use highest priority key
            }
        }
    }

// ===== HOLD MODE KEY HANDLING =====
    if (infiniteSustain) {
        // Use direct GPIO read for KEY_5 (GP17) toggle
        static bool lastKey5State = true;
        static bool holdIsOn = false;
        bool key5Pressed = (digitalRead(KEY_5) == LOW);  // KEY_5 is GP17
        
        // Detect button press (falling edge)
        if (key5Pressed && lastKey5State) {
            holdIsOn = !holdIsOn;
            
            if (holdIsOn) {
                // Turn ON - set to high feedback (controlled by Pot 1)
                siren.echoFeedback = 1.0f;  // Start at 100% feedback
                Serial.println("HOLD ON - Infinite loop active");
            } else {
                // Turn OFF - restore saved feedback
                siren.echoFeedback = savedFeedback;
                Serial.println("HOLD OFF - Normal feedback restored");
            }
            
            drawParameters();
        }
        
        lastKey5State = key5Pressed;
        
        // Continue to normal keyboard processing - DON'T RETURN
    }

// ===== RECORD MODE KEY HANDLING =====
    if (recorder.active) {
        // Use direct GPIO read with simple state tracking for instant response
        static bool lastKey3State = true;  // Start HIGH (not pressed)
        static bool lastKey4State = true;  // Start HIGH (not pressed)
        bool key3Pressed = (digitalRead(KEY_3) == LOW);
        bool key4Pressed = (digitalRead(KEY_4) == LOW);
        
        // Silence sound during key operations
        recordKeySilence = (key3Pressed || key4Pressed);
        
        // Key 3: Record/Play control
        // Detect falling edge (button just pressed)
        if (key3Pressed && !lastKey3State) {
            // Key 3 just pressed - cycle through record states
            if (!recorder.recording && !recorder.playing && !recorder.hasRecording) {
                // State 1: READY -> Start recording
                recorder.recording = true;
                recorder.recordPosition = 0;
                recorder.recordLength = 0;
                recorder.recordStartTime = millis();
                Serial.println("RECORDING STARTED...");
                drawParameters(); // Force immediate display update
                
            } else if (recorder.recording) {
                // State 2: RECORDING -> Stop and go to READY with recording
                recorder.recording = false;
                recorder.hasRecording = true;
                recorder.recordLength = recorder.recordPosition;
                Serial.print("RECORDING STOPPED - ");
                Serial.print(recorder.recordLength);
                Serial.print(" samples (");
                Serial.print(recorder.recordLength / 22.05f);
                Serial.println("ms)");
                Serial.println("Press Key 3 to PLAY");
                drawParameters(); // Force immediate display update
                
            } else if (recorder.hasRecording && !recorder.playing) {
                // State 3: READY (with recording) -> Start playback
                recorder.playing = true;
                recorder.recordPosition = 0;
                Serial.println("PLAYBACK STARTED...");
                drawParameters(); // Force immediate display update
                
            } else if (recorder.playing) {
                // State 4: PLAYING -> Stop playback
                recorder.playing = false;
                recorder.recordPosition = 0;
                Serial.println("PLAYBACK STOPPED");
                drawParameters(); // Force immediate display update
            }
        }
        
        // Key 4: Toggle loop/repeat mode
        if (key4Pressed && !lastKey4State) {
            recorder.loopEnabled = !recorder.loopEnabled;
            Serial.print("LOOP ");
            Serial.println(recorder.loopEnabled ? "ON" : "OFF");
            drawParameters(); // Force immediate display update
        }
        
        // Update state tracking AFTER all logic
        lastKey3State = key3Pressed;
        lastKey4State = key4Pressed;
        
        return; // Don't process normal keyboard in record mode
    }
    
    // ===== READ ALL KEYS ONCE (for non-record modes) =====
    keyboard.keysHeld[0] = !digitalRead(KEY_1);
    keyboard.keysHeld[1] = !digitalRead(KEY_2);
    keyboard.keysHeld[2] = !digitalRead(KEY_3);
    keyboard.keysHeld[3] = !digitalRead(KEY_4);
    keyboard.keysHeld[4] = !digitalRead(KEY_5);
    keyboard.keysHeld[5] = lfo2.active ? false : !digitalRead(KEY_6);  // Disable Key 6 when LFO2 active
    keyboard.keysHeld[6] = !digitalRead(KEY_7);
    keyboard.keysHeld[7] = !digitalRead(KEY_8);
    
    // Count held keys
    keyboard.numKeysHeld = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (keyboard.keysHeld[i]) {
            keyboard.numKeysHeld++;
        }
    }

// ===== FUNCTION KEY HANDLER - MUST COME FIRST! =====
if (funcShift) {
    // Use direct GPIO reads for instant response
    static bool lastFuncKey1 = true;
    static bool lastFuncKey2 = true;
    static bool lastFuncKey3 = true;
    static bool lastFuncKey4 = true;
    static bool lastFuncKey5 = true;
    static bool lastFuncKey6 = true;
    static bool lastFuncKey7 = true;
    static bool lastFuncKey8 = true;
    static unsigned long lastKey2RepeatTime = 0;
    
    bool key1 = digitalRead(KEY_1);
    bool key2 = digitalRead(KEY_2);
    bool key3 = digitalRead(KEY_3);
    bool key4 = digitalRead(KEY_4);
    bool key5 = digitalRead(KEY_5);
    bool key6 = digitalRead(KEY_6);
    bool key7 = digitalRead(KEY_7);
    bool key8 = digitalRead(KEY_8);
    
    unsigned long now = millis();
    
    // Key 1 - SEQUENCER
    if (key1 == LOW && lastFuncKey1 == HIGH) {
        Serial.println("Function + Key 1");
        handleFunctionKey(1);
    }
    lastFuncKey1 = key1;
    
    // Key 2 - LOOP (with repeat)
    if (key2 == LOW && lastFuncKey2 == HIGH) {
        Serial.println("Function + Key 2");
        handleFunctionKey(2);
        lastKey2RepeatTime = now;
    } else if (key2 == LOW && (now - lastKey2RepeatTime > 300)) {
        // Allow repeat every 300ms while held
        handleFunctionKey(2);
        lastKey2RepeatTime = now;
    }
    lastFuncKey2 = key2;
    
    // Key 3 - RECORD
    if (key3 == LOW && lastFuncKey3 == HIGH) {
        Serial.println("Function + Key 3");
        handleFunctionKey(3);
    }
    lastFuncKey3 = key3;
    
    // Key 4 - REVERSE
    if (key4 == LOW && lastFuncKey4 == HIGH) {
        Serial.println("Function + Key 4");
        handleFunctionKey(4);
    }
    lastFuncKey4 = key4;
    
    // Key 5 - HOLD
    if (key5 == LOW && lastFuncKey5 == HIGH) {
        Serial.println("Function + Key 5");
        handleFunctionKey(5);
    }
    lastFuncKey5 = key5;
    
    // Key 6 - LFO 2
    if (key6 == LOW && lastFuncKey6 == HIGH) {
        Serial.println("Function + Key 6");
        handleFunctionKey(6);
    }
    lastFuncKey6 = key6;
    
    // Key 7
    if (key7 == LOW && lastFuncKey7 == HIGH) {
        Serial.println("Function + Key 7");
        handleFunctionKey(7);
    }
    lastFuncKey7 = key7;
    
    // Key 8
    if (key8 == LOW && lastFuncKey8 == HIGH) {
        Serial.println("Function + Key 8");
        handleFunctionKey(8);
    }
    lastFuncKey8 = key8;
    
    return; // Exit early - don't process normal keyboard
}

// CRITICAL: Reset function key states when exiting special modes
if (!sequencer.active && !recorder.active) {
    static bool lastFuncKeyStates[8] = {false, false, false, false, false, false, false, false};
    // Force reset of all function key states
    for (uint8_t i = 0; i < 8; i++) {
        lastFuncKeyStates[i] = false;
    }
}
    
    // === SEQUENCER MODE KEY HANDLING ===
    if (sequencer.active) {
    // In sequencer mode:
    // - Short press = toggle step on/off
    // - Long hold = adjust pitch (don't toggle)
    static bool lastSeqKeyStates[8] = {false, false, false, false, false, false, false, false};
    static unsigned long keyPressTime[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    static bool keyHeldForAdjustment[8] = {false, false, false, false, false, false, false, false};
    
    keyboard.keysHeld[0] = !digitalRead(KEY_1);
    keyboard.keysHeld[1] = !digitalRead(KEY_2);
    keyboard.keysHeld[2] = !digitalRead(KEY_3);
    keyboard.keysHeld[3] = !digitalRead(KEY_4);
    keyboard.keysHeld[4] = !digitalRead(KEY_5);
    keyboard.keysHeld[5] = !digitalRead(KEY_6);
    keyboard.keysHeld[6] = !digitalRead(KEY_7);
    keyboard.keysHeld[7] = !digitalRead(KEY_8);
    
    for (uint8_t i = 0; i < 8; i++) {
        // Detect key press start
        if (keyboard.keysHeld[i] && !lastSeqKeyStates[i]) {
            // Key just pressed - record time
            keyPressTime[i] = millis();
            keyHeldForAdjustment[i] = false;
        }
        
        // Check if key is being held long enough for pitch adjustment (200ms threshold)
if (keyboard.keysHeld[i] && (millis() - keyPressTime[i] > 200)) {
    keyHeldForAdjustment[i] = true;
    
    // Read pot value directly for immediate response
    adc_select_input(0); // Pitch pot
    uint16_t pitchRaw = adc_read();
    
    // Map pot to 24 steps (2 octaves) with hysteresis to prevent jumping
    static uint8_t lastPotStep[8] = {0,0,0,0,0,0,0,0};
    uint8_t potStep = (pitchRaw * 24) / 4096;
    if (potStep >= 24) potStep = 23;
    
    // Only update if pot moved significantly (prevents micro-drifts)
    if (abs(potStep - lastPotStep[i]) > 0) {
        // Quantize to musical pitch - LOCK to exact frequency
        sequencer.stepFrequencies[i] = quantizePitchToScale(110.0f, potStep);
        lastPotStep[i] = potStep;
    }
}
        
        // Detect key release
        if (!keyboard.keysHeld[i] && lastSeqKeyStates[i]) {
            // Key just released
            if (!keyHeldForAdjustment[i]) {
                // Short press - toggle step on/off
                sequencer.stepEnabled[i] = !sequencer.stepEnabled[i];
                Serial.print("Step ");
                Serial.print(i + 1);
                Serial.println(sequencer.stepEnabled[i] ? " ON" : " OFF");
            } else {
                // Long hold - pitch was adjusted, don't toggle
                Serial.print("Step ");
                Serial.print(i + 1);
                Serial.print(" pitch set to ");
                Serial.print(sequencer.stepFrequencies[i]);
                Serial.println("Hz");
            }
            keyHeldForAdjustment[i] = false;
        }
        
        lastSeqKeyStates[i] = keyboard.keysHeld[i];
    }
    
    return; // Don't process normal keyboard functions in sequencer mode
}

// === SIMPLE KEYBOARD MODE - Just play the held key(s) ===
    keyboard.keyPressed = (keyboard.numKeysHeld > 0);
    
    if (keyboard.numKeysHeld > 0) {
        // Find the highest priority held key (last in array = highest priority)
        for (int8_t i = 7; i >= 0; i--) {
            if (keyboard.keysHeld[i]) {
                siren.baseFreq = keyboard.keyFrequencies[i];
                siren.frequency = siren.baseFreq;
                keyboard.currentKey = i + 1;
                break; // Use first (highest priority) key found
            }
        }
    }
    
    // ===== DRONE MODE - Update root frequency from keys =====
    if (drone.active && keyboard.numKeysHeld > 0) {
        // Update drone root frequency from held keys
        for (int8_t i = 7; i >= 0; i--) {
            if (keyboard.keysHeld[i]) {
                drone.rootFreq = keyboard.keyFrequencies[i];
                break; // Use highest priority key
            }
        }
    }
}

// =====================================================================
// DISCO SOUND GENERATION
// =====================================================================

int16_t generateDiscoSample() {
    // Calculate LFO2 modulation factor (same as in generateSample)
    float lfo2Modulation = 1.0f;
    if (lfo2.enabled) {
        float lfo2Value = fastSin(lfo2.phase) / 32768.0f;
        lfo2Modulation = 1.0f + (lfo2Value * lfo2.depth * 0.6f);
    }
    
    // Calculate time since trigger
    unsigned long elapsed = millis() - siren.discoStartTime;
    float elapsedSec = elapsed / 1000.0f;
    
    // DEBUG OUTPUT
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        Serial.print("DISCO MODE: ");
        Serial.print(discoModeNames[siren.discoMode]);
        Serial.print(" | triggered=");
        Serial.print(siren.triggered);
        Serial.print(" | elapsed=");
        Serial.print(elapsedSec);
        Serial.print("s | baseFreq=");
        Serial.println(siren.baseFreq);
        lastDebug = millis();
    }
    
    int16_t sample = 0;
    float envelope = 0.0f;
    float currentFreq = siren.baseFreq;
    
    switch (siren.discoMode) {
        case DISCO_ORCHESTRA_HIT:
        {
            // Punchy orchestral hit - short, bright, dramatic
            float attackTime = 0.005f; // 5ms attack
            float decayTime = 0.3f + (siren.discoCharacter * 0.7f); // 300-1000ms decay (Pot 3)
            
            if (elapsedSec < attackTime) {
                envelope = elapsedSec / attackTime;
            } else if (elapsedSec < decayTime) {
                envelope = 1.0f - ((elapsedSec - attackTime) / decayTime);
            } else {
                envelope = 0.0f;
            }
            
            // Bright harmonic stack (1st, 3rd, 5th harmonics)
            uint32_t phaseInc = (uint32_t)((currentFreq * 4294967296.0f) / SAMPLE_RATE);
            siren.discoPhase += phaseInc;
            
            int16_t fundamental = generateSawtooth(siren.discoPhase);
            int16_t third = generateSawtooth(siren.discoPhase * 3);
            int16_t fifth = generateSawtooth(siren.discoPhase * 5);
            
            sample = (fundamental + (third >> 1) + (fifth >> 2)) / 2;
            
            // Filter sweep DOWN (starts bright, closes to dark)
            float filterEnv = 1.0f - (elapsedSec / decayTime);
            if (filterEnv < 0.0f) filterEnv = 0.0f;
            int32_t dynamicCutoff = 500 + (int32_t)(filterEnv * 3500); // 500-4000Hz sweep
            
            static int32_t orchFilterState = 0;
            orchFilterState = orchFilterState + (((sample - orchFilterState) * dynamicCutoff) >> 12);
            sample = (int16_t)orchFilterState;
            
            sample = (int16_t)(sample * envelope * siren.discoBrightness * 2.0f); // 2x volume boost
            break;
        }
        
        case DISCO_STRING_SWEEP:
        {
            // Lush rising string machine - slow attack, sustain while held
            float attackTime = 0.5f + (siren.discoCharacter * 2.0f); // 0.5-2.5s attack (Pot 2)
            
            if (elapsedSec < attackTime) {
                envelope = elapsedSec / attackTime; // Rising
            } else if (siren.triggered) {
                envelope = 1.0f; // Full sustain ONLY while trigger held
            } else {
                // Release when trigger released
                float releaseTime = 0.5f; // 500ms release
                float releaseElapsed = elapsedSec - attackTime;
                if (releaseElapsed < releaseTime) {
                    envelope = 1.0f - (releaseElapsed / releaseTime);
                } else {
                    envelope = 0.0f;
                }
            }
            
            // Rich detuned voices (3 voices for chorus effect)
            uint32_t phaseInc = (uint32_t)((currentFreq * 4294967296.0f) / SAMPLE_RATE);
            siren.discoPhase += phaseInc;
            
            int16_t voice1 = generateSawtooth(siren.discoPhase);
            int16_t voice2 = generateSawtooth((uint32_t)(siren.discoPhase * 1.01f)); // +1% detune
            int16_t voice3 = generateSawtooth((uint32_t)(siren.discoPhase * 0.99f)); // -1% detune
            
            sample = (voice1 + voice2 + voice3) / 3;
            
            // Filter sweep UP (opens gradually)
            int32_t dynamicCutoff = 300 + (int32_t)(envelope * 3700); // 300-4000Hz sweep
            
            static int32_t stringFilterState = 0;
            stringFilterState = stringFilterState + (((sample - stringFilterState) * dynamicCutoff) >> 12);
            sample = (int16_t)stringFilterState;
            
            // Add subtle vibrato
            float vibrato = sin(elapsedSec * 5.0f * 6.28318f) * 0.02f;
            sample = (int16_t)(sample * (1.0f + vibrato) * envelope);
            break;
        }
        
        case DISCO_FUNK_BLAST:
        {
            // Punchy synth brass - fast attack, pitch bend down
            float attackTime = 0.01f; // 10ms attack
            float decayTime = 0.2f + (siren.discoCharacter * 0.3f); // 200-500ms (Pot 3)
            
            if (elapsedSec < attackTime) {
                envelope = elapsedSec / attackTime;
            } else if (elapsedSec < decayTime) {
                envelope = 1.0f - ((elapsedSec - attackTime) / (decayTime - attackTime));
            } else {
                envelope = 0.0f;
            }
            
            // Pitch bend down at start (first 50ms)
            float pitchBend = 1.0f;
            if (elapsedSec < 0.05f) {
                pitchBend = 1.0f + ((0.05f - elapsedSec) * 0.4f); // Bend down from +40%
            }
            
            float bendFreq = currentFreq * pitchBend;
            // Constrain to audible range
            if (bendFreq > 1500.0f) bendFreq = 1500.0f;
            if (bendFreq < 100.0f) bendFreq = 100.0f;
            float lfo2ModulatedFreq = bendFreq * lfo2Modulation;
            uint32_t phaseInc = (uint32_t)((lfo2ModulatedFreq * 4294967296.0f) / SAMPLE_RATE);
            siren.discoPhase += phaseInc;
            
            // Square + saw mix for brass character
            int16_t square = generateSquare(siren.discoPhase);
            int16_t saw = generateSawtooth(siren.discoPhase);
            sample = (square + saw) / 2;
            
            // Midrange emphasis filter
            static int32_t funkFilterState = 0;
            int32_t cutoff = 800 + (int32_t)(siren.discoBrightness * 1200); // 800-2000Hz
            funkFilterState = funkFilterState + (((sample - funkFilterState) * cutoff) >> 12);
            sample = (int16_t)funkFilterState;
            
            sample = (int16_t)(sample * envelope * 2.0f); // 2x volume boost
            break;
        }
        
        case DISCO_SPACE_WHOOSH:
        {
            // Rising/falling filtered noise sweep (spaceship flyby)
            float sweepTime = 0.5f + (siren.discoCharacter * 1.5f); // 0.5-2.0s sweep (Pot 3)
            
            if (elapsedSec < sweepTime) {
                envelope = 1.0f - (elapsedSec / sweepTime); // Fade out
            } else {
                envelope = 0.0f;
            }
            
            // White noise generator
            siren.discoNoiseState = siren.discoNoiseState * 1664525 + 1013904223;
            sample = (siren.discoNoiseState >> 16) - 16384;
            
            // Pitch sweep (up or down based on brightness)
            float sweepProgress = elapsedSec / sweepTime;
            float sweepFreq;
            if (siren.discoBrightness > 0.5f) {
                // Sweep UP
                sweepFreq = 200.0f + (sweepProgress * 3800.0f); // 200-4000Hz
            } else {
                // Sweep DOWN
                sweepFreq = 4000.0f - (sweepProgress * 3800.0f); // 4000-200Hz
            }
            
            // Resonant bandpass filter
            static int32_t whooshFilterState1 = 0;
            static int32_t whooshFilterState2 = 0;
            
            // Convert frequency to filter coefficient (0-4095 range)
            int32_t cutoff = (int32_t)((sweepFreq / 4000.0f) * 4095.0f);
            if (cutoff > 4095) cutoff = 4095;
            if (cutoff < 100) cutoff = 100; // Minimum cutoff
            
            // Two-stage filter for resonance
            whooshFilterState1 = whooshFilterState1 + (((sample - whooshFilterState1) * cutoff) >> 10);
            whooshFilterState2 = whooshFilterState2 + (((whooshFilterState1 - whooshFilterState2) * cutoff) >> 10);
            sample = (int16_t)whooshFilterState2;
            
            sample = (int16_t)(sample * envelope * 2.0f); // 2x volume boost
            break;
        }
        
        case DISCO_BUBBLE_POP:
        {
            // Bubble pop sound - short percussive "bloop" with snap
            float popTime = 0.04f + (siren.discoCharacter * 0.12f); // 40-160ms (Pot 2)
            
            // Sharp attack, exponential decay (like a real bubble bursting)
            if (elapsedSec < 0.002f) {
                envelope = elapsedSec / 0.002f; // 2ms attack (very fast)
            } else if (elapsedSec < popTime) {
                float decayProgress = (elapsedSec - 0.002f) / (popTime - 0.002f);
                envelope = pow(1.0f - decayProgress, 2.5f); // Exponential decay
            } else {
                envelope = 0.0f;
            }
            
            // Characteristic bubble "bloop" pitch - starts high, drops fast, slight bounce
            float pitchCurve;
            if (elapsedSec < 0.015f) {
                // Initial fast drop (the "pop" part)
                pitchCurve = 1.0f - (elapsedSec / 0.015f) * 0.7f; // Drop 70%
            } else {
                // Settle with tiny bounce
                float bounceTime = (elapsedSec - 0.015f) / (popTime - 0.015f);
                pitchCurve = 0.3f + sin(bounceTime * 20.0f) * 0.05f * (1.0f - bounceTime); // Small wobble
            }
            
            float bubbleFreq = currentFreq * (2.5f + pitchCurve * 2.0f); // 2.5x to 4.5x base freq
            // Constrain to audible range (prevent ultra-high squeaks)
            if (bubbleFreq > 3000.0f) bubbleFreq = 3000.0f;
            if (bubbleFreq < 200.0f) bubbleFreq = 200.0f;
            
            uint32_t phaseInc = (uint32_t)((bubbleFreq * 4294967296.0f) / SAMPLE_RATE);
            siren.discoPhase += phaseInc;
            
            // Pure sine wave for clean bubble tone
            sample = fastSin(siren.discoPhase);
            
            // Add noise burst at the very start (the "pop" snap)
            if (elapsedSec < 0.01f) {
                siren.discoNoiseState = siren.discoNoiseState * 1664525 + 1013904223;
                int16_t noise = (siren.discoNoiseState >> 16) - 16384;
                float noiseMix = (1.0f - elapsedSec / 0.01f) * 0.25f; // 25% noise at start
                sample = (int16_t)(sample * (1.0f - noiseMix) + noise * noiseMix);
            }
            
            // Resonant high-pass filter for "hollow" character
            static int32_t bubbleFilterState = 0;
            int32_t cutoff = 1500 + (int32_t)(siren.discoBrightness * 2500); // 1500-4000Hz (Pot 3)
            bubbleFilterState = bubbleFilterState + (((sample - bubbleFilterState) * cutoff) >> 11);
            sample = sample - (int16_t)bubbleFilterState; // High-pass = input - lowpass
            
            // Apply sharp envelope
            sample = (int16_t)(sample * envelope * 2.5f); // 2.5x volume boost
            break;
        }
        
        case DISCO_LASER_SWEEP:  // Now PWM PAD
        {
            // Thick PWM synth pad - completely different from strings
            // Think vintage analog synth pads with pulse width modulation
            
            // Medium attack, infinite sustain while held
            float attackTime = 0.4f + (siren.discoCharacter * 1.1f); // 0.4-1.5s attack (Pot 2)
            
            if (elapsedSec < attackTime) {
                envelope = elapsedSec / attackTime; // Fade in
            } else if (siren.triggered) {
                envelope = 1.0f; // Full sustain while held
            } else {
                // Quick release when trigger released
                float releaseTime = 0.3f; // 300ms release (faster than strings)
                float releaseElapsed = elapsedSec - attackTime;
                if (releaseElapsed < releaseTime) {
                    envelope = 1.0f - (releaseElapsed / releaseTime);
                } else {
                    envelope = 0.0f;
                }
            }
            
            // Generate PWM (Pulse Width Modulation) effect
            // This creates the classic analog synth pad sound
            uint32_t phaseInc = (uint32_t)((currentFreq * 4294967296.0f) / SAMPLE_RATE);
            siren.discoPhase += phaseInc;
            
            // Slowly modulating pulse width (10% to 90%)
            float pwmRate = 0.5f + (siren.discoCharacter * 2.0f); // Pot 2 controls PWM speed
            float pulseWidth = 0.5f + (sin(elapsedSec * pwmRate * 6.28318f) * 0.4f); // 10-90%
            
            // Generate PWM wave (variable duty cycle square wave)
            uint32_t phaseNormalized = siren.discoPhase >> 24; // 0-255
            int16_t pwm1 = (phaseNormalized < (uint32_t)(pulseWidth * 255)) ? 16384 : -16384;
            
            // Add slightly detuned second PWM voice for thickness
            uint32_t phase2 = (uint32_t)(siren.discoPhase * 1.005f); // +0.5% detune
            uint32_t phase2Normalized = phase2 >> 24;
            float pulseWidth2 = 0.5f - (sin(elapsedSec * pwmRate * 6.28318f) * 0.4f); // Inverted modulation
            int16_t pwm2 = (phase2Normalized < (uint32_t)(pulseWidth2 * 255)) ? 16384 : -16384;
            
            // Add sub-octave for depth (like vintage synths)
            uint32_t phaseSub = siren.discoPhase >> 1; // One octave down
            int16_t subWave = generateSquare(phaseSub);
            
            // Mix: main PWM voices + sub
            sample = (pwm1 + pwm2) / 2 + (subWave >> 2);
            
            // Static filter position (controlled by Pot 3 brightness)
            // Unlike strings which sweep, this stays put for classic pad sound
            static int32_t pwmFilterState = 0;
            int32_t cutoff = 600 + (int32_t)(siren.discoBrightness * 2400); // 600-3000Hz (Pot 3)
            pwmFilterState = pwmFilterState + (((sample - pwmFilterState) * cutoff) >> 12);
            sample = (int16_t)pwmFilterState;
            
            sample = (int16_t)(sample * envelope * 1.5f); // Moderate volume
            break;
        }
    }  
    
    // Store for waveform display
    lastSample = sample;
    
    // Apply gate (only sound when triggered OR envelope still active)
    // Disco effects should play out their full envelope even after trigger released
    if (!siren.triggered && envelope <= 0.001f) {
        sample = 0;
        siren.discoPhase = 0;
        siren.discoEnvelope = 0.0f;
    }
    
    return sample;
}

// =====================================================================
// DRONE SYNTH SOUND GENERATION
// =====================================================================

int16_t generateDroneSample() {
    // Check if we should be making sound
    bool shouldMakeSound = false;
    
    if (drone.playMode == 2) {
        // ON mode - always make sound
        shouldMakeSound = true;
    } else if (drone.playMode == 1) {
        // KEYS mode - only make sound when keys pressed
        shouldMakeSound = (keyboard.numKeysHeld > 0);
    }
    // playMode == 0 (OFF) - shouldMakeSound stays false
    
    // Fade in/out envelope
    const float fadeTime = 0.5f; // 500ms fade
    unsigned long elapsed = millis() - drone.fadeStartTime;
    float targetEnvelope = shouldMakeSound ? 1.0f : 0.0f;
    
    // Smooth envelope transition
    if (elapsed < (fadeTime * 1000)) {
        float progress = elapsed / (fadeTime * 1000.0f);
        if (shouldMakeSound) {
            drone.envelope = progress; // Fade in
        } else {
            drone.envelope = 1.0f - progress; // Fade out
        }
    } else {
        drone.envelope = targetEnvelope;
    }
    
    // If completely silent, return early
    if (drone.envelope < 0.001f) {
        return 0;
    }
    
    // Generate oscillators based on chord mode
    int32_t mixedSample = 0;
    int numOscillators = 5;
    
    // Define frequency ratios for each chord mode
    float freqRatios[5];
    
    switch (drone.chordMode) {
        case DRONE_UNISON:
            // All oscillators at root with slight detune
            freqRatios[0] = 1.0f;
            freqRatios[1] = 1.0f;
            freqRatios[2] = 1.0f;
            freqRatios[3] = 1.0f;
            freqRatios[4] = 1.0f;
            break;
            
        case DRONE_OCTAVE:
            // Root + octave spread
            freqRatios[0] = 0.5f;   // One octave down
            freqRatios[1] = 1.0f;   // Root
            freqRatios[2] = 1.0f;   // Root
            freqRatios[3] = 2.0f;   // One octave up
            freqRatios[4] = 2.0f;   // One octave up
            break;
            
        case DRONE_FIFTH:
            // Root + perfect fifth
            freqRatios[0] = 1.0f;     // Root
            freqRatios[1] = 1.0f;     // Root
            freqRatios[2] = 1.5f;     // Perfect fifth
            freqRatios[3] = 1.5f;     // Perfect fifth
            freqRatios[4] = 2.0f;     // Octave
            break;
            
        case DRONE_MAJOR:
            // Major chord (root, major third, fifth)
            freqRatios[0] = 1.0f;     // Root
            freqRatios[1] = 1.0f;     // Root
            freqRatios[2] = 1.25f;    // Major third
            freqRatios[3] = 1.5f;     // Perfect fifth
            freqRatios[4] = 2.0f;     // Octave
            break;
            
        case DRONE_MINOR:
            // Minor chord (root, minor third, fifth)
            freqRatios[0] = 1.0f;     // Root
            freqRatios[1] = 1.0f;     // Root
            freqRatios[2] = 1.189f;   // Minor third
            freqRatios[3] = 1.5f;     // Perfect fifth
            freqRatios[4] = 2.0f;     // Octave
            break;
            
        default:
            // Fallback to unison
            for (int i = 0; i < 5; i++) freqRatios[i] = 1.0f;
            break;
    }
    
    for (int i = 0; i < numOscillators; i++) {
        // Update drift LFO for this oscillator
        uint32_t driftPhaseInc = (uint32_t)((drone.driftSpeed * 4294967296.0f) / SAMPLE_RATE);
        drone.driftPhases[i] += driftPhaseInc;
        float driftLFO = fastSin(drone.driftPhases[i]) / 32768.0f; // -1.0 to +1.0
        
        // Calculate frequency with chord ratio, detune, and drift
        float detuneAmount = drone.oscDetunes[i] + (driftLFO * drone.detune * 0.02f);
        float oscFreq = drone.rootFreq * freqRatios[i] * (1.0f + detuneAmount);
        
        // Keep in reasonable range
        if (oscFreq < 20.0f) oscFreq = 20.0f;
        if (oscFreq > 4000.0f) oscFreq = 4000.0f;
        
        // Generate phase increment
        uint32_t phaseInc = (uint32_t)((oscFreq * 4294967296.0f) / SAMPLE_RATE);
        drone.oscPhases[i] += phaseInc;
        
        // Generate waveform based on brightness
        int16_t oscSample;
        
        if (drone.brightness < 0.33f) {
            // Dark: mostly triangle/sine
            oscSample = generateTriangle(drone.oscPhases[i]);
        } else if (drone.brightness < 0.67f) {
            // Medium: mix of saw and triangle
            int16_t saw = generateSawtooth(drone.oscPhases[i]);
            int16_t tri = generateTriangle(drone.oscPhases[i]);
            float mix = (drone.brightness - 0.33f) / 0.34f;
            oscSample = (int16_t)(tri * (1.0f - mix) + saw * mix);
        } else {
            // Bright: saw + square
            int16_t saw = generateSawtooth(drone.oscPhases[i]);
            int16_t square = generateSquare(drone.oscPhases[i]);
            float mix = (drone.brightness - 0.67f) / 0.33f;
            oscSample = (int16_t)((saw * (1.0f - mix * 0.5f)) + (square * mix * 0.5f));
        }
        
        mixedSample += oscSample;
    }
    
    // Average the oscillators
    int16_t finalSample = (int16_t)(mixedSample / 5);
    
    // Apply simple lowpass filter for smoothness
    static int32_t droneFilterState = 0;
    int32_t cutoff = 2000 + (int32_t)(drone.brightness * 2000); // 2000-4000Hz
    droneFilterState = droneFilterState + (((finalSample - droneFilterState) * cutoff) >> 12);
    finalSample = (int16_t)droneFilterState;
    
    // Apply envelope
    finalSample = (int16_t)(finalSample * drone.envelope);
    
    // Store for waveform display
    lastSample = finalSample;
    
    return finalSample;
}

// =====================================================================
// MAIN SAMPLE GENERATION (DUB SIREN)
// =====================================================================
int16_t generateSample() {
    int16_t filteredSample = 0;
    
    // ========== CALCULATE GLOBAL LFO 2 MODULATION FACTOR ==========
    // This will be applied INSIDE each instrument's synthesis
    float lfo2Modulation = 1.0f;  // Default: no modulation
    if (lfo2.enabled) {
        lfo2.phase += (uint32_t)((lfo2.rate * 4294967296.0f) / SAMPLE_RATE);
        float lfo2Value = fastSin(lfo2.phase) / 32768.0f;  // -1.0 to +1.0
        // Much stronger modulation: ±60% range (up to 1.6x or down to 0.4x frequency)
        lfo2Modulation = 1.0f + (lfo2Value * lfo2.depth * 0.6f);
    }
    
// ========== RECORD MODE PLAYBACK ==========
    int16_t recordedSample = 0;
    bool playingRecording = false;
    
    if (recorder.playing) {
        playingRecording = true;
        if (recorder.recordPosition < recorder.recordLength) {
            recordedSample = recorder.buffer[recorder.recordPosition];
            recorder.recordPosition++;
        } else {
            // Reached end of recording
            if (recorder.loopEnabled) {
                // Loop back to start
                recorder.recordPosition = 0;
                recordedSample = recorder.buffer[recorder.recordPosition];
                recorder.recordPosition++;
                Serial.println("LOOP");
            } else {
                // Playback finished
                recorder.playing = false;
                recorder.recordPosition = 0;
                recordedSample = 0;
                playingRecording = false;
                Serial.println("PLAYBACK FINISHED");
            }
        }
    }
    
if (sequencer.active && sequencer.running) {
    // Note: Step advancement is now handled in loop1() by sync or internal timer
    // This just generates audio for the current step
    
    // Get current time for envelope calculation
    unsigned long currentTime = millis();
    
    // === CHECK GATE LENGTH ===
    unsigned long noteElapsed = currentTime - sequencer.noteStartTime;
    unsigned long noteDuration = (sequencer.stepInterval * sequencer.gateLength) / 100;
    if (noteElapsed >= noteDuration) {
        sequencer.noteActive = false;
    }
    
    // Generate audio for current step - ONLY if step is enabled AND within gate time
if (sequencer.stepEnabled[sequencer.currentStep] && sequencer.noteActive) {
    // Get frequency for this step WITH octave shift
    // LOCK frequency to prevent drift - store and reuse exact value
    static float lockedFreq = 0;
    static uint8_t lastLockedStep = 255;
    static int8_t lastLockedOctave = 0;  // ADD THIS - track octave changes
    
    // Recalculate frequency when step changes OR octave changes
    if (lastLockedStep != sequencer.currentStep || lastLockedOctave != sequencer.octaveShift) {
        float noteFreq = sequencer.stepFrequencies[sequencer.currentStep];
        
        // Apply octave shift (multiply/divide by 2 for each octave)
        if (sequencer.octaveShift > 0) {
            noteFreq *= (1 << sequencer.octaveShift); // Shift up (multiply by 2^n)
        } else if (sequencer.octaveShift < 0) {
            noteFreq /= (1 << (-sequencer.octaveShift)); // Shift down (divide by 2^n)
        }
        
        // Constrain to reasonable audio range
        if (noteFreq < 20.0f) noteFreq = 20.0f;
        if (noteFreq > 4000.0f) noteFreq = 4000.0f;
        
        // LOCK this frequency
        lockedFreq = noteFreq;
        lastLockedStep = sequencer.currentStep;
        lastLockedOctave = sequencer.octaveShift;  // ADD THIS - remember octave
    }
    
    // Use the LOCKED frequency (no recalculation = perfect tuning stability)
    float noteFreq = lockedFreq * lfo2Modulation;  // Apply LFO2
        
        if (noteFreq > 0 && sequencer.noteActive) {
            uint32_t phaseInc = (uint32_t)((noteFreq * 4294967296.0f) / SAMPLE_RATE);
            sequencer.sequencerPhase += phaseInc;
            
            int16_t rawSample = 0;
            
            // Generate waveform based on sound mode
            switch (sequencer.soundMode) {
                case SEQ_SOUND_SAW:
                    // Mix saw and square (current sound)
                    {
                        int16_t saw = generateSawtooth(sequencer.sequencerPhase);
                        int16_t square = generateSquare(sequencer.sequencerPhase);
                        rawSample = (saw >> 1) + (square >> 2);
                    }
                    break;
                    
                case SEQ_SOUND_PULSE:
                    // Pulse width modulation - varies per step
                    {
                        uint32_t pwm = sequencer.sequencerPhase + (sequencer.currentStep * 536870912);
                        rawSample = generateSquare(pwm);
                    }
                    break;
                    
                case SEQ_SOUND_FILTERED:
                    // Heavy filtered sawtooth for dark bass
                    {
                        rawSample = generateSawtooth(sequencer.sequencerPhase);
                        // Extra filtering happens below
                    }
                    break;
                    
                case SEQ_SOUND_PLUCK:
                    // Plucked sound with decay envelope
                    {
                        int16_t saw = generateSawtooth(sequencer.sequencerPhase);
                        float decay = 1.0f - (noteElapsed / (float)noteDuration);
                        if (decay < 0.0f) decay = 0.0f;
                        rawSample = (int16_t)(saw * decay);
                    }
                    break;
                    
                case SEQ_SOUND_CHORD:
    // Octave + fifth for power chord sound
    {
        int16_t fundamental = generateSquare(sequencer.sequencerPhase);
        int16_t octave = generateSquare(sequencer.sequencerPhase * 2);      // Octave up
        int16_t fifth = generateSquare(sequencer.sequencerPhase * 3 / 2);   // Perfect fifth
        // Mix evenly for thick power chord
        rawSample = (fundamental + octave + fifth) / 3;
    }
    break;
            }
            
            // === APPLY FILTER (controlled by Pot 3) ===
            static int32_t seqFilterState = 0;
            int32_t filterAmount = sequencer.filterCutoff; // 200-4000
            
            // Extra filtering for FILTERED mode
            if (sequencer.soundMode == SEQ_SOUND_FILTERED) {
                filterAmount = filterAmount >> 2; // Quarter the cutoff for darker sound
            }
            
            seqFilterState = seqFilterState + (((rawSample - seqFilterState) * filterAmount) >> 12);
            filteredSample = (int16_t)seqFilterState;
            
            // Simple envelope
            unsigned long stepElapsed = currentTime - sequencer.lastStepTime;
            float envelope = 1.0f;
            if (stepElapsed < 50) {
                envelope = stepElapsed / 50.0f;
            } else if (stepElapsed > sequencer.stepInterval - 50) {
                envelope = (sequencer.stepInterval - stepElapsed) / 50.0f;
            }
            
            filteredSample = (int16_t)(filteredSample * envelope);
        } else {
            filteredSample = 0;
            sequencer.sequencerPhase = 0;
        }
    } else {
        // Step is disabled - output silence and reset phase
        filteredSample = 0;
        sequencer.sequencerPhase = 0;
    }
    
    lastSample = filteredSample;
}  // <--- ADD THIS CLOSING BRACE for the main "if (sequencer.active && sequencer.running)" block

    else if (siren.instrumentType == INST_RAY_GUN) {
        // RAY GUN: Use dedicated generator
        filteredSample = generateRayGunSample();
        
    } else if (siren.instrumentType == INST_LEAD_SYNTH) {
        // LEAD SYNTH: Use dedicated generator
        filteredSample = generateLeadSynthSample();
        
    } else if (siren.instrumentType == INST_DISCO) {
        // DISCO: Use dedicated generator
        filteredSample = generateDiscoSample();
        
        // CRITICAL: Store for display AFTER generation
        lastSample = filteredSample;
        
    } else {
        // DUB SIREN: Full synthesis chain
        
        // Check if a keyboard key is pressed - if so, disable LFO modulation
        bool keyboardActive = keyboard.keyPressed || keyboard.numKeysHeld > 0;
        
        // LFO modulation (only when keyboard not active)
        float lfo = 0.0f;
        if (!keyboardActive) {
            lfoPhase += (uint32_t)((siren.lfoRate * 4294967296.0f) / SAMPLE_RATE);
            lfo = fastSin(lfoPhase) / 32768.0f;
        } else {
            // Reset LFO phase when keyboard is active for clean restart
            lfoPhase = 0;
        }
        
        // LFO 2 modulation (always active when enabled)
        float lfo2Value = 0.0f;
        if (lfo2.enabled) {
            lfo2.phase += (uint32_t)((lfo2.rate * 4294967296.0f) / SAMPLE_RATE);
            lfo2Value = fastSin(lfo2.phase) / 32768.0f;
        }
        
        // Handle different modes
        float modFreq = siren.frequency;
        
        switch (siren.mode) {
            case MODE_CLASSIC_DUB:
            case MODE_SQUARE_WAVE:
            {
                // ADDITIVE modulation for smooth, even pitch sweeps (classic dub siren)
                if (!keyboardActive) {
                    float modulationHz = siren.frequency * siren.lfoDepth * 0.5f;
                    modFreq = siren.frequency + (lfo * modulationHz);
                    if (modFreq < 20.0f) modFreq = 20.0f;
                } else {
                    modFreq = siren.frequency;
                }
                
                // Add LFO 2 modulation (works with or without keyboard)
                if (lfo2.enabled) {
                    float lfo2ModHz = modFreq * lfo2.depth * 0.3f;
                    modFreq = modFreq + (lfo2Value * lfo2ModHz);
                    if (modFreq < 20.0f) modFreq = 20.0f;
                }
                break;
            }
                
            case MODE_DEEP_SUB:
                if (!keyboardActive) {
                    modFreq = siren.frequency + (lfo * siren.frequency * 0.15f);
                    if (modFreq < 20.0f) modFreq = 20.0f;
                } else {
                    modFreq = siren.frequency;
                }
                break;
                
            case MODE_RING_MOD:
                if (!keyboardActive) {
                    modFreq = siren.frequency + (lfo * siren.frequency * siren.lfoDepth * 0.5f);
                    if (modFreq < 20.0f) modFreq = 20.0f;
                } else {
                    modFreq = siren.frequency;
                }
                break;
                
            case MODE_PORTAMENTO:
                siren.frequency = applyPortamento(siren.frequency, siren.baseFreq, siren.glideTime);
                modFreq = siren.frequency;
                break;
                
            case MODE_LOFI_CRUSH:
                if (!keyboardActive) {
                    modFreq = siren.frequency + (lfo * siren.frequency * siren.lfoDepth * 0.5f);
                    if (modFreq < 20.0f) modFreq = 20.0f;
                } else {
                    modFreq = siren.frequency;
                }
                break;
        }
        
        // Generate phase increment with LFO2 modulation
        float lfo2ModulatedFreq = modFreq * lfo2Modulation;
        uint32_t phaseInc = (uint32_t)((lfo2ModulatedFreq * 4294967296.0f) / SAMPLE_RATE);
        phase += phaseInc;
        
        // Generate base waveform based on mode
        int16_t sample = 0;
        
        switch (siren.mode) {
            case MODE_CLASSIC_DUB:
            case MODE_DEEP_SUB:
            case MODE_LOFI_CRUSH:
            case MODE_PORTAMENTO:
                // Use mostly triangle with just a touch of sawtooth for warmth
                {
                    int16_t saw = generateSawtooth(phase);
                    int16_t tri = generateTriangle(phase);
                    sample = (tri * 3 + saw) >> 2; // 75% triangle, 25% sawtooth
                }
                break;
                
            case MODE_SQUARE_WAVE:
                sample = generateSquare(phase);
                break;
                
            case MODE_RING_MOD:
                // Ring modulation - multiply carrier by modulator
                {
                    int16_t carrier = generateSawtooth(phase);
                    
                    // Generate modulator at ringModFreq
                    static uint32_t ringPhase = 0;
                    uint32_t ringPhaseInc = (uint32_t)((siren.ringModFreq * 4294967296.0f) / SAMPLE_RATE);
                    ringPhase += ringPhaseInc;
                    int16_t modulator = generateSawtooth(ringPhase);
                    
                    // Ring mod = carrier * modulator
                    sample = (int16_t)(((int32_t)carrier * (int32_t)modulator) >> 15);
                }
                break;
        }
        
        // Apply mode-specific processing
        if (siren.mode == MODE_DEEP_SUB) {
            // Add sub-octave
            uint32_t subPhase = phase >> 1; // One octave down
            int16_t subSample = generateSawtooth(subPhase);
            sample = (int16_t)(sample * (1.0f - siren.subMix) + subSample * siren.subMix);
        }
        
        if (siren.mode == MODE_LOFI_CRUSH) {
            // Apply bit crushing
            sample = applyBitCrusher(sample, siren.bitDepth);
        }
        
        // Simple lowpass filter
        int32_t filterAmount = siren.filterCutoff;
        filterState = filterState + (((sample - filterState) * filterAmount) >> 12);
        filteredSample = (int16_t)filterState;

        // Store for waveform display BEFORE gating (so we see preview)
        lastSample = filteredSample;

        // Apply gate (only sound when triggered)
        if (!siren.triggered) {
            filteredSample = 0; // Completely silent when not triggered
            phase = 0; // Reset phase for clean start
            lfoPhase = 0;
        }
    }

// ========== APPLY DELAY/ECHO (COMMON TO BOTH INSTRUMENTS) ==========
    
    int16_t delayedSample = 0;
    
    // Apply echo/delay effect when delay is enabled
bool allowEcho = (siren.delayIndex > 0);
    
    // CRITICAL FIX: When echo is OFF, we must still pass audio through!
if (allowEcho) {  // Only apply echo if delay is on

        // Read from delay buffer
        uint16_t readPos = (echoWritePos + MAX_DELAY_SAMPLES - siren.echoTime) % MAX_DELAY_SAMPLES;
        delayedSample = echoBuffer[readPos];
        
        // APPLY PITCH SHIFT AND MIX if reverse mode active
        if (reverser.active && reverser.reversing) {
            // Store original delayed sample (dry signal)
            int16_t drySignal = delayedSample;
            
            // Apply pitch shift based on pitchShift setting
            static uint16_t pitchCounter = 0;
            static int16_t lastPitchSample = 0;
            int16_t wetSignal = delayedSample;
            
            if (reverser.pitchShift == -2) {
                // -2 octaves: read every 4th sample (quarter speed)
                if (pitchCounter % 4 == 0) {
                    lastPitchSample = delayedSample;
                }
                wetSignal = lastPitchSample;
                pitchCounter++;
            } else if (reverser.pitchShift == -1) {
                // -1 octave: read every 2nd sample (half speed) - ORIGINAL BEHAVIOR
                if (pitchCounter % 2 == 0) {
                    lastPitchSample = delayedSample;
                }
                wetSignal = lastPitchSample;
                pitchCounter++;
            } else if (reverser.pitchShift == 0) {
                // No pitch shift - true reverse at original pitch
                wetSignal = delayedSample;
            } else if (reverser.pitchShift == 1) {
                // +1 octave: read every 2nd sample but advance twice (double speed)
                pitchCounter += 2;
                wetSignal = delayedSample;
            } else if (reverser.pitchShift == 2) {
                // +2 octaves: read every 4th sample but advance 4x (quadruple speed)
                pitchCounter += 4;
                wetSignal = delayedSample;
            }
            
            // Mix dry and wet based on mixAmount
            // 0.0 = all dry (no reverse), 1.0 = all wet (full reverse)
            delayedSample = (int16_t)(
                (drySignal * (1.0f - reverser.mixAmount)) + 
                (wetSignal * reverser.mixAmount)
            );
        }
        
// Write to delay buffer with feedback
        int32_t feedbackSample;
        if (abs(filteredSample) < 50) { // Noise gate threshold
            feedbackSample = (delayedSample * siren.echoFeedback);
        } else {
            feedbackSample = filteredSample + (delayedSample * siren.echoFeedback);
        }
        if (feedbackSample > 32767) feedbackSample = 32767;
        if (feedbackSample < -32768) feedbackSample = -32768;
        echoBuffer[echoWritePos] = (int16_t)feedbackSample;
        
        echoWritePos = (echoWritePos + 1) % MAX_DELAY_SAMPLES;
        
        // Mix dry and wet (50/50 mix for classic dub sound)
        filteredSample = (filteredSample >> 1) + (delayedSample >> 1);
    } else {
        // When delay is OFF, clear the buffer and output dry signal only
        echoBuffer[echoWritePos] = 0;
        echoWritePos = (echoWritePos + 1) % MAX_DELAY_SAMPLES;
        // filteredSample stays as is (dry signal only)
    }
    
    // ========== MIX RECORDED AUDIO WITH LIVE SYNTH ==========
    if (playingRecording) {
        // Mix recorded sample with live synth (50/50 mix)
        filteredSample = (filteredSample >> 1) + (recordedSample >> 1);
        
        // Store mixed result for waveform display
        lastSample = filteredSample;
    }

// ========== MIX DRONE AUDIO IF ACTIVE ==========
    // CRITICAL: Mix drone BEFORE recording so it gets captured
    if (drone.active) {
        int16_t droneSample = generateDroneSample();
        
        // Debug output
        static unsigned long lastDroneDebug = 0;
        if (millis() - lastDroneDebug > 1000) {
            Serial.print("DRONE: mode=");
            if (drone.playMode == 0) Serial.print("OFF");
            else if (drone.playMode == 1) Serial.print("KEYS");
            else Serial.print("ON");
            Serial.print(" envelope=");
            Serial.print(drone.envelope);
            Serial.print(" sample=");
            Serial.println(droneSample);
            lastDroneDebug = millis();
        }
        
        // Mix drone with main audio (50/50 mix when both active)
        if (abs(droneSample) > 10) {
            if (abs(filteredSample) > 10) {
                // Both active - mix 50/50
                filteredSample = (filteredSample >> 1) + (droneSample >> 1);
            } else {
                // Only drone active - use full volume
                filteredSample = droneSample;
            }
        }
    }

    // ========== RECORD AUDIO IF RECORDING IS ACTIVE ==========
    if (recorder.recording && recorder.recordPosition < MAX_RECORD_SAMPLES) {
        recorder.buffer[recorder.recordPosition] = filteredSample;
        recorder.recordPosition++;
        
        if (recorder.recordPosition >= MAX_RECORD_SAMPLES) {
            // Buffer full - auto-stop
            recorder.recording = false;
            recorder.hasRecording = true;
            recorder.recordLength = recorder.recordPosition;
            Serial.println("RECORDING STOPPED - Buffer full!");
        }
    }

// Store final sample for waveform display (AFTER all effects)
    lastSample = filteredSample;
    
    // ========== APPLY SIDECHAIN DUCKING (FINAL STAGE) ==========
    if (sidechain.enabled) {
        // Apply envelope to create ducking effect
        int16_t originalSample = filteredSample;
        filteredSample = (int16_t)(filteredSample * sidechain.envelope);
        
        // ALWAYS show when envelope is active (every 5000 samples)
        static int debugCounter = 0;
        if (++debugCounter % 5000 == 0) {
            Serial.print("SC: orig=");
            Serial.print(originalSample);
            Serial.print(" ducked=");
            Serial.print(filteredSample);
            Serial.print(" env=");
            Serial.print(sidechain.envelope, 3);
            Serial.print(" enabled=");
            Serial.println(sidechain.enabled);
        }
    }
    
    // Silence output during record mode key presses
    if (recordKeySilence) {
        return 0;  // Silent output
    }
    
    return filteredSample;
}


// =====================================================================
// MUTATION FUNCTIONS
// =====================================================================

// Quantize frequency to musical scale
float quantizePitchToScale(float baseFreq, uint8_t scaleStep) {
    // Use proper equal temperament tuning from A440
    // A4 = 440Hz is our reference
    
    // Select scale based on current setting
    const float* scale;
    uint8_t scaleSize;
    
    switch (mutate.currentScale) {
        case SCALE_MINOR_PENT:
            scale = minorPentatonic;
            scaleSize = minorPentatonicSize;
            break;
        case SCALE_MAJOR:
            scale = majorScale;
            scaleSize = majorScaleSize;
            break;
        case SCALE_BLUES:
            scale = bluesScale;
            scaleSize = bluesScaleSize;
            break;
        case SCALE_MINOR:
            scale = minorScale;
            scaleSize = minorScaleSize;
            break;
        case SCALE_CHROMATIC:
            scale = chromaticScale;
            scaleSize = chromaticScaleSize;
            break;
        default:
            scale = minorPentatonic;
            scaleSize = minorPentatonicSize;
            break;
    }
    
    // Calculate which octave and note within scale
    uint8_t octave = scaleStep / scaleSize;
    uint8_t noteInScale = scaleStep % scaleSize;
    
    // Start from a musical root (110Hz = A2, two octaves below A440)
    float rootFreq = 110.0f;
    
    // Apply octave shift
    float octaveFreq = rootFreq * (1 << octave); // Multiply by 2^octave
    
    // Apply scale interval
    return octaveFreq * scale[noteInScale];
}

// Random walk through scale steps (stays musical)
void mutatePitch() {
    // Random walk: ±1 or ±2 steps, occasionally jump ±octave
    int change;
    
    // Get current scale size
    uint8_t scaleSize;
    switch (mutate.currentScale) {
        case SCALE_MINOR_PENT: scaleSize = minorPentatonicSize; break;
        case SCALE_MAJOR: scaleSize = majorScaleSize; break;
        case SCALE_BLUES: scaleSize = bluesScaleSize; break;
        case SCALE_MINOR: scaleSize = minorScaleSize; break;
        case SCALE_CHROMATIC: scaleSize = chromaticScaleSize; break;
        default: scaleSize = minorPentatonicSize; break;
    }
    
    uint8_t randomValue = random(100);
    
    if (randomValue < 10) {
        // 10% chance: octave jump
        change = (random(2) == 0) ? scaleSize : -scaleSize;
    } else if (randomValue < 40) {
        // 30% chance: ±2 steps
        change = (random(2) == 0) ? 2 : -2;
    } else {
        // 60% chance: ±1 step
        change = (random(2) == 0) ? 1 : -1;
    }
    
    // Apply change with wrapping
    int newStep = (int)mutate.pitchStep + change;
    
    // Keep in range (0 to scaleSize*4 for 4 octaves)
    if (newStep < 0) newStep = 0;
    if (newStep >= scaleSize * 4) newStep = scaleSize * 4 - 1;
    
    mutate.pitchStep = newStep;
    
// Calculate actual frequency
    uint8_t octaveOffset = mutate.pitchStep / scaleSize;
    uint8_t scaleNote = mutate.pitchStep % scaleSize;
    
    float newFreq = quantizePitchToScale(100.0f * (1 << octaveOffset), scaleNote);
    
    // Constrain to instrument-specific ranges
    if (siren.instrumentType == INST_DUB_SIREN) {
        if (newFreq < 50.0f) newFreq = 50.0f;
        if (newFreq > 2000.0f) newFreq = 2000.0f;
    } else if (siren.instrumentType == INST_RAY_GUN) {
        if (newFreq < 200.0f) newFreq = 200.0f;
        if (newFreq > 4000.0f) newFreq = 4000.0f;
    } else { // Lead Synth
        if (newFreq < 200.0f) newFreq = 200.0f;
        if (newFreq > 800.0f) newFreq = 800.0f;
        
        // For Lead Synth, also update the sequence with new notes
        generateMelody();
    }
    
    // Apply octave shift (multiply/divide by 2 for each octave)
    if (octaveShift > 0) {
        newFreq *= (1 << octaveShift); // Shift up
    } else if (octaveShift < 0) {
        newFreq /= (1 << (-octaveShift)); // Shift down
    }
    
    siren.baseFreq = newFreq;
    siren.frequency = newFreq;
}

// Quantize speed to musical divisions
void mutateSpeed() {
    // Musical tempo divisions
    const float tempoSteps[] = {0.5f, 1.0f, 2.0f, 3.5f, 5.0f, 8.0f, 12.0f, 16.0f, 20.0f};
    const uint8_t tempoStepCount = 9;
    
    // Random walk through tempo steps
    int change = (random(3) - 1); // -1, 0, or 1
    
    int newStep = (int)mutate.speedStep + change;
    if (newStep < 0) newStep = 0;
    if (newStep >= tempoStepCount) newStep = tempoStepCount - 1;
    
    mutate.speedStep = newStep;
    
    // Apply to LFO rate or step interval depending on instrument
    if (siren.instrumentType == INST_LEAD_SYNTH) {
        // Map to step interval (60-480ms)
        siren.stepInterval = 60 + (uint16_t)((mutate.speedStep / (float)tempoStepCount) * 420);
    } else {
        // Map to LFO rate
        siren.lfoRate = tempoSteps[mutate.speedStep];
    }
}

// Mutate modulation parameter
void mutateModulation() {
    // Smooth random walk ±10-20%
    float change = (random(41) - 20) / 100.0f; // -0.20 to +0.20
    
    if (siren.instrumentType == INST_DUB_SIREN) {
        siren.lfoDepth += change;
        if (siren.lfoDepth < 0.0f) siren.lfoDepth = 0.0f;
        if (siren.lfoDepth > 1.0f) siren.lfoDepth = 1.0f;
    } else if (siren.instrumentType == INST_RAY_GUN) {
        siren.resonance += change * 0.5f; // Smaller changes for resonance
        if (siren.resonance < 0.3f) siren.resonance = 0.3f;
        if (siren.resonance > 0.95f) siren.resonance = 0.95f;
    } 
    // Lead Synth: Do NOT mutate vibrato - user controls it manually via pot
}

// Mutate feedback/echo
void mutateFeedback() {
    // Sweet spots for echo feedback
    const float feedbackPresets[] = {0.0f, 0.3f, 0.5f, 0.7f, 0.9f};
    const uint8_t presetCount = 5;
    
    // Pick a random preset
    uint8_t preset = random(presetCount);
    siren.echoFeedback = feedbackPresets[preset];
}

// Main mutation trigger - called on sync input or loop gates
void triggerMutation() {
    // If mutations are fully disabled, don't execute
    if (mutate.fullyDisabled) return;
    
    // Don't mutate too frequently (min 100ms between mutations)
    unsigned long now = millis();
    if (now - mutate.lastMutateTime < 100) return;
    
    mutate.lastMutateTime = now;
    
    // Store the last active mutation mode (not OFF)
    static uint8_t lastActiveMutateMode = MUTATE_AMOUNT;
    
    // Remember last active mode when not OFF
    if (mutate.mode != MUTATE_OFF) {
        lastActiveMutateMode = mutate.mode;
    }
    
    // Always mutate based on last active mode (even when display shows OFF)
    // This allows Pot 1 to return to pitch control while mutations continue
    switch (lastActiveMutateMode) {
        case MUTATE_AMOUNT:
            // Mutate pitch for all instruments
            mutatePitch();
            break;
            
        case MUTATE_RHYTHM:
            // Mutate pitch AND speed/rhythm
            mutatePitch();
            mutateSpeed();
            break;
            
        case MUTATE_CUTOFF:
            // Mutate pitch, speed, AND modulation
            mutatePitch();
            mutateSpeed();
            mutateModulation();
            break;
    }
}

// Evaluate if gate should be active based on loop pattern
bool evaluateLoopPattern() {
    if (!loopState.active || loopState.pattern == LOOP_OFF) {
        return true; // No pattern - gate always on when triggered
    }
    
    unsigned long elapsed = millis() - loopState.patternStartTime;
    unsigned long cycleTime = loopState.patternTempo;
    
    // Normalize position in cycle (0.0 to 1.0)
    float position = (float)(elapsed % cycleTime) / (float)cycleTime;
    
    switch (loopState.pattern) {
        case LOOP_TRIPLET:
            // ███░░░███░░░ - triplet feel (3 hits per 2 beats)
            // 2 cycles of 3 subdivisions each = 6 total subdivisions
            {
                float subPosition = fmod(position * 6.0f, 1.0f);
                // On for first half of each triplet subdivision
                return (subPosition < 0.5f);
            }
            
        case LOOP_SWING:
            // ██░██░░██░ - swing/shuffle (60/40 duty cycle, 3 hits per cycle)
            // Classic dub/reggae bounce
            {
                float subPosition = fmod(position * 3.0f, 1.0f);
                return (subPosition < 0.6f); // 60% on, 40% off
            }
            
        case LOOP_STACCATO:
            // █░░█░░█░░ - short stabs (25% on, 75% off, 4 times per cycle)
            {
                float subPosition = fmod(position * 4.0f, 1.0f);
                return (subPosition < 0.25f);
            }
            
        case LOOP_GATE:
            // ████░░░░ - gate on first half, silent second half
            // Perfect for call-and-response or space
            return (position < 0.5f);
            
        case LOOP_RATCHET:
            // █░█░█░█░█░█░ - fast machine gun (12 hits per cycle)
            {
                float subPosition = fmod(position * 12.0f, 1.0f);
                return (subPosition < 0.5f);
            }
            
        case LOOP_DOTTED:
            // ██░█░██░█░ - dotted eighth feel (4 hits: long, short, long, short)
            {
                float subPosition = fmod(position * 4.0f, 1.0f);
                // First and third hits are longer (60%), second and fourth are shorter (30%)
                int hitNum = (int)(position * 4.0f);
                if (hitNum % 2 == 0) {
                    return (subPosition < 0.6f); // Long hit
                } else {
                    return (subPosition < 0.3f); // Short hit
                }
            }
            
        default:
            return true;
    }
}

// Draw loop pattern visualization (static, no animation)
void drawLoopPatternIndicator() {
    // Track last drawn pattern
    static uint8_t lastDrawnPattern = 255;
    
    // Only redraw if pattern changed
    if (loopState.pattern == lastDrawnPattern) {
        return; // No change, don't redraw
    }
    
    // Position: right below "LOOP: [NAME]" text (moved down to accommodate RECORD)
    int startX = 5;
    int startY = 70;
    int barWidth = 120;
    int barHeight = 8;
    
    // Clear the indicator area
    display.fillRect(startX - 2, startY - 2, barWidth + 4, barHeight + 4, COLOR_BG);
    
    // If loop mode not active, don't draw anything
    if (!loopState.active) {
        lastDrawnPattern = loopState.pattern;
        return;
    }
    
    // Draw dark background
    display.fillRect(startX, startY, barWidth, barHeight, 0x2104); // Very dark gray
    
    // Draw pattern-specific visualization
    switch (loopState.pattern) {
        case LOOP_TRIPLET:
            // ███░░░███░░░ - triplet feel (6 fast divisions, alternating on/off)
            {
                int divWidth = barWidth / 6;
                for (int i = 0; i < 6; i++) {
                    int x = startX + (i * divWidth) + 1;
                    int onWidth = (int)(divWidth * 0.5f);
                    if (onWidth < 4) onWidth = 4;
                    display.fillRect(x, startY + 1, onWidth, barHeight - 2, COLOR_WAVE);
                }
            }
            break;
            
        case LOOP_SWING:
            // ██░██░░██░ - 3 divisions with 60/40 duty cycle
            {
                int divWidth = barWidth / 3;
                for (int i = 0; i < 3; i++) {
                    int x = startX + (i * divWidth) + 1;
                    int onWidth = (int)(divWidth * 0.6f) - 2;
                    display.fillRect(x, startY + 1, onWidth, barHeight - 2, COLOR_WAVE);
                }
            }
            break;
            
        case LOOP_STACCATO:
            // █░░█░░█░░ - 4 divisions with 25% duty cycle
            {
                int divWidth = barWidth / 4;
                for (int i = 0; i < 4; i++) {
                    int x = startX + (i * divWidth) + 1;
                    int onWidth = (int)(divWidth * 0.25f);
                    if (onWidth < 4) onWidth = 4;
                    display.fillRect(x, startY + 1, onWidth, barHeight - 2, COLOR_WAVE);
                }
            }
            break;
            
        case LOOP_GATE:
            // ████░░░░ - half on, half off
            {
                int halfWidth = (barWidth / 2) - 2;
                display.fillRect(startX + 1, startY + 1, halfWidth, barHeight - 2, COLOR_WAVE);
            }
            break;
            
        case LOOP_RATCHET:
            // █░█░█░█░█░█░ - 12 fast divisions
            {
                int divWidth = barWidth / 12;
                for (int i = 0; i < 12; i++) {
                    int x = startX + (i * divWidth);
                    int onWidth = (divWidth / 2);
                    if (onWidth < 3) onWidth = 3;
                    display.fillRect(x, startY + 1, onWidth, barHeight - 2, COLOR_WAVE);
                }
            }
            break;
            
        case LOOP_DOTTED:
            // ██░█░██░█░ - 4 hits: long, short, long, short
            {
                int divWidth = barWidth / 4;
                for (int i = 0; i < 4; i++) {
                    int x = startX + (i * divWidth) + 1;
                    int onWidth = (i % 2 == 0) ? (int)(divWidth * 0.6f) - 2 : (int)(divWidth * 0.3f);
                    if (onWidth < 4) onWidth = 4;
                    display.fillRect(x, startY + 1, onWidth, barHeight - 2, COLOR_WAVE);
                }
            }
            break;
            
        default:
            break;
    }
    
    // Draw border
    display.drawRect(startX, startY, barWidth, barHeight, COLOR_TEXT);
    
    lastDrawnPattern = loopState.pattern;
}
         
void readPotentiometers() {
    // ============ READ ALL POTS WITH HEAVY FILTERING ============
    // Use exponential moving average to smooth ADC readings
    static uint16_t pitchFiltered = 2048;
    static uint16_t speedFiltered = 2048;
    static uint16_t modFiltered = 2048;
    static uint16_t feedbackFiltered = 2048;
    static bool firstRead = true;
    
   // Read raw values multiple times and average (reduces ADC noise)
    adc_select_input(0); // Pitch
    uint32_t pitchSum = 0;
    for (int i = 0; i < 4; i++) {
        pitchSum += adc_read();
        delayMicroseconds(10);
    }
    uint16_t pitchRaw = 4095 - (pitchSum / 4);
    
    adc_select_input(1); // Speed
    uint32_t speedSum = 0;
    for (int i = 0; i < 4; i++) {
        speedSum += adc_read();
        delayMicroseconds(10);
    }
    uint16_t speedRaw = 4095 - (speedSum / 4);
    
    adc_select_input(2); // Modulation
    uint32_t modSum = 0;
    for (int i = 0; i < 4; i++) {
        modSum += adc_read();
        delayMicroseconds(10);
    }
    uint16_t modRaw = 4095 - (modSum / 4);
    
    adc_select_input(3); // Echo Feedback
    uint32_t feedbackSum = 0;
    for (int i = 0; i < 4; i++) {
    feedbackSum += adc_read();
    delayMicroseconds(10);
    }
    uint16_t feedbackRaw_temp = 4095 - (feedbackSum / 4);
    
    // Apply exponential moving average filter (alpha = 0.2 for heavier smoothing)
    // filtered = (alpha * new) + ((1-alpha) * old)
    if (firstRead) {
        pitchFiltered = pitchRaw;
        speedFiltered = speedRaw;
        modFiltered = modRaw;
        feedbackFiltered = feedbackRaw_temp;
        firstRead = false;
    } else {
        pitchFiltered = (pitchRaw * 2 + (pitchFiltered * 8)) / 10;
        speedFiltered = (speedRaw * 2 + (speedFiltered * 8)) / 10;
        modFiltered = (modRaw * 2 + (modFiltered * 8)) / 10;
        feedbackFiltered = (feedbackRaw_temp * 2 + (feedbackFiltered * 8)) / 10;
    }
    
    // Apply hysteresis/dead zone (prevent wobble from ±2 ADC counts)
    static uint16_t lastStablePitch = 2048;
    static uint16_t lastStableSpeed = 2048;
    static uint16_t lastStableMod = 2048;
    static uint16_t lastStableFeedback = 2048;
    
    if (abs((int)pitchFiltered - (int)lastStablePitch) > 8) {
        lastStablePitch = pitchFiltered;
    }
    if (abs((int)speedFiltered - (int)lastStableSpeed) > 8) {
        lastStableSpeed = speedFiltered;
    }
    if (abs((int)modFiltered - (int)lastStableMod) > 8) {
        lastStableMod = modFiltered;
    }
    if (abs((int)feedbackFiltered - (int)lastStableFeedback) > 8) {
        lastStableFeedback = feedbackFiltered;
    }
    
    // Use stable filtered values
    pitchRaw = lastStablePitch;
    speedRaw = lastStableSpeed;
    modRaw = lastStableMod;
    uint16_t feedbackRaw = lastStableFeedback;
    
    // ============ PARAMETER LOCKING THRESHOLD ============
    // Pot must move at least this much to "unlock" a locked parameter
    const uint16_t UNLOCK_THRESHOLD = 150; // ~3.6% of full range

   // === SEQUENCER MODE POT HANDLING ===
    if (sequencer.active) {
        // Pot 1: Only used for individual step pitch adjustment (handled in readKeyboard)
        
        // Pot 2: Sequencer speed (100-1000ms per step)
        sequencer.stepInterval = 100 + (uint16_t)((speedRaw / 4095.0f) * 900);
        
        // Pot 3: Filter cutoff (200-4000Hz) - INVERT so CW = brighter
        sequencer.filterCutoff = 200 + (uint16_t)(((4095 - modRaw) / 4095.0f) * 3800);
        
        // Pot 4: Echo/Delay feedback (0-95%) - INVERTED so CW = more feedback
        siren.echoFeedback = ((4095 - feedbackRaw) / 4095.0f) * 0.95f;
        
        return; // Don't process normal pot handling in sequencer mode
    }
    
// ============ MUTATION CONTROL MODE WITH PARAMETER LOCKING ============
    // If in a mutation mode (not OFF), Pot 1 controls that mode's parameter
    // ONLY when NOT in LFO2 mode (LFO2 needs Pot 1 for rate)
    // NOTE: Pot 1 controls BOTH mutation AND pitch/root simultaneously for all instruments
    if (mutate.mode != MUTATE_OFF && !lfo2.active) {
        switch (mutate.mode) {
            case MUTATE_AMOUNT:
                // Check if parameter is locked
                if (mutate.probabilityLocked) {
                    int diff = abs((int)pitchRaw - (int)mutate.lockedProbabilityRaw);
                    Serial.print("MUTATE_AMOUNT locked: pitchRaw=");
                    Serial.print(pitchRaw);
                    Serial.print(" lockedRaw=");
                    Serial.print(mutate.lockedProbabilityRaw);
                    Serial.print(" diff=");
                    Serial.print(diff);
                    Serial.print(" threshold=");
                    Serial.println(UNLOCK_THRESHOLD);
                    
                    // Check if pot has moved enough to unlock
                    if (diff > UNLOCK_THRESHOLD) {
                        mutate.probabilityLocked = false;
                        Serial.println("UNLOCK: Probability parameter unlocked");
                    } else {
                        // Stay locked - don't update parameter
                        Serial.println("Still locked - not enough movement");
                        break;
                    }
                }
                
                // Pot 1 controls mutation probability (0-100%)
                // Invert so CW = increase
                mutate.mutationProbability = (4095 - pitchRaw) / 4095.0f;
                mutate.lockedProbabilityRaw = pitchRaw; // Store current position
                Serial.print("MUTATE_AMOUNT updated: prob=");
                Serial.println(mutate.mutationProbability);
                break;
                
            case MUTATE_RHYTHM:
                // Check if parameter is locked
                if (mutate.scaleLocked) {
                    // Check if pot has moved enough to unlock
                    if (abs((int)pitchRaw - (int)mutate.lockedScaleRaw) > UNLOCK_THRESHOLD) {
                        mutate.scaleLocked = false;
                        Serial.println("UNLOCK: Scale parameter unlocked");
                    } else {
                        // Stay locked - don't update parameter
                        break;
                    }
                }
                
                // Pot 1 controls musical scale selection
                // Invert so CW = increase
                {
                    uint16_t invertedRaw = 4095 - pitchRaw;
                    uint8_t scaleSelect = (invertedRaw * SCALE_COUNT) / 4096;
                    if (scaleSelect >= SCALE_COUNT) scaleSelect = SCALE_COUNT - 1;
                    mutate.currentScale = scaleSelect;
                    mutate.lockedScaleRaw = pitchRaw; // Store current position
                }
                break;
                
            case MUTATE_CUTOFF:
                // Check if parameter is locked
                if (mutate.cutoffLocked) {
                    // Check if pot has moved enough to unlock
                    if (abs((int)pitchRaw - (int)mutate.lockedCutoffRaw) > UNLOCK_THRESHOLD) {
                        mutate.cutoffLocked = false;
                        Serial.println("UNLOCK: Cutoff parameter unlocked");
                    } else {
                        // Stay locked - don't update parameter
                        break;
                    }
                }
                
                // Pot 1 controls filter cutoff (200-4000)
                // Invert so CW = increase
                siren.filterCutoff = 200 + (uint16_t)(((4095 - pitchRaw) / 4095.0f) * 3800);
                mutate.lockedCutoffRaw = pitchRaw; // Store current position
                break;
                
            default:
                break;
        }
    } else {
        // CRITICAL: When we're NOT actively controlling mutation parameters (because we're in
        // Lead Synth or LFO2 mode), we need to check if locked parameters need unlocking
        // This prevents parameters from staying locked forever
        
        // Only check unlocking for probability (MUTATE_AMOUNT uses Pot 1)
        if (mutate.probabilityLocked && siren.instrumentType != INST_LEAD_SYNTH && !lfo2.active) {
            if (abs((int)pitchRaw - (int)mutate.lockedProbabilityRaw) > UNLOCK_THRESHOLD) {
                mutate.probabilityLocked = false;
                Serial.println("UNLOCK: Probability parameter unlocked (background check)");
            }
        }
        
        // Only check unlocking for scale (MUTATE_RHYTHM uses Pot 1)
        if (mutate.scaleLocked && siren.instrumentType != INST_LEAD_SYNTH && !lfo2.active) {
            if (abs((int)pitchRaw - (int)mutate.lockedScaleRaw) > UNLOCK_THRESHOLD) {
                mutate.scaleLocked = false;
                Serial.println("UNLOCK: Scale parameter unlocked (background check)");
            }
        }
        
        // Only check unlocking for cutoff (MUTATE_CUTOFF uses Pot 1)
        if (mutate.cutoffLocked && siren.instrumentType != INST_LEAD_SYNTH && !lfo2.active) {
            if (abs((int)pitchRaw - (int)mutate.lockedCutoffRaw) > UNLOCK_THRESHOLD) {
                mutate.cutoffLocked = false;
                Serial.println("UNLOCK: Cutoff parameter unlocked (background check)");
            }
        }
    }
    
    // ============ ARPEGGIATOR SPEED CONTROL ============
    // When arp is enabled and keys are held, Pot 2 controls arp speed - INVERTED so CW = faster
    // BUT NOT in LFO2 mode
    if (arp.enabled && keyboard.numKeysHeld > 1 && !lfo2.active) {
        arp.arpSpeed = 500 - (uint16_t)(((4095 - speedRaw) / 4095.0f) * 450); // 500-50ms
    }

    // ============ LOOP PATTERN TEMPO CONTROL ============
    // When loop mode is active, Pot 2 controls pattern tempo
    // BUT NOT in LFO2 mode
    if (loopState.active && loopState.pattern != LOOP_OFF && !lfo2.active) {
        // Map pot to 200-1000ms (fast to slow rhythm)
        loopState.patternTempo = 200 + (uint16_t)((speedRaw / 4095.0f) * 800);
    }

    // ============ REVERSE MODE POT CONTROLS ============
    if (reverser.active) {
        // Pot 1: Pitch shift (-2 to +2 octaves) - NOT INVERTED
        // Map 0-4095 to 5 discrete positions
        int potStep = (pitchRaw * 5) / 4096; // 0, 1, 2, 3, 4
        if (potStep >= 5) potStep = 4;
        reverser.pitchShift = potStep - 2; // Convert to -2, -1, 0, +1, +2
        
        // Pot 3: Wet/Dry mix (0-100%) - INVERTED so CW = more wet
        reverser.mixAmount = (4095 - modRaw) / 4095.0f;
    }

    // ============ HOLD MODE POT CONTROLS ============
    if (infiniteSustain && holdIsOn) {
        // Pot 4: Feedback amount - ONLY infinite when fully CW
        // When fully CW (feedbackRaw near 0 after inversion): 100% feedback (infinite hold)
        // When not fully CW: normal feedback range (0-95%)
        
        uint16_t invertedFeedback = 4095 - feedbackRaw;
        
        // Check if pot is in the top 5% (fully clockwise zone)
        if (invertedFeedback >= 3900) {  // Top 5% of pot range
            // Fully CW - lock to 100% for infinite sustain
            siren.echoFeedback = 1.0f;
        } else {
            // Not fully CW - normal feedback range (0-95%)
            siren.echoFeedback = (invertedFeedback / 4095.0f) * 0.95f;
        }
        
        // DON'T override Pot 3 - let it control its normal parameter
        // (modulation/filter/vibrato depending on instrument)
        // This prevents volume jumps when entering/exiting Hold mode
    }

    // ============ LFO 2 MODE POT CONTROLS ============
    if (lfo2.active) {
        static float lastLFO2Rate = 0;
        static float lastLFO2Depth = 0;
        
        // Pot 1: LFO 2 Rate (0.1-20 Hz) - INVERTED so CW = faster
        float newRate = 0.1f + (((4095 - pitchRaw) / 4095.0f) * 19.9f);
        
        // Pot 3: LFO 2 Depth (0-100%) - INVERTED so CW = more
        float newDepth = (4095 - modRaw) / 4095.0f;
        
        // Only update if changed significantly
        if (abs(newRate - lastLFO2Rate) > 0.2f) {
            lfo2.rate = newRate;
            lastLFO2Rate = newRate;
        }
        
        if (abs(newDepth - lastLFO2Depth) > 0.02f) {
            lfo2.depth = newDepth;
            lastLFO2Depth = newDepth;
        }
    }

    // ============ LFO 2 MODE POT CONTROLS ============
    if (lfo2.active) {
        static float lastLFO2Rate = 0;
        static float lastLFO2Depth = 0;
        
        // Pot 1: LFO 2 Rate (0.1-20 Hz) - INVERTED so CW = faster
        float newRate = 0.1f + (((4095 - pitchRaw) / 4095.0f) * 19.9f);
        
        // Pot 3: LFO 2 Depth (0-100%) - INVERTED so CW = more
        float newDepth = (4095 - modRaw) / 4095.0f;
        
        // Only update if changed significantly
        if (abs(newRate - lastLFO2Rate) > 0.2f) {
            lfo2.rate = newRate;
            lastLFO2Rate = newRate;
        }
        
        if (abs(newDepth - lastLFO2Depth) > 0.02f) {
            lfo2.depth = newDepth;
            lastLFO2Depth = newDepth;
        }
    }
    
    // ============ DRONE MODE POT CONTROLS ============
    if (drone.active) {
        static float lastDroneRoot = 0;
        static float lastDroneDetune = 0;
        static float lastDroneBright = 0;
        static float lastDroneDrift = 0;
        
        // Pot 1: Root frequency (20-400Hz) - INVERTED so CW = higher
        // ONLY update from pot when no keyboard key is pressed
        float newRoot;
        if (keyboard.numKeysHeld > 0) {
            // Keys are pressed - use keyboard frequency
            newRoot = drone.rootFreq; // Keep current (keyboard-set) frequency
        } else {
            // No keys pressed - read from pot
            newRoot = 20.0f + ((pitchRaw / 4095.0f) * 380.0f);
        }
        
        // Pot 2: Detune amount (0-100%) - INVERTED so CW = more
        float newDetune = (4095 - speedRaw) / 4095.0f;
        
        // Pot 3: Brightness (0-100%) - INVERTED so CW = brighter
        float newBright = (4095 - modRaw) / 4095.0f;
        
        // Pot 4: Drift speed (0.1-5Hz) - INVERTED so CW = faster
        float newDrift = 0.1f + (((4095 - feedbackRaw) / 4095.0f) * 4.9f);
        
        // Only update if changed significantly
        if (abs(newRoot - lastDroneRoot) > 5.0f) {
            drone.rootFreq = newRoot;
            lastDroneRoot = newRoot;
        }
        
        if (abs(newDetune - lastDroneDetune) > 0.02f) {
            drone.detune = newDetune;
            // Update oscillator detunes based on amount
            drone.oscDetunes[0] = -drone.detune * 0.02f;
            drone.oscDetunes[1] = -drone.detune * 0.01f;
            drone.oscDetunes[2] = 0.0f;
            drone.oscDetunes[3] = drone.detune * 0.01f;
            drone.oscDetunes[4] = drone.detune * 0.02f;
            lastDroneDetune = newDetune;
        }
        
        if (abs(newBright - lastDroneBright) > 0.02f) {
            drone.brightness = newBright;
            lastDroneBright = newBright;
        }
        
        if (abs(newDrift - lastDroneDrift) > 0.2f) {
            drone.driftSpeed = newDrift;
            lastDroneDrift = newDrift;
        }
        
        // CRITICAL: Skip normal feedback processing in drone mode
        // Drone doesn't use echo/delay, so don't update siren.echoFeedback here
    }

    // ============ SIDECHAIN MODE POT CONTROLS ============
    if (sidechain.active) {
        static float lastSidechainDepth = 0;
        static float lastSidechainAttack = 0;
        static float lastSidechainRelease = 0;
        
        // Pot 1: Ducking depth (0-100%) - INVERTED so CW = more ducking
        float newDepth = (4095 - pitchRaw) / 4095.0f;
        
        // Pot 2: Attack time (1-50ms) - INVERTED so CW = faster (lower ms)
        float newAttack = 50.0f - (((4095 - speedRaw) / 4095.0f) * 49.0f); // 50-1ms
        
        // Pot 3: Release time (50-500ms) - INVERTED so CW = faster (lower ms)
        float newRelease = 500.0f - (((4095 - modRaw) / 4095.0f) * 450.0f); // 500-50ms
        
        // Only update if changed significantly
        if (abs(newDepth - lastSidechainDepth) > 0.02f) {
            sidechain.depth = newDepth;
            lastSidechainDepth = newDepth;
        }
        
        if (abs(newAttack - lastSidechainAttack) > 2.0f) {
            sidechain.attackTime = newAttack;
            lastSidechainAttack = newAttack;
        }
        
        if (abs(newRelease - lastSidechainRelease) > 10.0f) {
            sidechain.releaseTime = newRelease;
            lastSidechainRelease = newRelease;
        }
    }
        
    if (siren.instrumentType == INST_RAY_GUN) {
        // ============ RAY GUN CONTROLS ============
        // Pot 1: Laser frequency (200-4000Hz)
        siren.baseFreq = 200.0f + ((pitchRaw / 4095.0f) * 3800.0f);
        
        // Pot 2: Sweep speed (same range as Dub Siren LFO) - INVERTED so CW = faster
        siren.sweepSpeed = 0.1f + (((4095 - speedRaw) / 4095.0f) * 19.9f); // 0.1-20 Hz
        
        // Pot 3: Resonance for filter "sharpness" - INVERTED so CW = more
        siren.resonance = 0.3f + (((4095 - modRaw) / 4095.0f) * 0.65f); // 0.3-0.95
        
        // Pot 4: Echo/Delay feedback (shared with Dub Siren) - INVERTED so CW = more
        siren.echoFeedback = ((4095 - feedbackRaw) / 4095.0f) * 0.95f;
        
        // LFO depth for PWM - use resonance value for modulation
        siren.lfoDepth = siren.resonance;
        
      } else if (siren.instrumentType == INST_DISCO) {
        // ============ DISCO CONTROLS ============
        // Pot 1: PITCH/RANGE - Base frequency (100-2000Hz)
        // NOT INVERTED - CW = higher pitch (opposite of other instruments)
        // ONLY update from pot when no key is pressed
        if (!keyboard.keyPressed) {
            siren.baseFreq = 100.0f + ((pitchRaw / 4095.0f) * 1900.0f);
        }
        
        // Pot 2: SPEED/TIME - How fast the effect happens (varies by mode)
        // INVERTED so CW = faster (STORE IN STRUCT!)
        siren.discoCharacter = (4095 - speedRaw) / 4095.0f;
        
        // Pot 3: BRIGHTNESS/FILTER - Overall brightness/filter cutoff
        // INVERTED so CW = brighter
        siren.discoBrightness = (4095 - modRaw) / 4095.0f;
        
        // Pot 4: Echo/Delay feedback (shared with other instruments)
        // INVERTED so CW = more
        siren.echoFeedback = ((4095 - feedbackRaw) / 4095.0f) * 0.95f;
    }
        
    else if (siren.instrumentType == INST_LEAD_SYNTH) {
        // ============ LEAD SYNTH CONTROLS ============
        // Pot 1: Root note frequency (200-800Hz) - QUANTIZED to musical notes
        // Works simultaneously with mutation control (like Dub Siren and Ray Gun)
        {
            // Map pot to scale steps (0-23 = 2 octaves of current scale)
            uint8_t scaleSize;
            switch (mutate.currentScale) {
                case SCALE_MINOR_PENT: scaleSize = minorPentatonicSize; break;
                case SCALE_MAJOR: scaleSize = majorScaleSize; break;
                case SCALE_BLUES: scaleSize = bluesScaleSize; break;
                case SCALE_MINOR: scaleSize = minorScaleSize; break;
                case SCALE_CHROMATIC: scaleSize = chromaticScaleSize; break;
                default: scaleSize = minorPentatonicSize; break;
            }
            
            // Map pot to 24 steps (2 octaves)
            uint8_t potStep = (pitchRaw * 24) / 4096;
            if (potStep >= 24) potStep = 23;
            
            // Quantize to musical pitch
            float newRootFreq = quantizePitchToScale(220.0f, potStep);
            
            // Only update if frequency changed significantly (prevents jitter)
            static float lastLeadRootFreq = 0;
            if (abs(newRootFreq - lastLeadRootFreq) > 5.0f) {
                siren.baseFreq = newRootFreq;
                generateMelody(); // Regenerate melody with new root
                lastLeadRootFreq = newRootFreq;
            }
        }
        
        // Pot 2: Tempo/Step speed (100-1000ms per step) - INVERTED so CW = faster (lower ms)
        siren.stepInterval = 1000 - (uint16_t)(((4095 - speedRaw) / 4095.0f) * 900);
        
        // Pot 3: Vibrato depth - from completely off to subtle
        // Invert so CCW = off, CW = maximum vibrato
        float modNormalized = (4095 - modRaw) / 4095.0f; // INVERTED
        if (modNormalized < 0.05f) {
            // Dead zone - completely off for clean, in-tune sound
            siren.vibratoDepth = 0.0f;
            siren.vibratoRate = 0.0f;
        } else {
            // Scale from 0.05 to 1.0 → 0.0 to 0.1 vibrato depth (much subtler)
            siren.vibratoDepth = ((modNormalized - 0.05f) / 0.95f) * 0.1f; // Max 10% pitch variation
            siren.vibratoRate = 4.0f + (((modNormalized - 0.05f) / 0.95f) * 6.0f); // 4-10 Hz
        }
        
        // Pot 4: Gate length (20-100%) - INVERTED so CW = more
        siren.gateLength = 20 + (uint8_t)(((4095 - feedbackRaw) / 4095.0f) * 80);       
    }
    
    else {
        // ============ DUB SIREN CONTROLS ============
    // Map pitch: Quantized to semitones from C2 (65.41Hz) to C6 (1046.5Hz) = 48 semitones (4 octaves)
        // Only quantize when NOT in mutate mode - preserve original behavior in mutate modes
        if (mutate.mode == MUTATE_OFF) {
            // Map pot to 48 semitones (4 octaves)
            uint8_t semitone = (pitchRaw * 48) / 4096;
            if (semitone >= 48) semitone = 47;
            
            // Convert semitone to frequency using equal temperament
            // C2 = 65.41Hz is our base (MIDI note 36)
            float baseFreq = 65.41f;
            siren.baseFreq = baseFreq * pow(2.0f, semitone / 12.0f);
        } else {
            // In mutate modes, use original smooth frequency mapping
            siren.baseFreq = 50.0f + ((pitchRaw / 4095.0f) * 1950.0f);
        }
        
        // Direct frequency update for instant response
        if (siren.mode != MODE_PORTAMENTO) {
            siren.frequency = siren.baseFreq;
        }
        
        // Map speed based on mode
        if (siren.mode == MODE_PORTAMENTO) {
            // In portamento mode, speed controls glide time - INVERTED so CW = faster (lower glide time)
            siren.glideTime = 0.2f - (((4095 - speedRaw) / 4095.0f) * 0.199f); // 0.2 to 0.001 (slow to fast glide)
        } else {
            // Normal LFO speed - INVERTED so CW = faster
            siren.lfoRate = 0.1f + (((4095 - speedRaw) / 4095.0f) * 19.9f);
        }
        
        // Map modulation pot based on mode
        switch (siren.mode) {
            case MODE_CLASSIC_DUB:
            case MODE_SQUARE_WAVE:
            case MODE_RING_MOD:
                // Normal LFO depth - INVERTED so CW = more
                siren.lfoDepth = ((4095 - modRaw) / 4095.0f);
                break;
                
            case MODE_DEEP_SUB:
                // Sub-bass mix - INVERTED so CW = more
                siren.subMix = ((4095 - modRaw) / 4095.0f);
                break;
                
            case MODE_LOFI_CRUSH:
                // Bit depth (1-8 bits) - INVERTED so CW = more bits (less crushing)
                siren.bitDepth = 1 + (uint8_t)(((4095 - modRaw) / 4095.0f) * 7);
                if (siren.bitDepth > 8) siren.bitDepth = 8;
                break;
                
            case MODE_PORTAMENTO:
                // LFO depth still works in portamento - INVERTED so CW = more
                siren.lfoDepth = ((4095 - modRaw) / 4095.0f) * 0.3f; // Reduced range
                break;
        }

        // Ring mod frequency (for ring mod mode) - INVERTED so CW = more
        if (siren.mode == MODE_RING_MOD) {
            siren.ringModFreq = 10.0f + (((4095 - modRaw) / 4095.0f) * 500.0f); // 10-510 Hz
        }
        
        // Map echo feedback: 0.0 to 0.95 (0% to 95% feedback) - INVERTED so CW = more
        // (unless infinite sustain is active OR drone mode is active)
        if (!infiniteSustain && !drone.active) {
            siren.echoFeedback = ((4095 - feedbackRaw) / 4095.0f) * 0.95f;
        }
    }
// Ring mod frequency (for ring mod mode) - INVERTED so CW = more
        if (siren.mode == MODE_RING_MOD) {
            siren.ringModFreq = 10.0f + (((4095 - modRaw) / 4095.0f) * 500.0f); // 10-510 Hz
        }
        
        // Map echo feedback: 0.0 to 0.95 (0% to 95% feedback) - INVERTED so CW = more
        // (unless infinite sustain is active - then stay locked at 0.98)
        if (!infiniteSustain) {
            siren.echoFeedback = ((4095 - feedbackRaw) / 4095.0f) * 0.95f;
        }
    
    // Only update keyboard frequencies when pot actually changes significantly
    static uint16_t lastPitchForKeyboard = 0;
    if (abs((int)pitchRaw - (int)lastPitchForKeyboard) > 50) {
        updateKeyboardFrequencies();
        lastPitchForKeyboard = pitchRaw;
    }
}

void readButtons() {
    // Check for Function Shift double-press to exit sequencer or recorder OR disable mutations
    static unsigned long lastFuncShiftPress = 0;
    static bool lastFuncShiftReading = HIGH;
    
    bool funcShiftReading = digitalRead(BTN_FUNC_SHIFT);
    unsigned long now = millis();
    
    if (funcShiftReading == LOW && lastFuncShiftReading == HIGH) {
        // Function button just pressed
        
        // Check if we're NOT in any special mode and mutation has been used
        bool inSpecialMode = (sequencer.active || recorder.active || reverser.active || 
                             infiniteSustain || lfo2.active || drone.active || 
                             sidechain.active || loopState.active);
        
        // Exit REVERSER with double-press
        if (reverser.active && (now - lastFuncShiftPress < 500)) {
            reverser.active = false;
            reverser.reversing = false;
            
            Serial.println("-> REVERSE MODE DEACTIVATED (double-press)");
            
            display.fillScreen(COLOR_BG);
            
            // Force COMPLETE redraw
            drawParameters(true);
            drawWaveform(true);
            
            delay(50);
            drawParameters();
            
            lastFuncShiftPress = 0;
            
            // Reset keyboard states
            for (uint8_t i = 0; i < 8; i++) {
                keyboard.keysHeld[i] = false;
                lastFuncKeyStates[i] = false;
            }
            keyboard.numKeysHeld = 0;
        } else { 
            // Single press - record time
            lastFuncShiftPress = now;
        }
        
        // Exit INFINITE HOLD with double-press
        if (infiniteSustain && (now - lastFuncShiftPress < 500)) {
            infiniteSustain = false;
            holdIsOn = false;  // Reset global hold state
            
            // Restore original feedback
            siren.echoFeedback = savedFeedback;
            
            // Clear the echo buffer to stop sustaining audio
            for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
                echoBuffer[i] = 0;
            }
            echoWritePos = 0;
            
            Serial.println("-> INFINITE HOLD DEACTIVATED (double-press)");
            Serial.println("   Echo buffer cleared");
            
            display.fillScreen(COLOR_BG);
            
            // Force COMPLETE redraw
            drawParameters(true);
            drawWaveform(true);
            
            delay(50);
            drawParameters();
            
            lastFuncShiftPress = 0;
            
            // Reset keyboard states
            for (uint8_t i = 0; i < 8; i++) {
                keyboard.keysHeld[i] = false;
                lastFuncKeyStates[i] = false;
            }
            keyboard.numKeysHeld = 0;
        }

        // Exit LFO 2 MODE with double-press
else if (lfo2.active && (now - lastFuncShiftPress < 500)) {
    lfo2.active = false;
    lfo2.enabled = false;
    lfo2.phase = 0;
    
    Serial.println("-> LFO 2 MODE DEACTIVATED (double-press)");
    
    display.fillScreen(COLOR_BG);
    drawParameters(true);
    drawWaveform(true);
    
    // CRITICAL: Manually redraw sub-headings (same as sequencer exit)
    delay(50);
    display.setTextSize(1);
    
    // GATE
    display.fillRect(245, 8, 75, 8, COLOR_BG);
    display.setCursor(245, 8);
    display.setTextColor(COLOR_MOD);
    display.print("GATE:");
    display.setTextColor(COLOR_TEXT);
    display.print(gateModeNames[siren.gateMode]);
    
    // OCTAVE
    display.fillRect(245, 20, 75, 8, COLOR_BG);
    display.setCursor(245, 20);
    display.setTextColor(COLOR_MOD);
    display.print("OCTAVE:");
    display.setTextColor(COLOR_TEXT);
    if (octaveShift == 0) {
        display.print("0");
    } else if (octaveShift > 0) {
        display.print("+");
        display.print(octaveShift);
    } else {
        display.print(octaveShift);
    }
    
    // DELAY
    display.fillRect(245, 32, 75, 8, COLOR_BG);
    display.setCursor(245, 32);
    display.setTextColor(COLOR_MOD);
    display.print("DELAY:");
    display.setTextColor(COLOR_TEXT);
    display.print(delayTimeNames[siren.delayIndex]);
    
    lastFuncShiftPress = 0;
    
    // Reset keyboard states
    for (uint8_t i = 0; i < 8; i++) {
        keyboard.keysHeld[i] = false;
        lastFuncKeyStates[i] = false;
    }
    keyboard.numKeysHeld = 0;
}

// Exit DRONE MODE with double-press
        else if (drone.active && (now - lastFuncShiftPress < 500)) {
            drone.active = false;
            drone.playMode = 0;  // Set to OFF
            drone.envelope = 0.0f;
            
            Serial.println("-> DRONE MODE DEACTIVATED (double-press)");
            
            display.fillScreen(COLOR_BG);
            drawParameters(true);
            drawWaveform(true);
            
            delay(50);
            
            // Redraw standard sub-headings
            display.setTextSize(1);
            
            // GATE
            display.fillRect(245, 8, 75, 8, COLOR_BG);
            display.setCursor(245, 8);
            display.setTextColor(COLOR_MOD);
            display.print("GATE:");
            display.setTextColor(COLOR_TEXT);
            display.print(gateModeNames[siren.gateMode]);
            
            // OCTAVE
            display.fillRect(245, 20, 75, 8, COLOR_BG);
            display.setCursor(245, 20);
            display.setTextColor(COLOR_MOD);
            display.print("OCTAVE:");
            display.setTextColor(COLOR_TEXT);
            if (octaveShift == 0) {
                display.print("0");
            } else if (octaveShift > 0) {
                display.print("+");
                display.print(octaveShift);
            } else {
                display.print(octaveShift);
            }
            
            // DELAY
            display.fillRect(245, 32, 75, 8, COLOR_BG);
            display.setCursor(245, 32);
            display.setTextColor(COLOR_MOD);
            display.print("DELAY:");
            display.setTextColor(COLOR_TEXT);
            display.print(delayTimeNames[siren.delayIndex]);
            
            lastFuncShiftPress = 0;
            
            // Reset keyboard states
            for (uint8_t i = 0; i < 8; i++) {
                keyboard.keysHeld[i] = false;
                lastFuncKeyStates[i] = false;
            }
            keyboard.numKeysHeld = 0;
        }

        // Exit SIDECHAIN MODE with double-press
        else if (sidechain.active && (now - lastFuncShiftPress < 500)) {
            sidechain.active = false;
            sidechain.enabled = false;
            sidechain.envelope = 1.0f;
            sidechain.triggered = false;
            
            Serial.println("-> SIDECHAIN MODE DEACTIVATED (double-press)");
            
            display.fillScreen(COLOR_BG);
            drawParameters(true);
            drawWaveform(true);
            
            delay(50);
            
            // Redraw standard sub-headings
            display.setTextSize(1);
            
            // GATE
            display.fillRect(245, 8, 75, 8, COLOR_BG);
            display.setCursor(245, 8);
            display.setTextColor(COLOR_MOD);
            display.print("GATE:");
            display.setTextColor(COLOR_TEXT);
            display.print(gateModeNames[siren.gateMode]);
            
            // OCTAVE
            display.fillRect(245, 20, 75, 8, COLOR_BG);
            display.setCursor(245, 20);
            display.setTextColor(COLOR_MOD);
            display.print("OCTAVE:");
            display.setTextColor(COLOR_TEXT);
            if (octaveShift == 0) {
                display.print("0");
            } else if (octaveShift > 0) {
                display.print("+");
                display.print(octaveShift);
            } else {
                display.print(octaveShift);
            }
            
            // DELAY
            display.fillRect(245, 32, 75, 8, COLOR_BG);
            display.setCursor(245, 32);
            display.setTextColor(COLOR_MOD);
            display.print("DELAY:");
            display.setTextColor(COLOR_TEXT);
            display.print(delayTimeNames[siren.delayIndex]);
            
            lastFuncShiftPress = 0;
            
            // Reset keyboard states
            for (uint8_t i = 0; i < 8; i++) {
                keyboard.keysHeld[i] = false;
                lastFuncKeyStates[i] = false;
            }
            keyboard.numKeysHeld = 0;
        }

      // Exit LOOP MODE with double-press
        else if (loopState.active && (now - lastFuncShiftPress < 500)) {
            loopState.active = false;
            loopState.pattern = LOOP_OFF;
            
            Serial.println("-> LOOP MODE DEACTIVATED (double-press)");
            
            display.fillScreen(COLOR_BG);
            
            // Force COMPLETE redraw
            drawParameters(true);
            drawWaveform(true);
            
            delay(50);
            drawParameters();
            
            lastFuncShiftPress = 0;
            
            // Reset keyboard states
            for (uint8_t i = 0; i < 8; i++) {
                keyboard.keysHeld[i] = false;
                lastFuncKeyStates[i] = false;
            }
            keyboard.numKeysHeld = 0;
        }

        // Exit RECORDER with double-press
        else if (recorder.active && (now - lastFuncShiftPress < 500)) {
            recorder.active = false;
            recorder.recording = false;
            recorder.playing = false;
            recorder.hasRecording = false;
            recorder.recordPosition = 0;
            recorder.recordLength = 0;
            recordKeySilence = false;
            Serial.println("-> RECORD MODE DEACTIVATED (double-press)");
            
            display.fillScreen(COLOR_BG);
            
            // Force COMPLETE redraw with reset of all static flags
            drawParameters(true);  // Pass true to force full reset
            drawWaveform(true);
            
            // Small delay then draw sub-headings last
            
            // Force one more parameter redraw to ensure everything shows
            drawParameters();
            
            lastFuncShiftPress = 0;
            
            // CRITICAL: Reset all keyboard states AND function key states
            for (uint8_t i = 0; i < 8; i++) {
                keyboard.keysHeld[i] = false;
                lastFuncKeyStates[i] = false;
            }
            keyboard.numKeysHeld = 0;
        }
        // Exit SEQUENCER with double-press
        else if (sequencer.active && (now - lastFuncShiftPress < 500)) {
            // Double-press detected within 500ms - exit sequencer
            sequencer.active = false;
            sequencer.running = false;
            Serial.println("-> SEQUENCER MODE DEACTIVATED (double-press)");
            display.fillScreen(COLOR_BG);
            
            // Force complete redraw
            drawParameters();
            drawWaveform(true);
            
            // Small delay then draw sub-headings last
            delay(50);
            
            display.setTextSize(1);
            // GATE
            display.fillRect(245, 8, 75, 8, COLOR_BG);
            display.setCursor(245, 8);
            display.setTextColor(COLOR_MOD);
            display.print("GATE:");
            display.setTextColor(COLOR_TEXT);
            display.print(gateModeNames[siren.gateMode]);
    
            // OCTAVE
            display.fillRect(245, 20, 75, 8, COLOR_BG);
            display.setCursor(245, 20);
            display.setTextColor(COLOR_MOD);
            display.print("OCTAVE:");
            display.setTextColor(COLOR_TEXT);
            if (octaveShift == 0) {
                display.print("0");
            } else if (octaveShift > 0) {
                display.print("+");
                display.print(octaveShift);
            } else {
                display.print(octaveShift);
            }
            
            // DELAY
            display.fillRect(245, 32, 75, 8, COLOR_BG);
            display.setCursor(245, 32);
            display.setTextColor(COLOR_MOD);
            display.print("DELAY:");
            display.setTextColor(COLOR_TEXT);
            display.print(delayTimeNames[siren.delayIndex]);
            
            lastFuncShiftPress = 0; // Reset
            
            // CRITICAL: Reset all keyboard states so Function + Key 1 works again
            for (uint8_t i = 0; i < 8; i++) {
                keyboard.keysHeld[i] = false;
            }
            keyboard.numKeysHeld = 0;
        } else { 
            // Single press - record time
            lastFuncShiftPress = now;
        }
    }
    
    lastFuncShiftReading = funcShiftReading;
    
    readButton(&funcShiftBtn);

    // ===== MENU BUTTON (GP0) - cycles through instruments =====
    if (readButton(&menuBtn)) {
        siren.instrumentType = (siren.instrumentType + 1) % INST_COUNT;
        
        // Reset appropriate parameters when switching instruments
        phase = 0;
        lfoPhase = 0;
        siren.rayGunPhase = 0;
        filterState = 0;
        siren.frequency = siren.baseFreq;
        
        // CLEAR ECHO BUFFER when switching to prevent ghost notes
        // (Especially important for keyboard synth which changes pitch per note)
        for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
            echoBuffer[i] = 0;
        }
        echoWritePos = 0;
        
        // Force full redraw of parameters (header, mode, pots, sub-headings)
        drawParameters();
    }
    
    // ===== DELAY BUTTON (GP5) - instant response =====
    if (readButton(&delayBtn)) {
    siren.delayIndex = (siren.delayIndex + 1) % 6;
    siren.echoTime = delayTimes[siren.delayIndex];
    drawSubHeading(32, "DELAY", delayTimeNames[siren.delayIndex]);
}
    
  // ===== MODE BUTTON (GP4) - changes sub-modes =====
if (readButton(&modeBtn)) {
    if (sequencer.active) {
        // In sequencer mode - cycle sound modes
        sequencer.soundMode = (sequencer.soundMode + 1) % SEQ_SOUND_COUNT;
        Serial.print("Sequencer sound mode: ");
        Serial.println(seqSoundModeNames[sequencer.soundMode]);
        
        // SEQUENCER ONLY: Force immediate display update - EXACT width constraint
        display.fillRect(5, 23, 95, 10, COLOR_BG);  // 95px max to avoid overlap
        display.setTextSize(1);
        display.setCursor(5, 23);
        display.setTextColor(COLOR_MOD);  // Yellow heading
        display.print("SOUND:");
        display.setTextColor(COLOR_TEXT);  // White value
        display.print(seqSoundModeNames[sequencer.soundMode]);
    } else if (drone.active) {
        // In drone mode - cycle chord modes
        drone.chordMode = (drone.chordMode + 1) % 5;  // Assuming 5 chord modes
        Serial.print("Drone chord mode: ");
        Serial.println(drone.chordMode);
        
        // Force immediate display update
        display.fillRect(5, 23, 95, 10, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(5, 23);
        display.setTextColor(COLOR_MOD);
        display.print("CHORD:");
        display.setTextColor(COLOR_TEXT);
        // Add chord mode names here when you define them
        display.print(droneChordModeNames[drone.chordMode]);
    } else if (siren.instrumentType == INST_DUB_SIREN) {
        // Normal mode - Dub Siren
        siren.mode = (siren.mode + 1) % MODE_COUNT;
    } else if (siren.instrumentType == INST_RAY_GUN) {
        // Normal mode - Ray Gun
        siren.rayGunMode = (siren.rayGunMode + 1) % RAYGUN_MODE_COUNT;
    } else if (siren.instrumentType == INST_LEAD_SYNTH) {
        // Normal mode - Lead Synth
        siren.leadSynthMode = (siren.leadSynthMode + 1) % LEAD_MODE_COUNT;
    } else if (siren.instrumentType == INST_DISCO) {
        // Normal mode - Disco
        siren.discoMode = (siren.discoMode + 1) % DISCO_MODE_COUNT;
        // Force immediate display update for disco
        display.fillRect(5, 23, 95, 10, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(5, 23);
        display.setTextColor(COLOR_MOD);
        display.print("MODE:");
        display.setTextColor(COLOR_TEXT);
        display.print(discoModeNames[siren.discoMode]);
    }
    
    // Reset phase accumulators for clean mode change
    phase = 0;
    lfoPhase = 0;
    siren.rayGunPhase = 0;
}
    // Function button - cycles gate length only
if (readButton(&functionBtn)) {
    if (sequencer.active) {
        sequencer.gateLength += 20;
        if (sequencer.gateLength > 100) sequencer.gateLength = 20;
        drawSubHeadingGatePercent(8, sequencer.gateLength);
    } else {
        siren.gateMode = (siren.gateMode + 1) % GATE_MODE_COUNT;
        drawSubHeading(8, "GATE", gateModeNames[siren.gateMode]);
    }
}
    
// ===== OCTAVE BUTTON (GP23) - cycles octave shift =====
    if (readButton(&octaveBtn)) {
        if (sequencer.active) {
            sequencer.octaveShift++;
            if (sequencer.octaveShift > 2) sequencer.octaveShift = -2;
            Serial.print("Sequencer octave shift: ");
            Serial.println(sequencer.octaveShift);
            drawSubHeadingOctave(20, sequencer.octaveShift);
        } else {
            octaveShift++;
            if (octaveShift > 2) octaveShift = -1;
            drawSubHeadingOctave(20, octaveShift);
        }
    }
}  

// =====================================================================
// DISPLAY FUNCTIONS
// =====================================================================
void drawParameter(int x, int y, int width, const char* label, const char* value, const char* suffix, uint16_t valueColor) {
    display.fillRect(x, y, width, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(x, y);
    display.setTextColor(COLOR_MOD);
    display.print(label);
    display.setTextColor(valueColor);
    display.print(value);
    if (suffix && strlen(suffix) > 0) {
        display.print(suffix);
    }
}

void drawParameter(int x, int y, int width, const char* label, int value, const char* suffix, uint16_t valueColor) {
    display.fillRect(x, y, width, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(x, y);
    display.setTextColor(COLOR_MOD);
    display.print(label);
    display.setTextColor(valueColor);
    display.print(value);
    if (suffix && strlen(suffix) > 0) {
        display.print(suffix);
    }
}

void drawSubHeading(int y, const char* label, const char* value, uint16_t valueColor) {
    display.fillRect(245, y, 75, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(245, y);
    display.setTextColor(COLOR_MOD);
    display.print(label);
    display.print(":");
    display.setTextColor(valueColor);
    display.print(value);
}

void drawSubHeadingOctave(int y, int8_t octaveValue) {
    display.fillRect(245, y, 75, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(245, y);
    display.setTextColor(COLOR_MOD);
    display.print("OCTAVE:");
    display.setTextColor(COLOR_TEXT);
    if (octaveValue == 0) {
        display.print("0");
    } else if (octaveValue > 0) {
        display.print("+");
        display.print(octaveValue);
    } else {
        display.print(octaveValue);
    }
}

void drawSubHeadingGatePercent(int y, uint8_t percentage) {
    display.fillRect(245, y, 75, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(245, y);
    display.setTextColor(COLOR_MOD);
    display.print("GATE:");
    display.setTextColor(COLOR_TEXT);
    display.print(percentage);
    display.print("%");
}

// =====================================================================
// DISPLAY FUNCTIONS
// =====================================================================

void drawWaveform(bool forceRedraw) {
    // === SEQUENCER MODE DISPLAY ===
    if (sequencer.active) {
        // Track last drawn state to redraw only what changed
        static float lastDrawnFreqs[8] = {0,0,0,0,0,0,0,0};
        static bool lastDrawnEnabled[8] = {false,false,false,false,false,false,false,false};
        static uint8_t lastDrawnCurrentStep = 255;
        static bool sequencerDisplayInit = false;
        
        // Initial full draw
        if (!sequencerDisplayInit || forceRedraw) {
            display.fillRect(0, 45, 320, 120, COLOR_BG);
            sequencerDisplayInit = true;
            // Mark all as needing redraw
            for (uint8_t i = 0; i < 8; i++) {
                lastDrawnFreqs[i] = -1;
            }
            lastDrawnCurrentStep = 255;
        }
        
        // Draw 8 step indicators - only redraw changed steps
        int stepWidth = 35;
        int stepSpacing = 40;
        int startX = 0;
        int startY = 55;
        int maxStepHeight = 95;  // Increased height for taller bars
        
        for (uint8_t i = 0; i < 8; i++) {
            int x = startX + (i * stepSpacing);
            
            // Check if this step needs redrawing
            bool stepChanged = false;
            if (abs(sequencer.stepFrequencies[i] - lastDrawnFreqs[i]) > 10.0f) stepChanged = true;
            if (sequencer.stepEnabled[i] != lastDrawnEnabled[i]) stepChanged = true;
            if (sequencer.running && (i == sequencer.currentStep || i == lastDrawnCurrentStep)) stepChanged = true;
            if (!sequencerDisplayInit || forceRedraw) stepChanged = true;
            
            // Only redraw if this step changed
            if (stepChanged) {
                // Clear this step's area only
                display.fillRect(x, startY, stepWidth, maxStepHeight + 10, COLOR_BG);
                
                // Determine color
                uint16_t stepColor;
                if (sequencer.running && i == sequencer.currentStep) {
                    stepColor = COLOR_ACTIVE; // Red for current step
                } else if (sequencer.stepEnabled[i]) {
                    stepColor = COLOR_WAVE; // Green for enabled
                } else {
                    stepColor = COLOR_GRID; // Dim for disabled
                }
                
                // Calculate bar height based on pitch (if step is enabled)
                int barHeight;
                if (sequencer.stepEnabled[i]) {
                    // Map frequency: 55Hz to 880Hz (4 octaves)
                    float freq = sequencer.stepFrequencies[i];
                    
                    // Clamp to wide range
                    if (freq < 55.0f) freq = 55.0f;
                    if (freq > 880.0f) freq = 880.0f;
                    
                    // Linear mapping in log space (musical octaves)
                    float logMin = log2f(55.0f);
                    float logMax = log2f(880.0f);
                    float logFreq = log2f(freq);
                    float normalizedHeight = (logFreq - logMin) / (logMax - logMin);
                    
                    // Clamp normalized height to 0-1 range
                    if (normalizedHeight < 0.0f) normalizedHeight = 0.0f;
                    if (normalizedHeight > 1.0f) normalizedHeight = 1.0f;
                    
                    // Map to pixel height: 15 (low) to 70 (high)
                    barHeight = 15 + (int)(normalizedHeight * 55.0f);
                } else {
                    barHeight = 10; // Disabled step - very short
                }
                
                // Draw step bar
                display.fillRect(x, startY + (maxStepHeight - barHeight), stepWidth, barHeight, stepColor);
                
                // Draw step outline
                display.drawRect(x, startY, stepWidth, maxStepHeight, COLOR_TEXT);
                
                // Draw step number
                display.setTextSize(1);
                display.setTextColor(COLOR_TEXT);
                display.setCursor(x + 13, startY + maxStepHeight + 5);
                display.print(i + 1);
                
                // Save state for next frame
                lastDrawnFreqs[i] = sequencer.stepFrequencies[i];
                lastDrawnEnabled[i] = sequencer.stepEnabled[i];
            }
        }
        
        // Save playback state
        lastDrawnCurrentStep = sequencer.currentStep;
        
      // === DRAW FEEDBACK INDICATOR BAR (like normal mode) ===
static int lastSeqBarWidth = -1;
int barWidth = (int)(siren.echoFeedback * 320);

// Force redraw when entering sequencer or when forced
// Only update if change is significant (more than 8 pixels) or forced
if (lastSeqBarWidth == -1 || abs(barWidth - lastSeqBarWidth) > 8 || forceRedraw) {
    display.fillRect(0, 165, 320, 2, COLOR_BG);
    if (barWidth > 0) {
        display.fillRect(0, 165, barWidth, 2, COLOR_ECHO);
    }
    lastSeqBarWidth = barWidth;
}
        
        return; // Don't draw normal waveform in sequencer mode
    }
    
    // === NORMAL WAVEFORM DISPLAY ===
    // (All the original waveform code goes here - everything that was in the old function)
    
    static bool gridDrawn = false;
    static bool indicatorsDrawn = false;
    
    // Only draw grid once at startup or when forced
    if (!gridDrawn || forceRedraw) {
        // Clear waveform area
        display.fillRect(0, 45, 320, 120, COLOR_BG);
        
        // Draw subtle reference grid
        for (int x = 0; x < 320; x += 80) {
            for (int y = 50; y < 160; y += 4) {
                display.drawPixel(x, y, COLOR_GRID);
            }
        }
        
        // Draw center line
        display.drawFastHLine(0, 105, 320, 0x3186);
        gridDrawn = true;
    } else {
    // Just clear the waveform area without redrawing grid
    // BUT preserve text areas and loop indicator
    for (int x = 0; x < 320; x++) {
        // Skip the recorder LOOP text area (Y: 59-69, X: 3-127) when recorder is active
        // Skip the loop pattern indicator bar (Y: 68-80, X: 3-127) when loop pattern is active
        bool skipRecorderLoop = recorder.active && x >= 3 && x <= 127;
        bool skipPatternBar = loopState.pattern != LOOP_OFF && x >= 3 && x <= 127;
        
        if (skipRecorderLoop && skipPatternBar) {
            // Both active - skip Y: 59-80
            display.drawFastVLine(x, 55, 4, COLOR_BG);   // Clear above (Y: 55-58)
            display.drawFastVLine(x, 80, 75, COLOR_BG);  // Clear below (Y: 80-154)
        } else if (skipRecorderLoop) {
            // Only recorder LOOP text - skip Y: 59-69
            display.drawFastVLine(x, 55, 4, COLOR_BG);   // Clear above (Y: 55-58)
            display.drawFastVLine(x, 69, 86, COLOR_BG);  // Clear below (Y: 69-154)
        } else if (skipPatternBar) {
            // Only pattern bar - skip Y: 68-80 (expanded to 12px tall)
            display.drawFastVLine(x, 55, 13, COLOR_BG);  // Clear above (Y: 55-67)
            display.drawFastVLine(x, 80, 75, COLOR_BG);  // Clear below (Y: 80-154)
        } else {
            // Normal clear
            display.drawFastVLine(x, 55, 100, COLOR_BG);
        }
    }  
    // Redraw center line
    display.drawFastHLine(0, 105, 320, 0x3186);
}
    
    // Determine colors based on state
    uint16_t waveColor = COLOR_GRID;
    uint16_t trailColor = 0x1082;
    
    if (siren.triggered) {
        if (siren.delayIndex > 0 && siren.echoFeedback > 0.1f) {
            waveColor = COLOR_ECHO;
            trailColor = 0x0339;
        } else if (siren.lfoDepth > 0.6f) {
            waveColor = COLOR_MOD;
            trailColor = 0x8C20;
        } else {
            waveColor = COLOR_WAVE;
            trailColor = 0x0320;
        }
    } else {
        waveColor = 0x632C;
        trailColor = 0x2104;
    }
    
    // Calculate wave parameters for display
    float displayFreq;
    
    if (siren.instrumentType == INST_LEAD_SYNTH) {
        if (siren.triggered && siren.noteFrequencies[siren.currentStep] > 0) {
            displayFreq = siren.noteFrequencies[siren.currentStep] / 100.0f;
        } else {
            displayFreq = siren.baseFreq / 100.0f;
        }
    } else {
        displayFreq = siren.baseFreq / 100.0f;
    }
    
    if (displayFreq < 1.0f) displayFreq = 1.0f;
    float cyclesOnScreen = displayFreq / 5.0f;
    if (cyclesOnScreen < 1.0f) cyclesOnScreen = 1.0f;
    if (cyclesOnScreen > 8.0f) cyclesOnScreen = 8.0f;
    
    // Generate smooth waveform shape
    int16_t wavePoints[320];
    
    for (int x = 0; x < 320; x++) {
        float t = (x / 320.0f) * cyclesOnScreen;
        float lfoValue = sin(t * 3.14159f * 2.0f * (siren.lfoRate / 5.0f));
        float modAmount = lfoValue * siren.lfoDepth * 0.3f;
        float phase = fmod(t + modAmount, 1.0f);
        float sawValue = (phase * 2.0f) - 1.0f;
        
        float filterAmount = siren.filterCutoff / 4095.0f;
        if (filterAmount < 0.2f) filterAmount = 0.2f;
        
        if (filterAmount < 0.8f) {
            float smoothFactor = 1.0f - filterAmount;
            sawValue = sawValue * (1.0f - smoothFactor * 0.5f);
        }
        
        int16_t y = 105 - (int16_t)(sawValue * 50.0f);
        if (y < 55) y = 55;
        if (y > 155) y = 155;
        wavePoints[x] = y;
    }
    
    // Draw echo trails
    if (siren.delayIndex > 0 && siren.triggered && siren.echoFeedback > 0.1f) {
        int numEchoes = 2 + (int)(siren.echoFeedback * 3);
        if (numEchoes > 5) numEchoes = 5;
        int baseSpacing = 15 + (siren.delayIndex * 10);
        
        for (int echo = numEchoes; echo >= 1; echo--) {
            int offset = echo * baseSpacing;
            uint16_t echoColor;
            if (echo >= 5) echoColor = 0x0116;
            else if (echo == 4) echoColor = 0x0197;
            else if (echo == 3) echoColor = 0x0218;
            else if (echo == 2) echoColor = 0x02D9;
            else echoColor = 0x039A;
            
            for (int x = 1; x < 320 - offset; x++) {
                int x1 = x + offset - 1;
                int x2 = x + offset;
                
                // Skip if line crosses loop bar Y-range (68-80, X: 0-127) when loop is active
                if (loopState.active && x <= 127) {
                    int y1 = wavePoints[x-1];
                    int y2 = wavePoints[x];
                    if ((y1 >= 68 && y1 <= 80) || (y2 >= 68 && y2 <= 80)) continue;
                }
                
                if (x1 >= 0 && x2 < 320) {
                    display.drawLine(x1, wavePoints[x-1], x2, wavePoints[x], echoColor);
                }
            }
        }
    }
    
   // Draw main waveform
    for (int x = 1; x < 320; x++) {
        // Skip if waveform crosses text areas or loop bar
        bool skipForLoop = false;
        int y1 = wavePoints[x-1];
        int y2 = wavePoints[x];
        
        // Skip recorder LOOP text area (Y: 59-69) when recorder is active
        if (recorder.active && ((y1 >= 59 && y1 <= 69) || (y2 >= 59 && y2 <= 69))) {
            skipForLoop = true;
        }
        // Skip loop pattern indicator bar (Y: 68-80, X: 0-127) when loop mode is active
        if (loopState.active && x <= 127 && ((y1 >= 68 && y1 <= 80) || (y2 >= 68 && y2 <= 80))) {
            skipForLoop = true;
        }
        
        if (!skipForLoop) {
            display.drawLine(x-1, wavePoints[x-1], x, wavePoints[x], waveColor);
            
            if (siren.triggered) {
                display.drawLine(x-1, wavePoints[x-1]+1, x, wavePoints[x]+1, waveColor);
                
                if (abs(wavePoints[x] - wavePoints[x-1]) > 3) {
                    display.drawLine(x-1, wavePoints[x-1]-1, x, wavePoints[x]-1, trailColor);
                }
            }
        }
    }
    
    // Draw indicator bar with strong hysteresis to prevent jitter
static int lastBarWidth = -1;
int barWidth;

if (siren.instrumentType == INST_LEAD_SYNTH) {
    barWidth = (int)((siren.gateLength / 100.0f) * 320);
} else {
    barWidth = (int)(siren.echoFeedback * 320);
}

// Force redraw on mode changes or if first draw
// Only update if change is significant (more than 8 pixels) or first draw or forced
if (lastBarWidth == -1 || abs(barWidth - lastBarWidth) > 8 || forceRedraw) {
    display.fillRect(0, 165, 320, 2, COLOR_BG);
    if (barWidth > 0) {
        display.fillRect(0, 165, barWidth, 2, COLOR_ECHO);
    }
    lastBarWidth = barWidth;
}
    
    indicatorsDrawn = true;

    // Draw loop pattern indicator (if loop is active) - ALWAYS draw it here
    if (!sequencer.active) {
        drawLoopPatternIndicator();
    }
}  

void drawParameters(bool forceFullRedraw) {
    static bool headerDrawn = false;
    static bool statusAreaDrawn = false;
    static bool paramLabelsDrawn = false;
    static bool delayLabelDrawn = false;
    static bool gateLabelDrawn = false;
    static uint8_t lastMode = 255; // Force initial draw
    static bool lastSequencerWasActive = false;
    static int lastPot1Value = -999;
    static int lastPot2Value = -999;
    static int lastPot3Value = -999;
    static uint8_t lastPotInstrument = 255;
    
    // FORCE RESET if requested (for exiting special modes)
    if (forceFullRedraw) {
        headerDrawn = false;
        statusAreaDrawn = false;
        paramLabelsDrawn = false;
        delayLabelDrawn = false;
        gateLabelDrawn = false;
        lastMode = 255;
        lastPot1Value = -999;
        lastPot2Value = -999;
        lastPot3Value = -999;
        lastPotInstrument = 255;
    }
   
    // Reset ALL flags when sequencer mode changes
if (sequencer.active != lastSequencerWasActive) {
    headerDrawn = false;
    statusAreaDrawn = false;
    paramLabelsDrawn = false;
    delayLabelDrawn = false;
    gateLabelDrawn = false;
    lastMode = 255;
    lastSequencerWasActive = sequencer.active;
}

// Force redraw when entering/exiting sequencer or changing instruments
bool potForceRedraw = (siren.instrumentType != lastPotInstrument) || (sequencer.active != lastSequencerWasActive) || forceFullRedraw;
if (potForceRedraw) {
    lastPotInstrument = siren.instrumentType;
    lastPot1Value = -999;
    lastPot2Value = -999;
    lastPot3Value = -999;
    
    // CRITICAL FIX: Unlock AND invalidate locked pot positions when changing instruments
    mutate.probabilityLocked = false;
    mutate.scaleLocked = false;
    mutate.cutoffLocked = false;
    mutate.lockedProbabilityRaw = 0;  // Invalidate (will be set on first pot read)
    mutate.lockedScaleRaw = 0;
    mutate.lockedCutoffRaw = 0;
    
    Serial.println("Instrument changed - all mutation parameters unlocked and reset");
}
    
// Draw header - update when instrument changes OR when forced OR in sequencer mode
static uint8_t lastHeaderInstrument = 255;
if (!headerDrawn || siren.instrumentType != lastHeaderInstrument || sequencer.active) {
    // Clear ONLY the left side of header (instrument name area) - wider to ensure full clear
    display.fillRect(0, 0, 125, 20, COLOR_BG);  // Increased width and height to fully clear
    display.setTextSize(2);
    display.setTextColor(COLOR_TEXT);
    display.setCursor(5, 5);
    
    // Show SEQUENCER in header when in that mode
    if (sequencer.active) {
        display.print("SEQUENCER");
    } else {
        display.print(instrumentNames[siren.instrumentType]);
    }
    
    headerDrawn = true;
    lastHeaderInstrument = siren.instrumentType;
}

// Draw sound/mode name
display.fillRect(5, 23, 95, 10, COLOR_BG);  // Exact width: 95px max
display.setTextSize(1);
display.setTextColor(COLOR_MOD);  // Yellow heading
display.setCursor(5, 23);

if (sequencer.active) {
    display.print("SOUND:");
    display.setTextColor(COLOR_TEXT);  // White value
    display.print(seqSoundModeNames[sequencer.soundMode]);
} else if (drone.active) {
    display.print("CHORD:");
    display.setTextColor(COLOR_TEXT);  // White value
    display.print(droneChordModeNames[drone.chordMode]);
} else {
    display.print("MODE:");
    display.setTextColor(COLOR_TEXT);  // White value
    
    if (siren.instrumentType == INST_DUB_SIREN) {
        display.print(modeNames[siren.mode]);
    } else if (siren.instrumentType == INST_RAY_GUN) {
        display.print(rayGunModeNames[siren.rayGunMode]);
    } else if (siren.instrumentType == INST_LEAD_SYNTH) {
        display.print(leadSynthModeNames[siren.leadSynthMode]);
    } else if (siren.instrumentType == INST_DISCO) {
        display.print(discoModeNames[siren.discoMode]);
    }
}

// Draw MUTATE status directly under MODE (ALWAYS visible in non-sequencer mode)
if (!sequencer.active) {
    display.fillRect(5, 35, 138, 10, COLOR_BG);  // Clear up to X=143 (5px before POT at 148)
    display.setTextSize(1);
    display.setCursor(5, 35);
    display.setTextColor(COLOR_MOD);  // Yellow heading
    display.print("MUTATE:");
    if (mutate.mode == MUTATE_OFF) {
        display.setTextColor(COLOR_GRID);  // Gray when OFF
        display.print("OFF");
    } else {
        display.setTextColor(COLOR_TEXT);  // White when active
        display.print(mutationModeNames[mutate.mode]);
        
        // Display mutation parameter value next to mode name
        display.print(" ");
        switch (mutate.mode) {
            case MUTATE_AMOUNT:
                // Show probability percentage
                display.print((int)(mutate.mutationProbability * 100));
                display.print("%");
                break;
                
            case MUTATE_RHYTHM:
                // Show current scale
                display.print(scaleNames[mutate.currentScale]);
                break;
                
            case MUTATE_CUTOFF:
                // Show filter cutoff value
                display.print(siren.filterCutoff);
                display.print("Hz");
                break;
        }
    }
}

// Draw REVERSE status directly under MUTATE (only when active) (Y=47)
static bool lastReverserWasActive = false;

if (!sequencer.active && reverser.active) {
    // Always draw REVERSE status when reverser is active
    display.fillRect(5, 47, 120, 10, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(5, 47);
    display.setTextColor(COLOR_MOD);
    display.print("REVERSE:");
    display.setTextColor(reverser.reversing ? COLOR_ACTIVE : COLOR_GRID);
    display.print(reverser.reversing ? "ON" : "OFF");
    
    lastReverserWasActive = true;
}

// When NOT in reverser mode, clear Y=47
if (!sequencer.active && !reverser.active && lastReverserWasActive) {
    display.fillRect(5, 47, 120, 10, COLOR_BG);
    lastReverserWasActive = false;
}

// Draw HOLD status (only when active)
static bool lastSustainWasActive = false;
static bool lastHoldIsOn = false;

if (!sequencer.active && infiniteSustain) {
    // Calculate Y position based on what else is active
    int sustainY = 47;
    if (reverser.active) sustainY = 59;
    
    // Only redraw if hold state changed
    if (holdIsOn != lastHoldIsOn || forceFullRedraw) {
    
    // Always draw HOLD status when mode is active
    display.fillRect(5, sustainY, 120, 10, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(5, sustainY);
    display.setTextColor(COLOR_MOD);
    display.print("HOLD:");
    
    if (holdIsOn) {
            display.setTextColor(COLOR_ACTIVE);  // Red when ON
            // Show feedback percentage
            int fbPercent = (int)(siren.echoFeedback * 100);
            display.print(fbPercent);
            display.print("%");
        } else {
            display.setTextColor(COLOR_GRID);  // Gray when OFF
            display.print("OFF");
        }
        
        lastHoldIsOn = holdIsOn;
    }
    
    lastSustainWasActive = true;
}

// When NOT in hold mode, clear the area (only once)
if (!sequencer.active && !infiniteSustain && lastSustainWasActive) {
    display.fillRect(5, 47, 120, 10, COLOR_BG);
    display.fillRect(5, 59, 120, 10, COLOR_BG);
    lastSustainWasActive = false;
}

// Draw LFO 2 status (only when active)
static bool lastLFO2WasActive = false;
static bool lastLFO2IsOn = false;
static int lastLFO2RateDisplay = -1;
static int lastLFO2DepthDisplay = -1;

if (!sequencer.active && lfo2.active) {
    // Calculate Y position based on what else is active
    int lfo2Y = 47;
    if (reverser.active) lfo2Y = 59;
    if (infiniteSustain) lfo2Y = 59;
    if (recorder.active) lfo2Y = 59;
    if (reverser.active && infiniteSustain) lfo2Y = 71;
    if (reverser.active && recorder.active) lfo2Y = 71;
    if (infiniteSustain && recorder.active) lfo2Y = 71;
    
    // Calculate display values
    int rateDisplay = (int)(lfo2.rate * 10);
    int depthDisplay = (int)(lfo2.depth * 100);
    
    // Redraw if state changed OR values changed
    if (lfo2.enabled != lastLFO2IsOn || 
        rateDisplay != lastLFO2RateDisplay || 
        depthDisplay != lastLFO2DepthDisplay || 
        forceFullRedraw) {
        
        display.fillRect(5, lfo2Y, 120, 10, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(5, lfo2Y);
        display.setTextColor(COLOR_MOD);
        display.print("LFO2:");
        
        if (lfo2.enabled) {
            display.setTextColor(COLOR_ACTIVE);  // Red when ON
            display.print("ON ");
            // Show rate
            display.print(rateDisplay);
            display.print("Hz ");
            // Show depth
            display.print(depthDisplay);
            display.print("%");
        } else {
            display.setTextColor(COLOR_GRID);  // Gray when OFF
            display.print("OFF");
        }
        
        lastLFO2IsOn = lfo2.enabled;
        lastLFO2RateDisplay = rateDisplay;
        lastLFO2DepthDisplay = depthDisplay;
    }
    
    lastLFO2WasActive = true;
}

// When NOT in LFO2 mode, clear the area (only once)
if (!sequencer.active && !lfo2.active && lastLFO2WasActive) {
    display.fillRect(5, 47, 120, 10, COLOR_BG);
    display.fillRect(5, 59, 120, 10, COLOR_BG);
    display.fillRect(5, 71, 120, 10, COLOR_BG);
    lastLFO2WasActive = false;
}

// Draw DRONE status (only when active)
static bool lastDroneWasActive = false;
static int lastDroneRootDisplay = -1;
static int lastDroneDetuneDisplay = -1;

if (!sequencer.active && drone.active) {
    // Calculate Y position based on what else is active
    int droneY = 47;
    if (reverser.active) droneY = 59;
    if (infiniteSustain) droneY = 59;
    if (lfo2.active) droneY = 59;
    if (recorder.active) droneY = 59;
    if (reverser.active && infiniteSustain) droneY = 71;
    if (reverser.active && lfo2.active) droneY = 71;
    if (infiniteSustain && lfo2.active) droneY = 71;
    if (reverser.active && recorder.active) droneY = 71;
    if (infiniteSustain && recorder.active) droneY = 71;
    if (lfo2.active && recorder.active) droneY = 71;
    
    // Calculate display values
    int rootDisplay = (int)drone.rootFreq;
    int detuneDisplay = (int)(drone.detune * 100);
    
    // Redraw if state changed OR values changed
    if (drone.playMode != lastDronePlayMode || 
        rootDisplay != lastDroneRootDisplay || 
        detuneDisplay != lastDroneDetuneDisplay || 
        forceFullRedraw) {
        
        display.fillRect(5, droneY, 138, 10, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(5, droneY);
        display.setTextColor(COLOR_MOD);
        display.print("DRONE:");
        
        if (drone.playMode == 2) {
            display.setTextColor(COLOR_ACTIVE);  // Red when ON
            display.print("ON ");
        } else if (drone.playMode == 1) {
            display.setTextColor(COLOR_WAVE);    // Green when KEYS
            display.print("KEYS ");
        } else {
            display.setTextColor(COLOR_GRID);    // Gray when OFF
            display.print("OFF ");
        }
        
        // ALWAYS show values regardless of mode
        if (drone.playMode != 0) {
            // Show root freq
            display.print(rootDisplay);
            display.print("Hz ");
            // Show detune
            display.print(detuneDisplay);
            display.print("%");
        }
        
        lastDronePlayMode = drone.playMode;  // Track the actual mode
        lastDroneRootDisplay = rootDisplay;
        lastDroneDetuneDisplay = detuneDisplay;
    }
    
    lastDroneWasActive = true;
}

// When NOT in drone mode, clear the area (only once)
if (!sequencer.active && !drone.active && lastDroneWasActive) {
    display.fillRect(5, 47, 138, 10, COLOR_BG);
    display.fillRect(5, 59, 138, 10, COLOR_BG);
    display.fillRect(5, 71, 138, 10, COLOR_BG);
    lastDroneWasActive = false;
}

// Draw SIDECHAIN status (only when active)
static bool lastSidechainWasActive = false;
static bool lastSidechainIsOn = false;
static int lastSidechainDepthDisplay = -1;
static int lastSidechainAttackDisplay = -1;
static int lastSidechainReleaseDisplay = -1;

if (!sequencer.active && sidechain.active) {
    // Calculate Y position based on what else is active
    int sidechainY = 47;
    
    // Single mode active
    if (reverser.active) sidechainY = 59;
    if (infiniteSustain) sidechainY = 59;
    if (lfo2.active) sidechainY = 59;
    if (recorder.active) sidechainY = 59;
    if (drone.active) sidechainY = 59;
    
    // Two modes active
    if (reverser.active && infiniteSustain) sidechainY = 71;
    if (reverser.active && lfo2.active) sidechainY = 71;
    if (reverser.active && recorder.active) sidechainY = 71;
    if (reverser.active && drone.active) sidechainY = 71;
    if (infiniteSustain && lfo2.active) sidechainY = 71;
    if (infiniteSustain && recorder.active) sidechainY = 71;
    if (infiniteSustain && drone.active) sidechainY = 71;
    if (lfo2.active && recorder.active) sidechainY = 71;
    if (lfo2.active && drone.active) sidechainY = 71;
    if (recorder.active && drone.active) sidechainY = 71;
    
    // Three modes active - move to Y=83
    int numActive = 0;
    if (reverser.active) numActive++;
    if (infiniteSustain) numActive++;
    if (lfo2.active) numActive++;
    if (recorder.active) numActive++;
    if (drone.active) numActive++;
    if (numActive >= 2) sidechainY = 83;
    
    // Calculate display values
    int depthDisplay = (int)(sidechain.depth * 100);
    int attackDisplay = (int)sidechain.attackTime;
    int releaseDisplay = (int)sidechain.releaseTime;
    
    // Redraw if state changed OR values changed
    if (sidechain.enabled != lastSidechainIsOn || 
        depthDisplay != lastSidechainDepthDisplay || 
        attackDisplay != lastSidechainAttackDisplay ||
        releaseDisplay != lastSidechainReleaseDisplay ||
        forceFullRedraw) {
        
        display.fillRect(5, sidechainY, 150, 10, COLOR_BG);  // Wider to clear ghost text
        display.setTextSize(1);
        display.setCursor(5, sidechainY);
        display.setTextColor(COLOR_MOD);
        display.print("SIDECHAIN:");
        
        if (sidechain.enabled) {
            display.setTextColor(COLOR_ACTIVE);  // Red when ON
            display.print("ON ");
        } else {
            display.setTextColor(COLOR_GRID);  // Gray when OFF
            display.print("OFF ");
        }
        
        // ALWAYS show values (whether ON or OFF)
        display.print(depthDisplay);
        display.print("% ");
        display.print(attackDisplay);
        display.print("/");
        display.print(releaseDisplay);
        display.print("ms");
        
        lastSidechainIsOn = sidechain.enabled;
        lastSidechainDepthDisplay = depthDisplay;
        lastSidechainAttackDisplay = attackDisplay;
        lastSidechainReleaseDisplay = releaseDisplay;
    }
    
    lastSidechainWasActive = true;
}

// When NOT in sidechain mode, clear the area (only once)
if (!sequencer.active && !sidechain.active && lastSidechainWasActive) {
    display.fillRect(5, 47, 150, 10, COLOR_BG);  // Match wider clear area
    display.fillRect(5, 59, 150, 10, COLOR_BG);
    display.fillRect(5, 71, 150, 10, COLOR_BG);
    display.fillRect(5, 83, 150, 10, COLOR_BG);  // Add Y=83
    lastSidechainWasActive = false;
}

// Draw RECORD status directly under MUTATE (only when active) (Y=47 or Y=59)
static bool lastRecorderWasActive = false;

if (!sequencer.active && recorder.active) {
    // Draw RECORD - adjust position based on what else is active
    int recordY = 47;
    if (reverser.active) recordY = 59;
    if (infiniteSustain) recordY = 59;
    if (reverser.active && infiniteSustain) recordY = 71;
    
    // Always draw RECORD status when recorder is active
    display.fillRect(5, recordY, 120, 10, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(5, recordY);
    display.setTextSize(1);
    display.setCursor(5, 47);
    display.setTextColor(COLOR_MOD);
    display.print("RECORD:");
    display.setTextColor(COLOR_TEXT);
    
    if (recorder.recording) {
        display.setTextColor(COLOR_ACTIVE); // Red when recording
        display.print("REC");
        // Show time
        unsigned long elapsed = (millis() - recorder.recordStartTime) / 1000;
        display.print(" ");
        display.print(elapsed);
        display.print("s");
    } else if (recorder.playing) {
        display.setTextColor(COLOR_WAVE); // Green when playing
        display.print("PLAY");
    } else if (recorder.hasRecording) {
        display.print("READY");
    } else {
        display.setTextColor(COLOR_GRID);
        display.print("WAITING");
    }
    
    // Always draw LOOP status under RECORD when in record mode
    int loopY = 59;
    if (reverser.active) loopY = 71;
    if (infiniteSustain) loopY = 71;
    if (reverser.active && infiniteSustain) loopY = 83;
    display.fillRect(5, loopY, 120, 10, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(5, loopY);
    display.setTextColor(COLOR_MOD);
    display.print("LOOP:");
    display.setTextColor(recorder.loopEnabled ? COLOR_WAVE : COLOR_GRID);
    display.print(recorder.loopEnabled ? "ON" : "OFF");
    
    lastRecorderWasActive = true;
}

// When NOT in recorder mode, clear both Y=47 and Y=59 (only once)
if (!sequencer.active && !recorder.active && lastRecorderWasActive) {
    display.fillRect(5, 47, 120, 10, COLOR_BG);  // Clear RECORD area
    display.fillRect(5, 59, 120, 10, COLOR_BG);  // Clear recorder LOOP area
    lastRecorderWasActive = false;
}

// Draw LOOP pattern status (only when loop mode active) (Y=47)
static uint8_t lastLoopPattern = 255;
if (!sequencer.active && !recorder.active && loopState.active) {
    if (loopState.pattern != lastLoopPattern || forceFullRedraw) {
        display.fillRect(5, 47, 120, 10, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(5, 47);
        display.setTextColor(COLOR_MOD);
        display.print("LOOP:");
        display.setTextColor(COLOR_TEXT);
        display.print(loopPatternNames[loopState.pattern]);
        lastLoopPattern = loopState.pattern;
    }
} else if (!sequencer.active && !recorder.active && loopState.pattern == LOOP_OFF && lastLoopPattern != LOOP_OFF) {
    display.fillRect(5, 47, 120, 10, COLOR_BG);
    lastLoopPattern = LOOP_OFF;
}

// POT 1 Display (Y: 8) - TOP CENTER
int pot1Value = 0;
const char* pot1Label = "";
const char* pot1Suffix = "";
bool pot1IsSpecial = false; // Flag for special display handling

if (sequencer.active) {
    pot1Label = "SEQ:";
    pot1Value = sequencer.running ? -3 : -2; // -3 = RUNNING, -2 = STOPPED
    pot1Suffix = "";
    pot1IsSpecial = true;
} else if (reverser.active) {
    pot1Label = "PITCH:";
    pot1Value = reverser.pitchShift; // Can be -2, -1, 0, +1, +2
    pot1Suffix = " oct";
    pot1IsSpecial = true; // Need special handling for negative values
} else if (siren.instrumentType == INST_RAY_GUN) {
    pot1Label = "FREQ:";
    pot1Value = (int)siren.baseFreq;
    pot1Suffix = "Hz";
} else if (siren.instrumentType == INST_LEAD_SYNTH) {
    pot1Label = "ROOT:";
    pot1Value = (int)siren.baseFreq;
    pot1Suffix = "Hz";
} else if (siren.instrumentType == INST_DISCO) {
    pot1Label = "PITCH:";
    pot1Value = (int)siren.baseFreq;
    pot1Suffix = "Hz";
}else {
    pot1Label = "PITCH:";
    pot1Value = (int)siren.frequency;
    pot1Suffix = "Hz";
}

// Handle special modes (sequencer and reverse) differently
if (pot1IsSpecial) {
    bool valueChanged = (pot1Value != lastPot1Value);
    
    if (valueChanged || lastPot1Value == -999 || potForceRedraw) {
        display.fillRect(148, 8, 80, 8, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(148, 8);
        display.setTextColor(COLOR_MOD);
        display.print(pot1Label);
        display.setTextColor(COLOR_TEXT);
        
        if (sequencer.active) {
            // Sequencer mode: show RUNNING/STOPPED
            if (pot1Value == -3) {
                display.setTextColor(COLOR_ACTIVE); // Red for RUNNING
                display.print("RUNNING");
            } else {
                display.setTextColor(COLOR_GRID); // Gray for STOPPED
                display.print("STOPPED");
            }
        } else if (reverser.active) {
            // Reverse mode: show pitch shift with sign
            if (pot1Value > 0) {
                display.print("+");
            }
            display.print(pot1Value);
            display.print(pot1Suffix);
        }
        
        lastPot1Value = pot1Value;
    }
} else {
    // Normal mode: only redraw on significant change
    if (abs(pot1Value - lastPot1Value) > 5 || lastPot1Value == -999 || potForceRedraw) {
        display.fillRect(148, 8, 80, 8, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(148, 8);
        display.setTextColor(COLOR_MOD);
        display.print(pot1Label);
        display.setTextColor(COLOR_TEXT);
        display.print(pot1Value);
        display.print(pot1Suffix);
        lastPot1Value = pot1Value;
    }
}

// POT 2 Display (Y: 20) - TOP CENTER
int pot2Value = 0;
const char* pot2Label = "";
const char* pot2Suffix = "";

if (sequencer.active) {
    pot2Label = "TEMPO:";
    pot2Value = sequencer.stepInterval;
    pot2Suffix = "ms";
} else if (siren.instrumentType == INST_RAY_GUN) {
    pot2Label = "SWEEP:";
    pot2Value = (int)(siren.sweepSpeed * 10);
    pot2Suffix = "Hz";
} else if (siren.instrumentType == INST_LEAD_SYNTH) {
    pot2Label = "TEMPO:";
    pot2Value = siren.stepInterval;
    pot2Suffix = "ms";
} else if (siren.mode == MODE_PORTAMENTO) {
    pot2Label = "GLIDE:";
    pot2Value = (int)(siren.glideTime * 100);
    pot2Suffix = "";
} else if (siren.instrumentType == INST_DISCO) {
    pot2Label = "SPEED:";
    pot2Value = (int)(siren.discoCharacter * 100);
    pot2Suffix = "%";
}else {
    pot2Label = "SPEED:";
    pot2Value = (int)(siren.lfoRate * 10);
    pot2Suffix = "Hz";
}

if (abs(pot2Value - lastPot2Value) > 3 || lastPot2Value == -999 || potForceRedraw) {
    display.fillRect(148, 20, 80, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(148, 20);
    display.setTextColor(COLOR_MOD);
    display.print(pot2Label);
    display.setTextColor(COLOR_TEXT);
    display.print(pot2Value);
    display.print(pot2Suffix);
    lastPot2Value = pot2Value;
}

// POT 3 Display (Y: 28) - TOP CENTER
int pot3Value = 0;
const char* pot3Label = "";
const char* pot3Suffix = "";

if (sequencer.active) {
    pot3Label = "FILT:";
    pot3Value = sequencer.filterCutoff;
    pot3Suffix = "Hz";

} else if (reverser.active) {
    pot3Label = "MIX:";
    pot3Value = (int)(reverser.mixAmount * 100);
    pot3Suffix = "%";

} else if (siren.instrumentType == INST_RAY_GUN) {
    pot3Label = "RES:";
    pot3Value = (int)(siren.resonance * 100);
    pot3Suffix = "%";
} else if (siren.instrumentType == INST_LEAD_SYNTH) {
    pot3Label = "VIB:";
    pot3Value = (int)(siren.vibratoDepth * 100);
    pot3Suffix = "%";
} else if (siren.instrumentType == INST_DISCO) {
    pot3Label = "BRIGHT:";
    pot3Value = (int)(siren.discoBrightness * 100);
    pot3Suffix = "%";
} else if (siren.mode == MODE_DEEP_SUB) {
    pot3Label = "SUB:";
    pot3Value = (int)(siren.subMix * 100);
    pot3Suffix = "%";
} else if (siren.mode == MODE_LOFI_CRUSH) {
    pot3Label = "BITS:";
    pot3Value = siren.bitDepth;
    pot3Suffix = "b";
} else if (siren.mode == MODE_RING_MOD) {
    pot3Label = "RMOD:";
    pot3Value = (int)siren.ringModFreq;
    pot3Suffix = "Hz";
} else {
    pot3Label = "MOD:";
    pot3Value = (int)(siren.lfoDepth * 100);
    pot3Suffix = "%";
}

if (abs(pot3Value - lastPot3Value) > 3 || lastPot3Value == -999 || potForceRedraw) {
    display.fillRect(148, 32, 80, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(148, 32);
    display.setTextColor(COLOR_MOD);
    display.print(pot3Label);
    display.setTextColor(COLOR_TEXT);
    display.print(pot3Value);
    display.print(pot3Suffix);
    lastPot3Value = pot3Value;
}

    // Draw persistent labels every time (simple approach)
    display.setTextSize(1);
    display.setTextColor(COLOR_TEXT);
 
// ============================================================================
// PERSISTENT SUB-HEADINGS - MUST ALWAYS BE VISIBLE ON ALL SCREENS
// ============================================================================
static uint8_t lastGateValue = 255;
static int8_t lastOctValue = 127;
static uint8_t lastDelayIdx = 255;
static uint8_t lastInstrumentType = 255;
static bool lastWasSequencer = false;
static bool forceSubHeadingDraw = false;

// Force sub-heading redraw if full redraw requested
if (forceFullRedraw) {
    lastGateValue = 255;
    lastOctValue = 127;
    lastDelayIdx = 255;
    forceSubHeadingDraw = true;
}

// Get current values
uint8_t currentGate = sequencer.active ? sequencer.gateLength : siren.gateMode;
int8_t currentOct = sequencer.active ? sequencer.octaveShift : octaveShift;

// Detect if mode changed (instrument or sequencer)
bool modeChanged = (siren.instrumentType != lastInstrumentType) || (sequencer.active != lastWasSequencer);

// Force immediate draw on mode change OR if flag is set
if (modeChanged || forceSubHeadingDraw) {
    lastInstrumentType = siren.instrumentType;
    lastWasSequencer = sequencer.active;
    forceSubHeadingDraw = false;  // Clear the flag after drawing
    
    // Draw all three sub-headings immediately (don't wait for value changes)
    // GATE
    display.fillRect(245, 8, 75, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(245, 8);
    display.setTextColor(COLOR_MOD);
    display.print("GATE:");
    display.setTextColor(COLOR_TEXT);
    if (sequencer.active) {
        display.print(sequencer.gateLength);
        display.print("%");
    } else {
        display.print(gateModeNames[siren.gateMode]);
    }
    lastGateValue = currentGate;
    
    // OCTAVE
    display.fillRect(245, 20, 75, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(245, 20);
    display.setTextColor(COLOR_MOD);
    display.print("OCTAVE:");
    display.setTextColor(COLOR_TEXT);
    if (currentOct == 0) {
        display.print("0");
    } else if (currentOct > 0) {
        display.print("+");
        display.print(currentOct);
    } else {
        display.print(currentOct);
    }
    lastOctValue = currentOct;
    
    // DELAY
    display.fillRect(245, 32, 75, 8, COLOR_BG);
    display.setTextSize(1);
    display.setCursor(245, 32);
    display.setTextColor(COLOR_MOD);
    display.print("DELAY:");
    display.setTextColor(COLOR_TEXT);
    display.print(delayTimeNames[siren.delayIndex]);
    lastDelayIdx = siren.delayIndex;
}
// Otherwise, only redraw if values actually changed
else {
    // GATE - draw only if changed
    if (currentGate != lastGateValue) {
        display.fillRect(245, 8, 75, 8, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(245, 8);
        display.setTextColor(COLOR_MOD);
        display.print("GATE:");
        display.setTextColor(COLOR_TEXT);
        if (sequencer.active) {
            display.print(sequencer.gateLength);
            display.print("%");
        } else {
            display.print(gateModeNames[siren.gateMode]);
        }
        lastGateValue = currentGate;
    }

    // OCTAVE - draw only if changed
    if (currentOct != lastOctValue) {
        display.fillRect(245, 20, 75, 8, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(245, 20);
        display.setTextColor(COLOR_MOD);
        display.print("OCTAVE:");
        display.setTextColor(COLOR_TEXT);
        if (currentOct == 0) {
            display.print("0");
        } else if (currentOct > 0) {
            display.print("+");
            display.print(currentOct);
        } else {
            display.print(currentOct);
        }
        lastOctValue = currentOct;
    }

    // DELAY - draw only if changed
    if (siren.delayIndex != lastDelayIdx) {
        display.fillRect(245, 32, 75, 8, COLOR_BG);
        display.setTextSize(1);
        display.setCursor(245, 32);
        display.setTextColor(COLOR_MOD);
        display.print("DELAY:");
        display.setTextColor(COLOR_TEXT);
        display.print(delayTimeNames[siren.delayIndex]);
        lastDelayIdx = siren.delayIndex;
    }
}
}

void updateDisplay() {
    static unsigned long lastWaveUpdate = 0;
    static unsigned long lastParamUpdate = 0;
    static bool lastTriggered = false;
    static uint8_t lastDelayIndex = 0;
    static uint8_t lastMode = 0;
    static int lastFreq = 0;
    static int lastLFO = 0;
    static int lastMod = 0;
    static int lastFeedback = 0;
    static bool initialized = false;
    static bool lastSequencerActive = false;
    static bool lastRecorderActive = false;
    
    // Detect sequencer mode change - DON'T reinit, it's already been done by handleFunctionKey()
    if (sequencer.active != lastSequencerActive) {
        lastSequencerActive = sequencer.active;
        // Detect recorder mode change
    if (recorder.active != lastRecorderActive) {
        lastRecorderActive = recorder.active;
        
        if (recorder.active) {
            // Recorder just activated - force a full parameter redraw
            drawParameters(true);  // Force full redraw
            lastParamUpdate = millis();
        }
        
        // Update tracking variables
        lastFreq = (int)siren.frequency;
        lastLFO = (int)(siren.lfoRate * 10);
        lastMod = (int)(siren.lfoDepth * 100);
        lastFeedback = (int)(siren.echoFeedback * 100);
        lastTriggered = siren.triggered;
        lastDelayIndex = siren.delayIndex;
        lastMode = siren.mode;
        return;  // Exit early, let the forced redraw settle
    }
        // Don't set initialized = false here! 
        // The display was already set up by handleFunctionKey()
        
        // Just update the tracking variables
        lastFreq = (int)siren.frequency;
        lastLFO = (int)(siren.lfoRate * 10);
        lastMod = (int)(siren.lfoDepth * 100);
        lastFeedback = (int)(siren.echoFeedback * 100);
        lastTriggered = siren.triggered;
        lastDelayIndex = siren.delayIndex;
        lastMode = siren.mode;
        lastParamUpdate = millis();
        lastWaveUpdate = millis();
        return;  // Exit early, don't redraw anything
    }
    
    // Initialize display once (only on first startup)
    if (!initialized) {
        display.fillScreen(COLOR_BG);
        drawParameters();
        drawWaveform(true); // Force initial draw with grid
        initialized = true;
        lastFreq = (int)siren.frequency;
        lastLFO = (int)(siren.lfoRate * 10);
        lastMod = (int)(siren.lfoDepth * 100);
        lastFeedback = (int)(siren.echoFeedback * 100);
        lastTriggered = siren.triggered;
        lastDelayIndex = siren.delayIndex;
        lastMode = siren.mode;
        lastParamUpdate = millis();
        lastWaveUpdate = millis();
        return;
    }
    
    // Update waveform buffer
    waveBuffer[waveBufferPos] = lastSample;
    waveBufferPos = (waveBufferPos + 1) % WAVE_BUFFER_SIZE;
    
    // Check for trigger state change - update status LED
    if (siren.triggered != lastTriggered) {
        lastTriggered = siren.triggered;
    }
    
    // Check for mutation parameter changes - update immediately
    static uint8_t lastMutateMode = 255;
    static uint8_t lastMutateScale = 255;
    static uint8_t lastMutateProb = 255;
    static uint16_t lastMutateCutoff = 0;
    
    unsigned long now = millis();
    
    uint8_t currentProb = (uint8_t)(mutate.mutationProbability * 100);
    bool mutateParamChanged = (mutate.mode != lastMutateMode) ||
                              (mutate.mode == MUTATE_AMOUNT && currentProb != lastMutateProb) ||
                              (mutate.mode == MUTATE_RHYTHM && mutate.currentScale != lastMutateScale) ||
                              (mutate.mode == MUTATE_CUTOFF && siren.filterCutoff != lastMutateCutoff);

    // Redraw if mutation mode OR mutation value changed
    if (mutateParamChanged) {
        drawParameters();
        lastParamUpdate = now;
    }

    // Update tracking variables
    lastMutateMode = mutate.mode;
    lastMutateProb = currentProb;
    lastMutateScale = mutate.currentScale;
    lastMutateCutoff = siren.filterCutoff;
    
   // Redraw waveform more frequently to show parameter changes
    
    // Check if pitch changed significantly (use baseFreq for preview responsiveness)
    int currentPitch = (int)siren.baseFreq;
    bool pitchChanged = abs(currentPitch - lastFreq) > 5;
    
    // Update waveform much more frequently, especially when parameters change
    unsigned long waveInterval;
    if (!siren.triggered) {
        // When not triggered, update very fast to show pot changes
        waveInterval = pitchChanged ? 20 : 50; // 20ms when changing, 50ms when stable
    } else {
        waveInterval = 50; // Fast rate when triggered to show lead synth steps
    }
    
    if (now - lastWaveUpdate > waveInterval || pitchChanged) {
    drawWaveform(false); // Don't redraw grid
    lastWaveUpdate = now;
}
    
    // Check for significant parameter changes
int currentFreqDisplay;
if (siren.instrumentType == INST_DISCO) {
    currentFreqDisplay = (int)siren.baseFreq;  // DISCO uses baseFreq directly
} else {
    currentFreqDisplay = (int)siren.frequency;
}

// Check the appropriate speed parameter based on instrument OR sequencer
int currentSpeed;
if (sequencer.active) {
    currentSpeed = (int)sequencer.stepInterval;  // SEQUENCER uses stepInterval
} else if (siren.instrumentType == INST_RAY_GUN) {
    currentSpeed = (int)(siren.sweepSpeed * 10);  // Ray Gun uses sweepSpeed
} else if (siren.instrumentType == INST_LEAD_SYNTH) {
    currentSpeed = (int)siren.stepInterval;  // Lead Synth uses stepInterval
} else if (siren.instrumentType == INST_DISCO) {
    currentSpeed = (int)(siren.discoCharacter * 100);  // DISCO uses discoCharacter
} else {
    currentSpeed = (int)(siren.lfoRate * 10);  // Dub Siren uses lfoRate
}
    
    // Check modulation parameter based on mode
int currentMod;
if (sequencer.active) {
    currentMod = (int)sequencer.filterCutoff;  // SEQUENCER uses filterCutoff
} else if (siren.instrumentType == INST_DISCO) {
    currentMod = (int)(siren.discoBrightness * 100);  // DISCO uses discoBrightness
} else {
    currentMod = (int)(siren.lfoDepth * 100);
}
    int currentFeedback = (int)(siren.echoFeedback * 100);
    
    // Larger thresholds to reduce updates
    bool freqChanged = abs(currentFreqDisplay - lastFreq) > 10;
    bool speedChanged = abs(currentSpeed - lastLFO) > 2;  // Renamed from lfoChanged
    bool modChanged = abs(currentMod - lastMod) > 5;
    bool feedbackChanged = abs(currentFeedback - lastFeedback) > 5;
    bool delayChanged = (siren.delayIndex != lastDelayIndex);
    bool modeChanged = false;
    if (siren.instrumentType == INST_DUB_SIREN) {
        modeChanged = (siren.mode != lastMode);
    } else if (siren.instrumentType == INST_RAY_GUN) {
        modeChanged = (siren.rayGunMode != lastMode);
    } else if (siren.instrumentType == INST_LEAD_SYNTH) {
        modeChanged = (siren.leadSynthMode != lastMode);
    }
    
    bool paramsChanged = freqChanged || speedChanged || modChanged || feedbackChanged || delayChanged || modeChanged;
    
    // Update parameters only if changed AND sufficient time has passed (or mode changed)
    if (paramsChanged && ((now - lastParamUpdate > 200) || modeChanged)) {
        drawParameters();
        lastParamUpdate = now;
        lastFreq = currentFreqDisplay;
        lastLFO = currentSpeed;  // Now storing the correct speed value
        lastMod = currentMod;
        lastFeedback = currentFeedback;
        lastDelayIndex = siren.delayIndex;
        if (siren.instrumentType == INST_DUB_SIREN) {
            lastMode = siren.mode;
        } else if (siren.instrumentType == INST_RAY_GUN) {
            lastMode = siren.rayGunMode;
        } else if (siren.instrumentType == INST_LEAD_SYNTH) {
            lastMode = siren.leadSynthMode;
        }
    }
}
// =====================================================================
// CORE 0: AUDIO GENERATION
// =====================================================================

void setup1() {
    // Audio generation runs on Core 0
}

void loop1() {
    // === DECLARE SYNC STATE VARIABLES AT FUNCTION SCOPE ===
    static bool lastSyncState = false;
    static bool syncConnected = false;
    static unsigned long lastSyncActivityTime = 0;
    
    // Read trigger button with debouncing
    bool triggerReading = digitalRead(BTN_TRIGGER);
    
    // Debounce the trigger button
    if (triggerReading != lastTriggerReading) {
        lastTriggerDebounceTime = micros();
    }
    
    if ((micros() - lastTriggerDebounceTime) > 5000) { // 5ms debounce
        if (triggerReading != debouncedTriggerState) {
            debouncedTriggerState = triggerReading;
        }
    }
    
    lastTriggerReading = triggerReading;
    bool buttonTriggered = (debouncedTriggerState == LOW);

    // === READ SYNC INPUT ===
    bool syncInputHigh = (digitalRead(SYNC_IN) == HIGH);
    
    // Detect sync cable activity
    if (syncInputHigh != lastSyncState) {
        syncConnected = true;
        lastSyncActivityTime = millis();
    }
    
    // Timeout sync connection after 2 seconds of no activity
    if (millis() - lastSyncActivityTime > 2000) {
        syncConnected = false;
    }
    
    // Detect clean rising edge (LOW->HIGH transition)
    bool syncRisingEdge = (syncInputHigh && !lastSyncState);
    
    // Update last state at the END to ensure clean edge detection
    // This happens AFTER we check for rising edge
    
    // === DRONE MODE TRIGGER HANDLING ===
    if (drone.active) {
        // Button cycles through play modes: OFF -> KEYS -> ON
        static bool lastDroneTriggerState = false;
        
        if (buttonTriggered && !lastDroneTriggerState) {
            drone.playMode = (drone.playMode + 1) % 3;  // Cycle 0->1->2->0
            drone.fadeStartTime = millis();
            lastDronePlayMode = 255;  // Force display update
            forceDisplayUpdate = true;  // Signal UI core to update
            
            Serial.print("DRONE MODE: ");
            if (drone.playMode == 0) Serial.println("OFF");
            else if (drone.playMode == 1) Serial.println("KEYS");
            else Serial.println("ON");
        }
        lastDroneTriggerState = buttonTriggered;
        
        // Don't process normal trigger logic in drone mode
        siren.triggered = false;
        
    } else if (sequencer.active) {
        // === SEQUENCER MODE TRIGGER HANDLING ===
        // Button toggles sequencer on/off
        static bool lastSeqTriggerState = false;
        
        if (buttonTriggered && !lastSeqTriggerState) {
            sequencer.running = !sequencer.running;
            if (sequencer.running) {
                sequencer.lastStepTime = millis();
                sequencer.currentStep = 0;
                Serial.println("Sequencer STARTED");
            } else {
                Serial.println("Sequencer STOPPED");
            }
        }
        lastSeqTriggerState = buttonTriggered;
        
        // === SEQUENCER STEP ADVANCEMENT ===
        if (sequencer.running) {
            unsigned long currentTime = millis();
            bool advanceStep = false;
            
            // PRIORITY 1: Sync pulse - advances immediately on rising edge
            if (syncConnected && syncRisingEdge) {
                advanceStep = true;
                sequencer.lastStepTime = currentTime; // Reset internal timer
                Serial.print("SYNC PULSE -> ");
            }
            // PRIORITY 2: Internal timer (only when no sync connected)
            else if (!syncConnected && (currentTime - sequencer.lastStepTime >= sequencer.stepInterval)) {
                advanceStep = true;
                sequencer.lastStepTime = currentTime;
                Serial.print("TIMER -> ");
            }
            
            // Advance to next enabled step
            if (advanceStep) {
                // Store old step for debug
                uint8_t oldStep = sequencer.currentStep;
                
                // Calculate next step
                uint8_t nextStep = (sequencer.currentStep + 1) % 8;
                uint8_t stepsChecked = 0;
                
                // Skip disabled steps
                while (!sequencer.stepEnabled[nextStep] && stepsChecked < 8) {
                    nextStep = (nextStep + 1) % 8;
                    stepsChecked++;
                }
                
                if (sequencer.stepEnabled[nextStep]) {
                    sequencer.currentStep = nextStep;
                    sequencer.noteActive = true;
                    sequencer.noteStartTime = currentTime; 
                    Serial.print("Step ");
                    Serial.print(oldStep + 1);
                    Serial.print(" -> ");
                    Serial.println(sequencer.currentStep + 1);
                } else {
                    sequencer.noteActive = false;
                    Serial.println("All steps disabled!");
                }
            }
        }
        
        // Don't process normal trigger logic in sequencer mode
        siren.triggered = false;
        
    } else {
        // === NORMAL MODE (NON-SEQUENCER, NON-RECORDER) ===
        // Read keyboard state and frequency directly in audio loop for immediate response
        bool keyPressed = false;
        
      // FOR LEAD SYNTH: Constrain frequencies to 200-800Hz range
        if (siren.instrumentType == INST_LEAD_SYNTH) {
            static float lastLeadFreq = 0;
            float tempFreq = 0;
            
            // Check keys WITHOUT else-if (allows multiple key detection)
            if (!digitalRead(KEY_1)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[0]; }
            if (!digitalRead(KEY_2)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[1]; }
            if (!digitalRead(KEY_3) && !recorder.active) { keyPressed = true; tempFreq = keyboard.keyFrequencies[2]; }
            if (!digitalRead(KEY_4) && !reverser.active) { keyPressed = true; tempFreq = keyboard.keyFrequencies[3]; }
            if (!digitalRead(KEY_5) && !infiniteSustain) { keyPressed = true; tempFreq = keyboard.keyFrequencies[4]; }
            if (!digitalRead(KEY_6) && !lfo2.active) { keyPressed = true; tempFreq = keyboard.keyFrequencies[5]; }
            if (!digitalRead(KEY_7)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[6]; }
            if (!digitalRead(KEY_8)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[7]; }
            
            if (keyPressed && tempFreq > 0) {
                while (tempFreq > 800.0f) tempFreq /= 2.0f;
                while (tempFreq < 200.0f) tempFreq *= 2.0f;
                
                if (tempFreq != lastLeadFreq) {
                    siren.baseFreq = tempFreq;
                    siren.frequency = tempFreq;
                    generateMelody();
                    lastLeadFreq = tempFreq;
                }
            }
        }
        // FOR DISCO: Constrain frequencies to 100-2000Hz range
        else if (siren.instrumentType == INST_DISCO) {
            static float lastDiscoFreq = 0;
            float tempFreq = 0;
            
            // Check keys WITHOUT else-if
            if (!digitalRead(KEY_1)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[0]; }
            if (!digitalRead(KEY_2)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[1]; }
            if (!digitalRead(KEY_3) && !recorder.active) { keyPressed = true; tempFreq = keyboard.keyFrequencies[2]; }
            if (!digitalRead(KEY_4) && !reverser.active) { keyPressed = true; tempFreq = keyboard.keyFrequencies[3]; }
            if (!digitalRead(KEY_5) && !infiniteSustain) { keyPressed = true; tempFreq = keyboard.keyFrequencies[4]; }
            if (!digitalRead(KEY_6) && !lfo2.active) { keyPressed = true; tempFreq = keyboard.keyFrequencies[5]; }
            if (!digitalRead(KEY_7)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[6]; }
            if (!digitalRead(KEY_8)) { keyPressed = true; tempFreq = keyboard.keyFrequencies[7]; }
            
            if (keyPressed && tempFreq > 0) {
                // Constrain to disco-appropriate range
                while (tempFreq > 2000.0f) tempFreq /= 2.0f;
                while (tempFreq < 100.0f) tempFreq *= 2.0f;
                
                if (tempFreq != lastDiscoFreq) {
                    siren.baseFreq = tempFreq;
                    siren.frequency = tempFreq;
                    lastDiscoFreq = tempFreq;
                }
            }
        }
        // FOR OTHER INSTRUMENTS: Use full range
        else {
            // Check keys WITHOUT else-if - priority to last pressed
            if (!digitalRead(KEY_1)) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[0]; }
            if (!digitalRead(KEY_2)) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[1]; }
            if (!digitalRead(KEY_3) && !recorder.active) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[2]; }
            if (!digitalRead(KEY_4) && !reverser.active) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[3]; }
            if (!digitalRead(KEY_5) && !infiniteSustain) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[4]; }
            if (!digitalRead(KEY_6) && !lfo2.active) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[5]; }
            if (!digitalRead(KEY_7)) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[6]; }
            if (!digitalRead(KEY_8)) { keyPressed = true; siren.baseFreq = keyboard.keyFrequencies[7]; }
            
            if (keyPressed) {
                siren.frequency = siren.baseFreq;
            }
        }

        // === TRIGGER PRIORITY SYSTEM ===
        // 1. Keyboard (manual keys) - highest priority
        // 2. Button trigger
        // 3. Sync input - lowest priority
        
        // === TRIGGER PRIORITY SYSTEM ===
        // 1. Keyboard (manual keys) - highest priority
        // 2. Button trigger
        // 3. Sync input - lowest priority
        
        if (keyPressed) {
            // Keyboard trigger - has priority over everything
            bool patternGate = evaluateLoopPattern();
            siren.triggered = patternGate;
            siren.gateActive = false;
            
            // DISCO: Retrigger on EVERY key press or frequency change
            static bool lastKeyPressed = false;
            static float lastDiscoKeyFreq = 0;
            if (siren.instrumentType == INST_DISCO) {
                // Retrigger if new key press OR different key
                if (!lastKeyPressed || abs(siren.baseFreq - lastDiscoKeyFreq) > 5.0f) {
                    siren.discoStartTime = millis();
                    siren.discoPhase = 0;
                    lastDiscoKeyFreq = siren.baseFreq;
                }
            }
            
            // Reset pattern timing on new key press
            if (!lastKeyPressed) {
                loopState.patternStartTime = millis();
                
                // TRIGGER MUTATION on new key press (mutation uses last active mode, even when OFF)
                static uint8_t lastActiveMutateMode1 = MUTATE_OFF;
                if (mutate.mode != MUTATE_OFF) {
                    lastActiveMutateMode1 = mutate.mode;
                }
                
                // Trigger if we've ever had mutation active
                if (lastActiveMutateMode1 != MUTATE_OFF) {
                    triggerMutation();
                }
            }
            
            // TRIGGER MUTATION on loop pattern gate edges (when loop mode active)
            // Mutations continue even when MUTATE is OFF (using locked parameter values)
            static uint8_t lastMutateModeSeen = MUTATE_OFF;
            if (mutate.mode != MUTATE_OFF) {
                lastMutateModeSeen = mutate.mode;
            }
            
            if (loopState.active && loopState.pattern != LOOP_OFF && lastMutateModeSeen != MUTATE_OFF) {
                static bool lastLoopGateState = false;
                bool currentLoopGate = evaluateLoopPattern();
                
                // Trigger mutation on rising edge of gate (when gate turns ON)
                if (currentLoopGate && !lastLoopGateState) {
                    triggerMutation();
                }
                
                lastLoopGateState = currentLoopGate;
            }
            
            lastKeyPressed = true;
        }
        else if (buttonTriggered) {
            // Button trigger - works when no keyboard pressed
            bool patternGate = evaluateLoopPattern();
            siren.triggered = patternGate;
            siren.gateActive = false;
            
            // Reset disco start time for retriggerable effects
            if (siren.instrumentType == INST_DISCO) {
                siren.discoStartTime = millis();
                siren.discoPhase = 0;
            }
            
            // Reset pattern timing on new button press
            static bool lastButtonTriggered = false;
            if (!lastButtonTriggered) {
                loopState.patternStartTime = millis();
                
                // TRIGGER MUTATION on new button press (mutation uses last active mode, even when OFF)
                static uint8_t lastActiveMutateMode2 = MUTATE_OFF;
                if (mutate.mode != MUTATE_OFF) {
                    lastActiveMutateMode2 = mutate.mode;
                }
                
                // Trigger if we've ever had mutation active
                if (lastActiveMutateMode2 != MUTATE_OFF) {
                    triggerMutation();
                }
            }
            
            // TRIGGER MUTATION on loop pattern gate edges (when loop mode active)
            // Mutations continue even when MUTATE is OFF (using locked parameter values)
            static uint8_t lastMutateModeSeen2 = MUTATE_OFF;
            if (mutate.mode != MUTATE_OFF) {
                lastMutateModeSeen2 = mutate.mode;
            }
            
            if (loopState.active && loopState.pattern != LOOP_OFF && lastMutateModeSeen2 != MUTATE_OFF) {
                static bool lastLoopGateStateBtn = false;
                bool currentLoopGate = evaluateLoopPattern();
                
                // Trigger mutation on rising edge of gate (when gate turns ON)
                if (currentLoopGate && !lastLoopGateStateBtn) {
                    triggerMutation();
                }
                
                lastLoopGateStateBtn = currentLoopGate;
            }
            
            lastButtonTriggered = true;
        }
        else {
            // Sync trigger - only when nothing else pressed
            if (syncConnected && syncRisingEdge) {
                // Rising edge of sync - start gate
                siren.gateStartTime = millis();
                siren.gateActive = true;
                
                // Set triggered for all instruments
                siren.triggered = true;
                
                // Reset disco phase and start time on sync trigger
                if (siren.instrumentType == INST_DISCO) {
                    siren.discoPhase = 0;
                    siren.discoStartTime = millis();
                }
                
                // Reset loop pattern timing to sync with external clock
                if (loopState.pattern != LOOP_OFF) {
                    loopState.patternStartTime = millis();
                }
                
                // Always trigger mutation (uses last active mode, even when OFF)
                static uint8_t lastActiveMutateMode3 = MUTATE_OFF;
                if (mutate.mode != MUTATE_OFF) {
                    lastActiveMutateMode3 = mutate.mode;
                }
                
                // Trigger if we've ever had mutation active
                if (lastActiveMutateMode3 != MUTATE_OFF) {
                    Serial.print("MUTATE: mode=");
                    Serial.print(mutationModeNames[lastActiveMutateMode3]);
                    Serial.print(" baseFreq before=");
                    Serial.print(siren.baseFreq);
                    
                    triggerMutation();
                    
                    Serial.print(" after=");
                    Serial.println(siren.baseFreq);
                }
                // === SIDECHAIN TRIGGER (on sync) ===
            if (sidechain.enabled && syncConnected && syncRisingEdge) {
                sidechain.triggered = true;
                sidechain.lastTriggerTime = millis();
            }
            }
            
            // Handle gate timing for sync input
            if (siren.gateActive) {
                unsigned long elapsed = millis() - siren.gateStartTime;
                
                // ALL INSTRUMENTS: Use the gate time setting (Button 4)
                unsigned long gateLength = gateTimesMs[siren.gateMode];

                if (elapsed >= gateLength) {
                    siren.gateActive = false;
                    siren.triggered = false; // Turn off for all instruments
                }
                
                // Update triggered based on gate for all instruments
                siren.triggered = siren.gateActive;
            } else {
                // Gate not active
                siren.triggered = false;
            }
        }
    }
    
    // UPDATE SYNC STATE AFTER ALL LOGIC
    lastSyncState = syncInputHigh;

// UPDATE SYNC STATE AFTER ALL LOGIC
    lastSyncState = syncInputHigh;
    
    // === SIDECHAIN ENVELOPE CALCULATION ===
    if (sidechain.active && sidechain.enabled) {
        // Check if any keyboard key is pressed (directly read GPIO)
        bool anyKeyPressed = (!digitalRead(KEY_1) || !digitalRead(KEY_2) || 
                              !digitalRead(KEY_3) || !digitalRead(KEY_4) ||
                              !digitalRead(KEY_5) || !digitalRead(KEY_6) ||
                              !digitalRead(KEY_7) || !digitalRead(KEY_8));
        
        // Debug output (limit rate)
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime > 1000) {
            Serial.print("SIDECHAIN: env=");
            Serial.print(sidechain.envelope, 3);
            Serial.print(" trig=");
            Serial.print(sidechain.triggered);
            Serial.print(" depth=");
            Serial.print(sidechain.depth, 2);
            Serial.print(" attack=");
            Serial.print(sidechain.attackTime);
            Serial.print("ms release=");
            Serial.print(sidechain.releaseTime);
            Serial.println("ms");
            lastDebugTime = millis();
        }
        
        // Direct button read (not buttonTriggered variable)
        bool btnPressed = (digitalRead(BTN_TRIGGER) == LOW);
        
        // Trigger on sync (highest priority)
        bool shouldTrigger = false;
        if (syncConnected && syncRisingEdge) {
            shouldTrigger = true;
            Serial.println("SIDECHAIN TRIGGERED (sync)");
        }
        // Trigger on button/key press (with edge detection)
        else if (!syncConnected) {
            static bool lastBtnOrKeyState = false;
            bool btnOrKeyPressed = (btnPressed || anyKeyPressed);
            
            if (btnOrKeyPressed && !lastBtnOrKeyState) {
                shouldTrigger = true;
                Serial.println("SIDECHAIN TRIGGERED (button/key)");
            }
            lastBtnOrKeyState = btnOrKeyPressed;
            
            // Also check loop pattern gates
            if (loopState.active && loopState.pattern != LOOP_OFF) {
                static bool lastLoopGateForSidechain = false;
                bool currentLoopGate = evaluateLoopPattern();
                
                if (currentLoopGate && !lastLoopGateForSidechain) {
                    shouldTrigger = true;
                    Serial.println("SIDECHAIN TRIGGERED (loop gate)");
                }
                lastLoopGateForSidechain = currentLoopGate;
            }
        }
        
        // AUTO-RETRIGGER for continuous pumping
        if (!sidechain.triggered) {
            unsigned long timeSince = millis() - sidechain.lastTriggerTime;
            unsigned long cycleDuration = (unsigned long)(sidechain.attackTime + sidechain.releaseTime);
            
            if (timeSince >= cycleDuration) {
                shouldTrigger = true;
                Serial.println("SIDECHAIN AUTO-RETRIGGER");
            }
        }
        
        // Apply trigger
        if (shouldTrigger) {
            sidechain.triggered = true;
            sidechain.lastTriggerTime = millis();
        }
        
        // Calculate envelope
        if (sidechain.triggered) {
            unsigned long elapsed = millis() - sidechain.lastTriggerTime;
            float elapsedMs = elapsed;
            
            // Attack phase (fast duck down)
            if (elapsedMs < sidechain.attackTime) {
                float attackProgress = elapsedMs / sidechain.attackTime;
                sidechain.envelope = 1.0f - (sidechain.depth * attackProgress);
            }
            // Release phase (slow return to full volume)
            else {
                float releaseElapsed = elapsedMs - sidechain.attackTime;
                if (releaseElapsed < sidechain.releaseTime) {
                    float releaseProgress = releaseElapsed / sidechain.releaseTime;
                    sidechain.envelope = (1.0f - sidechain.depth) + (sidechain.depth * releaseProgress);
                } else {
                    // Finished - back to full volume
                    sidechain.envelope = 1.0f;
                    sidechain.triggered = false;
                }
            }
            
            // Clamp envelope
            if (sidechain.envelope < (1.0f - sidechain.depth)) sidechain.envelope = (1.0f - sidechain.depth);
            if (sidechain.envelope > 1.0f) sidechain.envelope = 1.0f;
        } else {
            // Not triggered - full volume
            sidechain.envelope = 1.0f;
        }
    }

   // MUTATE BUTTON - instant response with double-tap detection
bool mutateReading = digitalRead(BTN_MUTATE);
static bool lastMutateReadingInLoop1 = HIGH;
static bool firstMutateRead = true;
static unsigned long lastMutatePress = 0;

// Skip first read to avoid false trigger on startup
if (firstMutateRead) {
    lastMutateReadingInLoop1 = mutateReading;
    firstMutateRead = false;
} else {
    if (mutateReading == LOW && lastMutateReadingInLoop1 == HIGH) {
        // Button just pressed - check for double-tap
        unsigned long now = millis();
        
        if (now - lastMutatePress < 500) {
            // Double-tap detected - toggle mutation execution
            mutate.fullyDisabled = !mutate.fullyDisabled;
            
            if (mutate.fullyDisabled) {
                Serial.println("=== MUTATE DISABLED (double-tap) ===");
                Serial.println("Mutations stopped. Double-tap again to re-enable.");
            } else {
                Serial.println("=== MUTATE ENABLED (double-tap) ===");
            }
            
            lastMutatePress = 0; // Reset timer
        } else {
            // Single press - cycle mutation mode
            uint8_t oldMode = mutate.mode;
            mutate.mode = (mutate.mode + 1) % MUTATE_MODE_COUNT;
            
            // Re-enable mutations if they were fully disabled
            if (mutate.fullyDisabled && mutate.mode != MUTATE_OFF) {
                mutate.fullyDisabled = false;
                Serial.println("=== MUTATE RE-ENABLED (via MUTATE button) ===");
            }
            
            // LOCK the parameter we're leaving behind
            if (oldMode == MUTATE_AMOUNT) {
                mutate.probabilityLocked = true;
                Serial.println("LOCK: Probability parameter locked at current value");
            } else if (oldMode == MUTATE_RHYTHM) {
                mutate.scaleLocked = true;
                Serial.println("LOCK: Scale parameter locked at current value");
            } else if (oldMode == MUTATE_CUTOFF) {
                mutate.cutoffLocked = true;
                Serial.println("LOCK: Cutoff parameter locked at current value");
            }
            
            lastMutatePress = now; // Record time for double-tap detection
        }
    }
    lastMutateReadingInLoop1 = mutateReading;
}

// SIDECHAIN KEY 8 - instant response in audio loop (same as MUTATE button)
    if (sidechain.active) {
        bool key8Reading = digitalRead(KEY_8);
        static bool lastKey8ReadingInLoop1 = HIGH;
        static bool firstKey8Read = true;
        
        // Skip first read to avoid false trigger on startup
        if (firstKey8Read) {
            lastKey8ReadingInLoop1 = key8Reading;
            firstKey8Read = false;
        } else {
            if (key8Reading == LOW && lastKey8ReadingInLoop1 == HIGH) {
                // Key just pressed - instant response
                sidechain.enabled = !sidechain.enabled;
                forceDisplayUpdate = true;  // Force UI update
                Serial.print("SIDECHAIN ");
                Serial.println(sidechain.enabled ? "ON" : "OFF");
            }
            lastKey8ReadingInLoop1 = key8Reading;
        }
    }
    
    // LOOP MODE KEY 2 - instant response in audio loop
    if (loopState.active) {
        static bool lastKey2StateInLoop1 = HIGH;
        bool key2Reading = digitalRead(KEY_2);
        
        if (key2Reading == LOW && lastKey2StateInLoop1 == HIGH) {
            // Key 2 just pressed - instant response
            loopState.pattern = (loopState.pattern + 1) % LOOP_COUNT;
            loopState.patternStartTime = millis();
            
            Serial.print("LOOP Pattern: ");
            Serial.println(loopPatternNames[loopState.pattern]);
        }
        
        lastKey2StateInLoop1 = key2Reading;
    }
    
    int16_t sample = generateSample();
    uint8_t pwmValue = (sample >> 8) + 128;
    pwm_set_gpio_level(AUDIO_PIN, pwmValue);
    delayMicroseconds(45); // ~22050 Hz
}

// =====================================================================
// CORE 1: UI AND CONTROLS
// =====================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== RP2040 DUB SIREN ===\n");
    
    // Allocate recording buffer
    recorder.buffer = (int16_t*)malloc(MAX_RECORD_SAMPLES * sizeof(int16_t));
    if (recorder.buffer == nullptr) {
        Serial.println("ERROR: Failed to allocate recording buffer!");
    } else {
        Serial.print("Recording buffer allocated: ");
        Serial.print(MAX_RECORD_SAMPLES);
        Serial.println(" samples (5 seconds)");
    }

    // Allocate reverse buffer
    reverser.buffer = (int16_t*)malloc(MAX_REVERSE_SAMPLES * sizeof(int16_t));
    if (reverser.buffer == nullptr) {
        Serial.println("ERROR: Failed to allocate reverse buffer!");
    } else {
        Serial.print("Reverse buffer allocated: ");
        Serial.print(MAX_REVERSE_SAMPLES);
        Serial.println(" samples (1 second)");
        // Initialize buffer to silence
        for (int i = 0; i < MAX_REVERSE_SAMPLES; i++) {
            reverser.buffer[i] = 0;
        }
    }
    
    // Initialize ADC
    adc_init();
    adc_gpio_init(POT_PITCH);
    adc_gpio_init(POT_SPEED);
    adc_gpio_init(POT_MOD);
    adc_gpio_init(POT_FEEDBACK);
    
    // Initialize buttons
    pinMode(BTN_TRIGGER, INPUT_PULLUP);
    pinMode(BTN_DELAY, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);
    pinMode(BTN_FUNCTION, INPUT_PULLUP);
    pinMode(BTN_MENU, INPUT_PULLUP);
    pinMode(BTN_MUTATE, INPUT_PULLUP);
    pinMode(BTN_OCTAVE, INPUT_PULLUP);
    pinMode(BTN_FUNC_SHIFT, INPUT_PULLUP);
    
    // Initialize keyboard pins
    pinMode(KEY_1, INPUT_PULLUP);
    pinMode(KEY_2, INPUT_PULLUP);
    pinMode(KEY_3, INPUT_PULLUP);
    pinMode(KEY_4, INPUT_PULLUP);
    pinMode(KEY_5, INPUT_PULLUP);
    pinMode(KEY_6, INPUT_PULLUP);
    pinMode(KEY_7, INPUT_PULLUP);
    pinMode(KEY_8, INPUT_PULLUP);
    
    // Initialize sync input with pulldown to prevent floating when disconnected
    pinMode(SYNC_IN, INPUT_PULLDOWN);
    
    // Initialize echo buffer
    for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
        echoBuffer[i] = 0;
    }
    
    // Setup PWM for audio
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(AUDIO_PIN);
    pwm_set_wrap(slice_num, PWM_WRAP);
    pwm_set_clkdiv(slice_num, 1.0f);
    pwm_set_enabled(slice_num, true);
    pwm_set_gpio_level(AUDIO_PIN, 128);
    
    // Setup display
    SPI.setTX(TFT_MOSI);
    SPI.setSCK(TFT_SCK);
    SPI.begin();
    display.init(170, 320);
    display.setRotation(3);
    display.fillScreen(COLOR_BG);
    
    // Initialize keyboard frequencies ONCE at startup
    updateKeyboardFrequencies();
    
    // Initial display will be drawn in updateDisplay()
    
    
    Serial.println("Controls:");
    Serial.println("- Pot 1: Pitch (50-2000Hz)");
    Serial.println("- Pot 2: LFO Speed / Glide Time (mode dependent)");
    Serial.println("- Pot 3: Modulation / Sub Mix / Bit Depth (mode dependent)");
    Serial.println("- Pot 4: Echo Feedback (0-95%)");
    Serial.println("- Button MENU (GP0): Cycle Instruments (DUB SIREN <-> RAY GUN)");
    Serial.println("- Button 1 (GP3): Hold to Trigger Sound");
    Serial.println("- Button 2 (GP5): Cycle Delay Time (OFF/50/100/175/250/375ms)");
    Serial.println("- Button 3 (GP4): Cycle Synthesis Mode");
    Serial.println("- Button 4 (GP1): Cycle Gate Length (SHORT/MED/LONG/X-LNG)");
    Serial.println("- Sync In (GP2): External 3-5V trigger");
    Serial.println("- Button MUTATE (GP9): Cycle Mutation Mode (OFF/PITCH/PITCH+SPD/ALL)");
    Serial.println("\n=== SPECIAL FUNCTIONS ===");
    Serial.println("- Function + Key 1: Activate SEQUENCER mode");
    Serial.println("- Function + Key 2: Cycle LOOP patterns");
    Serial.println("- Function + Key 3: Activate RECORD mode");
    Serial.println("- Function + Key 4: Activate REVERSE mode");
    Serial.println("- Function + Key 7: Activate DRONE mode (multi-oscillator synth)");
    Serial.println("- Function + Key 5: Toggle INFINITE HOLD");
    Serial.println("  * Press Key 3 to START recording");
    Serial.println("  * Press Key 3 to STOP recording");
    Serial.println("  * Press Key 3 to PLAY recording");
    Serial.println("  * Press Key 4 to toggle LOOP on/off");
    Serial.println("  * Double-press Function to EXIT and clear");
    Serial.println("=== INSTRUMENTS ===");
    Serial.println("1. DUB SIREN - Classic reggae/dub swoops");
    Serial.println("2. RAY GUN - Sci-fi laser zaps");
    Serial.println("3. LEAD SYNTH - Generative melodic sequences");
    Serial.println("4. DISCO - Classic disco sound effects");
    Serial.println("\n=== DUB SIREN Modes:");
    Serial.println("\nModes:");
    Serial.println("1. CLASSIC DUB - Sawtooth with LFO");
    Serial.println("2. DEEP SUB - Sub-octave bass");
    Serial.println("3. SQUARE WAVE - Hollow aggressive tone");
    Serial.println("4. LO-FI CRUSH - Bit crusher effect");
    Serial.println("5. RING MOD - Metallic ring modulation");
    Serial.println("6. PORTAMENTO - Smooth pitch glide\n");
    Serial.println("=== RAY GUN Modes:");
    Serial.println("1. ZAP - Quick upward sweep");
    Serial.println("2. LASER - Downward sweep with tail");
    Serial.println("3. BLASTER - Short burst");
    Serial.println("4. PHASER - Modulated pulse\n");
    Serial.println("=== LEAD SYNTH Modes:");
    Serial.println("1. SEQUENCE - Pre-programmed melody");
    Serial.println("2. ARPEGGIO - Arpeggiator with octave jumps");
    Serial.println("3. EUCLIDEAN - Euclidean rhythm patterns");
    Serial.println("4. GENERATIVE - Auto-generated melodies\n");
}

void loop() {
    readButtons();        // Read buttons FIRST
    readKeyboard();       // Read keyboard keys
    readPotentiometers(); // Then read pots (which checks button hold state)
    
    // Check if audio core requested display update
    if (forceDisplayUpdate) {
        drawParameters();  // Force immediate update
        forceDisplayUpdate = false;  // Clear flag
    }
    
    updateDisplay();
    
    // No delay - maximum responsiveness for trigger button
}