# Medusa: user manual

This manual tells you how to play Medusa. For the hardware, the build and the firmware download, see the [README](../README.md).

## Contents

- [Controls](#controls)
- [Instruments](#instruments)
- [Mutate](#mutate)
- [Special functions](#special-functions)

---

## Controls

### Knobs

| Knob | Function |
|------|----------|
| Pot 1 (Pitch) | Sets the base frequency (note). |
| Pot 2 (Speed) | Sets how fast the sound moves (LFO speed or tempo). |
| Pot 3 (Modulation) | Sets the effect intensity (vibrato, filter and others). |
| Pot 4 (Echo Feedback) | Sets how much the echo repeats. |

In some instruments and functions, the knobs have different tasks. The tables below give these tasks.

### Buttons

| Button | Function |
|--------|----------|
| TRIGGER | Hold to play the sound. |
| MENU | Selects the next instrument: Dub Siren → Ray Gun → Lead Synth → Disco. |
| MODE | Selects the next sound variation of the instrument. |
| DELAY | Selects the next delay time: OFF → 50 ms → 100 ms → 175 ms → 250 ms → 375 ms. |
| GATE | Changes the gate length (how long each note plays). |
| OCTAVE | Moves the pitch up or down by one octave. |
| MUTATE | Selects the next mutation setting. See [Mutate](#mutate). |
| FUNCTION | Used with keys 1 to 8 to start the special functions. See [Special functions](#special-functions). |

### Keys 1 to 8

- Each key plays one note of a musical scale, as on a small keyboard.
- The keys also start the special functions (with FUNCTION).

---

## Instruments

Push **MENU** to select an instrument. Push **MODE** to select a variation.

### 1. Dub Siren (classic reggae and dub sweeps)

| Knob | Function | Range |
|------|----------|-------|
| Pot 1 (PITCH) | Base frequency. Turn to change the note. | 50 Hz to 2000 Hz |
| Pot 2 (SPEED) | LFO speed: how fast the sound wobbles | 0.1 Hz to 20 Hz |
| Pot 3 (MOD) | Modulation depth: how much the pitch sweeps | 0 % to 100 % |
| Pot 4 (Echo Feedback) | Echo repeats, for dub delays | 0 % to 95 % |

| Mode | Sound |
|------|-------|
| CLASSIC DUB | Smooth sawtooth wobble |
| DEEP SUB | Adds a sub-bass octave. Pot 3 sets the sub-bass level. |
| SQUARE WAVE | Hollow, aggressive tone |
| LO-FI CRUSH | Bit-crusher effect. Pot 3 sets the bit depth (1 to 8). |
| RING MOD | Metallic ring modulation. Pot 3 sets the ring modulation frequency. |
| PORTAMENTO | Smooth pitch glide. Pot 2 sets the glide time. |

### 2. Ray Gun (science-fiction laser sounds)

| Knob | Function | Range |
|------|----------|-------|
| Pot 1 (FREQ) | Laser frequency: the pitch of the laser | 200 Hz to 4000 Hz |
| Pot 2 (SWEEP) | Sweep speed: how fast it sweeps | 0.1 Hz to 20 Hz |
| Pot 3 (RES) | Resonance: filter sharpness and brightness | 30 % to 95 % |
| Pot 4 (Echo Feedback) | Echo repeats | 0 % to 95 % |

| Mode | Sound |
|------|-------|
| ZAP | Quick upward sweep (classic ray gun) |
| LASER | Downward sweep with a noise burst |
| BLASTER | Rapid short bursts |
| PHASER | Modulated pulse |

### 3. Lead Synth (generative melodies)

| Knob | Function | Range |
|------|----------|-------|
| Pot 1 (ROOT) | Root note: the base pitch of the melody | 200 Hz to 800 Hz |
| Pot 2 (TEMPO) | Step time: how fast the notes play | 100 ms to 1000 ms |
| Pot 3 (VIB) | Vibrato depth: pitch wobble on the notes | 0 % to 10 % |
| Pot 4 (Gate) | Gate length: how long each note plays | 20 % to 100 % |

| Mode | Sound |
|------|-------|
| SEQUENCE | Plays a melody that is in the firmware |
| ARPEGGIO | Arpeggiator with octave jumps |
| EUCLIDEAN | Euclidean rhythm patterns |
| GENERATIVE | Makes new melodies automatically |

### 4. Disco (classic disco effects)

| Knob | Function | Range |
|------|----------|-------|
| Pot 1 (PITCH) | Base frequency: the pitch of the effect | 100 Hz to 2000 Hz |
| Pot 2 (SPEED) | Effect duration: how long the effect lasts | — |
| Pot 3 (BRIGHT) | Brightness: filter cutoff and tone | 0 % to 100 % |
| Pot 4 (Echo Feedback) | Echo repeats | 0 % to 95 % |

| Mode | Sound |
|------|-------|
| ORCH HIT | Punchy orchestral stab |
| STRINGS | Rising string-machine sweep |
| FUNK BLAST | Synth brass stab with pitch bend |
| WHOOSH | Noise sweep (a spaceship that flies past) |
| BUBBLE | Percussive bubble sound |
| LASER | Musical laser sweep with vibrato |

---

## Mutate

Mutate makes the sound change at random on each sync input trigger.

- Mutate changes only the pitch. The pitch knob controls it.
- Mutate operates only with a signal on the sync input.

Push **MUTATE** to select the next setting:

1. **Mutation probability:** 0 % to 100 %.
2. **Rhythm:** selects the musical scale: MIN PENT → MAJOR → BLUES → MINOR.
3. **Cutoff:** filter cutoff, 200 Hz to 4000 Hz.

When you set a value, it stays locked.

- To turn mutate off, push **MUTATE** two times quickly.
- To turn mutate on again, push **MUTATE** one time.

---

## Special functions

Hold **FUNCTION** and push a key to start a special function.

To turn off all functions, push **FUNCTION** two times quickly.

### Sequencer (FUNCTION + key 1)

An 8-step sequence that you can program.

- Push a key to turn its step on or off.
- Hold a key and turn Pot 1 to set the pitch of that step.
- Push **TRIGGER** to start or stop the sequence.

### Record (FUNCTION + key 3)

1. Push key 3 to start the recording (maximum 5 seconds).
2. Push key 3 again to stop the recording.
3. Push key 3 again to play the recording.
4. Push key 4 to turn the loop on or off.

### Reverse (FUNCTION + key 4)

- Push key 4 to turn the reverse effect on or off.
- Pot 1 sets the pitch shift.
- Pot 3 sets the wet and dry mix.

### Infinite delay (FUNCTION + key 5)

- Push key 5 to turn infinite sustain on or off.
- Pot 1 sets the feedback (95 % to 100 %) when sustain is on.
- Pot 3 sets the filter cutoff.

### LFO 2 (FUNCTION + key 6)

A second LFO that changes the frequency, for vibrato and tremolo effects.

- Push key 6 to turn LFO 2 on or off.
- Pot 1 sets the LFO 2 rate (0.1 Hz to 20 Hz).
- Pot 3 sets the LFO 2 depth (0 % to 100 %).

### Drone (FUNCTION + key 7)

An additive drone synth with 5 oscillators, with drift and detune.

- Push **TRIGGER** to select the next drone setting: OFF → KEYS → ON.
- Pot 1 sets the root pitch (20 Hz to 400 Hz).
- Pot 2 sets the detune (0 % to 100 %): the spread between the oscillators.
- Pot 3 sets the brightness (0 % to 100 %): the tone, from dark to bright.
- Pot 4 sets the drift speed (0.1 Hz to 5 Hz): a slow LFO.

Push **MODE** to select the next chord:

| Chord | Notes |
|-------|-------|
| UNISON | All oscillators at the same pitch |
| OCTAVE | Root and octaves |
| FIFTH | Root and perfect fifth |
| MAJOR | Root, major third and fifth |
| MINOR | Root, minor third and fifth |

### Sidechain (FUNCTION + key 8)

A classic "pumping" sidechain compression effect.

- Push key 8 to turn sidechain on or off.
- Pot 1 sets the ducking depth (0 % to 100 %): how much the volume decreases.
- Pot 2 sets the attack time (1 ms to 50 ms): how fast the volume decreases.
- Pot 3 sets the release time (50 ms to 500 ms): how fast the volume comes back.

The sidechain follows the loop patterns or the sync input, for a rhythmic pump. It starts again automatically, to make a continuous pumping effect.
