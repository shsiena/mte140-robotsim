#ifndef ROBOT_BITGRID_H
#define ROBOT_BITGRID_H

#include <stdint.h>
#include <string.h>

// A fixed-size boolean map, one bit per sub-cell, packed into 32-bit words and
// stored row-major with cy = 0 as the bottom row. Sized entirely from template
// parameters, so instances live in .bss and nothing is ever allocated.
//
// Reads outside the grid report false, matching the simulator's BoolGrid:
// off-board space reads as free.
template <int COLS, int ROWS>
class BitGrid {
 public:
  void fill(bool value) { memset(words_, value ? 0xFF : 0x00, sizeof(words_)); }

  bool get(int cx, int cy) const {
    if (cx < 0 || cy < 0 || cx >= COLS || cy >= ROWS) return false;
    return ((words_[cy][cx >> 5] >> (cx & 31)) & 1u) != 0;
  }

  void set(int cx, int cy, bool value) {
    if (cx < 0 || cy < 0 || cx >= COLS || cy >= ROWS) return;
    const uint32_t bit = 1u << (cx & 31);
    if (value) {
      words_[cy][cx >> 5] |= bit;
    } else {
      words_[cy][cx >> 5] &= ~bit;
    }
  }

  // True if any bit in row cy between xLo and xHi inclusive is set.
  //
  // Testing a span of up to 32 sub-cells per word operation is what makes the
  // clearance masks affordable: a mask row costs a handful of word tests
  // rather than one lookup per sub-cell it covers.
  bool anySetInRange(int cy, int xLo, int xHi) const {
    if (cy < 0 || cy >= ROWS) return false;
    if (xLo < 0) xLo = 0;
    if (xHi > COLS - 1) xHi = COLS - 1;
    if (xLo > xHi) return false;

    const uint32_t* row = words_[cy];
    const int loWord = xLo >> 5;
    const int hiWord = xHi >> 5;
    const uint32_t loMask = 0xFFFFFFFFu << (xLo & 31);
    const uint32_t hiMask = 0xFFFFFFFFu >> (31 - (xHi & 31));

    if (loWord == hiWord) return (row[loWord] & loMask & hiMask) != 0;
    if ((row[loWord] & loMask) != 0) return true;
    for (int w = loWord + 1; w < hiWord; w++) {
      if (row[w] != 0) return true;
    }
    return (row[hiWord] & hiMask) != 0;
  }

 private:
  // Rows are padded out to a whole number of words. Padding bits are never
  // reachable through the accessors above, which clamp to COLS first.
  enum { WORDS_PER_ROW = (COLS + 31) / 32 };
  uint32_t words_[ROWS][WORDS_PER_ROW];
};

#endif  // ROBOT_BITGRID_H
