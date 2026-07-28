// Prints the committed footprint tables so test/masks.test.ts can compare them
// against the simulator's own swept offsets. The tables are generated rather
// than derived, so what this guards against is staleness: geometry that has
// moved on in src/ without cpp/include/masks.h being regenerated would
// otherwise only surface as a pathfinding bug on one board.

#include <stdio.h>

#include "masks.h"

static void dump(const char* name, const masks::Mask& mask) {
  printf("%s %u\n", name, static_cast<unsigned>(mask.count));
  for (int k = 0; k < mask.count; k++) {
    printf("%d %d %d\n", mask.rows[k].dy, mask.rows[k].xMin, mask.rows[k].xMax);
  }
}

int main() {
  printf("START_CLEAR_RADIUS_CM %.17g\n", masks::START_CLEAR_RADIUS_CM);
  dump("FACING_NORTH", masks::FACING_NORTH);
  dump("FACING_EAST", masks::FACING_EAST);
  dump("QUARTER_TURN", masks::QUARTER_TURN);
  dump("FULL_TURN", masks::FULL_TURN);
  return 0;
}
