
#ifndef BOARDS_H
#define BOARDS_H

#if defined(BOARD_NRF6310)
  #include "boards/nrf6310.h"
#elif defined(BOARD_PCA10000)
  #include "boards/pca10000.h"
#elif defined(BOARD_PCA10001)
  #include "boards/pca10001.h"
#elif defined(BOARD_PCA10003)
  #include "boards/pca10003.h"
#elif defined(BOARD_SQUALL)
  #include "squall.h"
#else
#error "Board is not defined"
#endif

#endif
