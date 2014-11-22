#ifndef BOARDS_H
#define BOARDS_H

#if defined(BOARD_NRF6310)
	#include "boards/nrf6310.h"
	#define DEVICE_NAME "nrf6310"
#elif defined(BOARD_PCA10000)
	#include "boards/pca10000.h"
	#define DEVICE_NAME "pca10000"
#elif defined(BOARD_PCA10001)
	#include "boards/pca10001.h"
	#define DEVICE_NAME "pca10001"
#elif defined(BOARD_PCA10003)
	#include "boards/pca10003.h"
	#define DEVICE_NAME "luxapose: demo_floor"
#elif defined(BOARD_SQUALL)
	#include "squall.h"
#elif defined(BOARD_ZIGBEAG)
	#include "zigbeag.h"
#elif defined(BOARD_QUITBIT)
	#include "quitbit.h"
#else
	#error "Board is not defined"
#endif

#endif
