
#include "app_error.h"
#include "spi_slave.h"

#include "boards.h"
#include "led.h"

#include "bcp.h"
#include "bcp_spi_slave.h"
#include "interrupt_event_queue.h"



uint8_t spi_tx_buf[SPI_BUF_LEN] = {0};
uint8_t spi_rx_buf[SPI_BUF_LEN] = {0};


spi_slave_state_e state = SPI_SLAVE_STATE_WAIT_FOR_COMMAND;




static void spi_slave_event_handle(spi_slave_evt_t event) {
	uint32_t err_code;
//	led_off(LED_0);
led_off(LED_0);
	// Check the event type. There are only two events, and only one is useful.
	if (event.evt_type == SPI_SLAVE_XFER_DONE) {

		if (event.rx_amount == BCP_COMMAND_LEN) {
			// Got just a command. Setup the response as required.
			// In most cases we don't respond, but we still need to set the
			// RX buffer.

			switch (spi_rx_buf[0]) {

			  case BCP_CMD_READ_IRQ: {
				// We interrupted the host processor because an event
				// occurred. Now read that back to the master.

				uint8_t data_len;
				uint8_t len;
				uint8_t  response_type;

				// Start by clearing the interrupt that got us here.
				bcp_interupt_host_clear();

				data_len = interrupt_event_queue_get(&response_type, spi_tx_buf+2);
				len = data_len + 1; // for the response type byte

				// Create the response SPI buffer.
				spi_tx_buf[0] = len;
				spi_tx_buf[1] = response_type;

				// Send the TX buffer to the SPI module
				err_code = spi_slave_buffers_set(spi_tx_buf,
				                                 spi_rx_buf,
				                                 SPI_BUF_LEN,
				                                 SPI_BUF_LEN);

    nrf_gpio_pin_toggle(3);
				APP_ERROR_CHECK(err_code);

				// The next SPI transaction will be looking for us to return
				// data.
				state = SPI_SLAVE_STATE_RUN_COMMAND;

				return;
			  }

			  case BCP_CMD_SNIFF_ADVERTISEMENTS:
				// Instructs us to send all advertisements to the host

				bcp_sniff_advertisements();
				state = SPI_SLAVE_STATE_WAIT_FOR_COMMAND;
				break;

			  default:
				break;
			}

		}



	// 	// React based on which state we are in.
	// 	switch (state) {
	// 	  case SPI_SLAVE_STATE_WAIT_FOR_COMMAND:
	// 		// We were waiting for a command from the master.

	// 		// Commands are 1 byte.
	// 		APP_ERROR_CHECK_BOOL(event.rx_amount == BCP_COMMAND_LEN);
	// //led_off(LED_0);

	// 		// We got a command from the host. Do the correct thing.
	// 		switch (spi_rx_buf[0]) {

	// 		  case BCP_CMD_READ_IRQ: {
	// 			// We interrupted the host processor because an event
	// 			// occurred. Now read that back to the master.

	// 			uint8_t data_len;
	// 			uint8_t len;
	// 			uint8_t  response_type;

	// 			// Start by clearing the interrupt that got us here.
	// 			bcp_interupt_host_clear();

	// 			data_len = interrupt_event_queue_get(&response_type, spi_tx_buf+2);
	// 			len = data_len + 1; // for the response type byte

	// 			// Create the response SPI buffer.
	// 			spi_tx_buf[0] = len;
	// 			//spi_tx_buf[0] = 0x23;
	// 			spi_tx_buf[1] = response_type;
	// 			// spi_tx_buf[1] = 0xb0;
	// 			// spi_tx_buf[2] = 0xb1;
	// 			// spi_tx_buf[3] = 0xb2;
	// 			// spi_tx_buf[4] = 0xb3;
	// 			// spi_tx_buf[5] = 0xb4;

	// 			// Send the TX buffer to the SPI module
	// 			err_code = spi_slave_buffers_set(spi_tx_buf,
	// 			                                 spi_rx_buf,
	// 			                                 SPI_BUF_LEN,
	// 			                                 SPI_BUF_LEN);

 //    nrf_gpio_pin_toggle(3);
	// 			APP_ERROR_CHECK(err_code);

	// 			// The next SPI transaction will be looking for us to return
	// 			// data.
	// 			state = SPI_SLAVE_STATE_RUN_COMMAND;

	// 			return;
	// 		  }

	// 		  case BCP_CMD_SNIFF_ADVERTISEMENTS:
	// 			// Instructs us to send all advertisements to the host

	// 			bcp_sniff_advertisements();
	// 			state = SPI_SLAVE_STATE_WAIT_FOR_COMMAND;
	// 			break;

	// 		  default:
	// 			break;
	// 		}



	// 		break;

	// 	  case SPI_SLAVE_STATE_RUN_COMMAND:
	// 		// Already gave the buffer to the SPI module. Don't actually
	// 		// need to do anything here.
	// 		state = SPI_SLAVE_STATE_WAIT_FOR_COMMAND;
	// 		break;

	// 	  default:
	// 		state = SPI_SLAVE_STATE_WAIT_FOR_COMMAND;
	// 		break;

	// 	}

		spi_tx_buf[0] = 0;
		// spi_tx_buf[1] = 0;
		// spi_tx_buf[2] = 0;
		// spi_tx_buf[3] = 0;

		err_code = spi_slave_buffers_set(spi_tx_buf,
				                         spi_rx_buf,
				                         SPI_BUF_LEN,
				                         SPI_BUF_LEN);

	}
}


uint32_t spi_slave_example_init(void)
{
	uint32_t           err_code;
	spi_slave_config_t spi_slave_config;

	// This callback fires after the master has de-asserted chip select
	err_code = spi_slave_evt_handler_register(spi_slave_event_handle);
	APP_ERROR_CHECK(err_code);

	// Setup the pins from the board's .h file
	spi_slave_config.pin_miso         = SPIS_MISO_PIN;
	spi_slave_config.pin_mosi         = SPIS_MOSI_PIN;
	spi_slave_config.pin_sck          = SPIS_SCK_PIN;
	spi_slave_config.pin_csn          = SPIS_CSN_PIN;
	spi_slave_config.mode             = SPI_MODE_0;
	spi_slave_config.bit_order        = SPIM_MSB_FIRST;
	spi_slave_config.def_tx_character = 0xFF;
	spi_slave_config.orc_tx_character = 0x55;

	err_code = spi_slave_init(&spi_slave_config);
	APP_ERROR_CHECK(err_code);

	// Set buffers.
	err_code = spi_slave_buffers_set(spi_tx_buf,
									 spi_rx_buf,
									 SPI_BUF_LEN,
									 SPI_BUF_LEN);
	APP_ERROR_CHECK(err_code);
//led_off(LED_0);
	return NRF_SUCCESS;
}

