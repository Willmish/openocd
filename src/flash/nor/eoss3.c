/***************************************************************************
 *   Copyright (C) 2022 by Peter Lawrence                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/*
With the EOS S3, the flash memory is never memory mapped into the target's
memory space, hence the '.verify' implementation below.  Because it is not
memory mapped, the de facto OpenOCD "verify" argument to "program" will not
work.  Instead, consider "flash verify_image" (available via the console).

In "Host Mode", the EOS S3 utilizes an external SPI flash memory; in
"Companion Mode", the EOS S3 is fed initialization data from another device.

For "Host Mode" the flash contents must be formatted according to the TRM,
and this image is copied by the 'Configuration Manager' sub-system (when
GPIO_19 is not configured to allow the debugger) into internal SRAM after a
hardware or power-on reset.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <helper/time_support.h>

#define SPI_MS_BASE 0x40007000

#define CTRLR0 (SPI_MS_BASE + 0x00)
#define CTRLR0_DFS_8_BIT    (0x7 << 0)
#define CTRLR0_TMOD_TX      (0x1 << 8)
#define CTRLR0_TMOD_RX      (0x2 << 8)
#define CTRLR0_TMOD_EEPROM  (0x3 << 8)

#define CTRLR1 (SPI_MS_BASE + 0x04)
#define SSIENR (SPI_MS_BASE + 0x08)
#define SSIENR_SSI_DISABLE  (0x0 << 0)
#define SSIENR_SSI_EN       (0x1 << 0)

#define SER    (SPI_MS_BASE + 0x10)
#define SER_SS_0_N_SELECTED (0x1 << 0)

#define BAUDR  (SPI_MS_BASE + 0x14)

#define SR     (SPI_MS_BASE + 0x28)
#define SR_BUSY             (0x1 << 0)
#define SR_TFE              (0x1 << 2)
#define SR_RFNE             (0x1 << 3)

#define IMR    (SPI_MS_BASE + 0x2C)
#define ICR    (SPI_MS_BASE + 0x48)
#define DR0    (SPI_MS_BASE + 0x60)

static int spi_wait_until_txfifo_empty(struct target *target)
{
	int err = ERROR_OK;
	for (;;) {
		uint32_t status;
		err = target_read_u32(target, SR, &status);
		if (ERROR_OK != err)
			break;
		if ((status & SR_TFE) && !(status & SR_BUSY))
			break;
	}

	return err;
}

static int spi_xfer(struct target *target, const uint8_t *tx, uint32_t tx_len, uint8_t *rx, uint32_t rx_len)
{
	int err = ERROR_OK;

	const uint32_t ctrlr0_mode = CTRLR0_DFS_8_BIT | (0 /* CPOL */ << 7) | (0 /* CPHA */ << 6);

	/* disable peripheral */
	target_write_u32(target, SSIENR, SSIENR_SSI_DISABLE);

	/* clear and disable all interrupts */
	target_write_u32(target, ICR, 0xFF);
	target_write_u32(target, IMR, 0);

	target_write_u32(target, BAUDR, 2);

	/* disable slave select */
	target_write_u32(target, SER, 0);

	/* configure SPI transfer mode */
	if (rx_len) {
		target_write_u32(target, CTRLR0, ((tx_len) ? CTRLR0_TMOD_EEPROM : CTRLR0_TMOD_RX) | ctrlr0_mode);
		target_write_u32(target, CTRLR1, rx_len - 1);
	} else {
		target_write_u32(target, CTRLR0, CTRLR0_TMOD_TX | ctrlr0_mode);
	}

	err = spi_wait_until_txfifo_empty(target);
    //LOG_INFO("spi_wait_until_txfifo_empty: %d", err);
	if (ERROR_OK != err)
    {
        LOG_INFO("spi_wait_until_txfifo_empty failed\n");
		return err;
    }

	/* enable peripheral */
	target_write_u32(target, SSIENR, SSIENR_SSI_EN);

	/* fill TX FIFO */
	while (tx_len) {
		target_write_u32(target, DR0, *tx++);
		tx_len--;
	}

	/* enable slave select */
	target_write_u32(target, SER, SER_SS_0_N_SELECTED);

	err = spi_wait_until_txfifo_empty(target);
	if (ERROR_OK != err)
    {
        LOG_INFO("spi_wait_until_txfifo_empty failed\n");
		return err;
    }
    //LOG_INFO("Done spi_wait_until_txfifo_empty and tx_len = %d\n", tx_len);

	/* retrieve data from RX FIFO */
	while (rx_len) {
		uint32_t status;
		err = target_read_u32(target, SR, &status);
		if (ERROR_OK != err)
        {
            LOG_INFO("target_read_u32 failed\n");
			break;
        }
		if (status & SR_RFNE) {
			err = target_read_u8(target, DR0, rx++);
			if (ERROR_OK != err)
            {
                LOG_INFO("target_read_u8 failed\n");
				break;
            }
			rx_len--;
		}
	}

	return err;
}

static int spi_cmd(struct target *target, uint8_t cmd)
{
	return spi_xfer(target, &cmd, 1, NULL, 0);
}

static int spi_wait_for_write(struct target *target, int timeout)
{
	int64_t endtime = timeval_ms() + timeout;
	const uint8_t cmd = SPIFLASH_READ_STATUS;
	uint8_t status;

	do {
		int err = spi_xfer(target, &cmd, 1, &status, 1);
		if (ERROR_OK != err)
			return err;
		if (!(status & SPIFLASH_BSY_BIT))
			return ERROR_OK;
	} while (timeval_ms() < endtime);

	LOG_ERROR("Timeout while polling BSY");
	return ERROR_FLASH_OPERATION_FAILED;
}

struct eoss3_flash_bank {
	bool probed;
	const struct flash_device *dev;
};

FLASH_BANK_COMMAND_HANDLER(eoss3_flash_bank_command)
{
	struct eoss3_flash_bank *priv;
	priv = malloc(sizeof(struct eoss3_flash_bank));

	priv->probed = false;
	bank->driver_priv = priv;

	return ERROR_OK;
}

static int eoss3_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct eoss3_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;
	int err = ERROR_OK;

	for (unsigned int index = first; index <= last; index++) {
		err = spi_cmd(target, SPIFLASH_WRITE_ENABLE);
		if (ERROR_OK != err)
			break;

		uint8_t tx[4];
		tx[0] = priv->dev->erase_cmd;
		tx[1] = bank->sectors[index].offset >> 16;
		tx[2] = bank->sectors[index].offset >> 8;
		tx[3] = bank->sectors[index].offset >> 0;

		err = spi_xfer(target, tx, sizeof(tx), NULL, 0);
		if (ERROR_OK != err)
			break;

		err = spi_wait_for_write(target, 1000);
		if (ERROR_OK != err)
			break;
	}

	return err;
}

static int eoss3_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct eoss3_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;
	uint8_t tx[131 /* max depth of TX FIFO */];
	const uint32_t page_size = priv->dev->pagesize;
	const uint32_t max_write = sizeof(tx) - 4;
	int err = ERROR_OK;

	while (count) {
		uint32_t chunk_size = count;
		if (chunk_size > max_write)
			chunk_size = max_write;
		uint32_t remaining_bytes_in_page = page_size - (offset & (page_size - 1));
		if (chunk_size > remaining_bytes_in_page)
			chunk_size = remaining_bytes_in_page;

		err = spi_cmd(target, SPIFLASH_WRITE_ENABLE);
		if (ERROR_OK != err)
			break;

		tx[0] = SPIFLASH_PAGE_PROGRAM;
		tx[1] = offset >> 16;
		tx[2] = offset >> 8;
		tx[3] = offset >> 0;
		memcpy(tx + 4, buffer, chunk_size);
		err = spi_xfer(target, tx, 4 + chunk_size, NULL, 0);
		if (ERROR_OK != err)
			break;

		err = spi_wait_for_write(target, 500);
		if (ERROR_OK != err)
			break;

		offset += chunk_size; count -= chunk_size; buffer += chunk_size;
	}

	return err;
}

static int eoss3_internal_flash_read_verify(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count,
bool verify)
{
	struct target *target = bank->target;
	uint8_t scratchpad[8 /* polled I/O transfers are limited to eight bytes */];
	int err;

	while (count) {
		uint32_t chunk_size = count;
		if (chunk_size > sizeof(scratchpad))
			chunk_size = sizeof(scratchpad);

		uint8_t tx[4];
		tx[0] = SPIFLASH_READ;
		tx[1] = offset >> 16;
		tx[2] = offset >> 8;
		tx[3] = offset >> 0;
		err = spi_xfer(target, tx, sizeof(tx), (verify) ? scratchpad : buffer, chunk_size);
		if (ERROR_OK != err)
			return err;
		if (verify)
			if (memcmp(scratchpad, buffer, chunk_size))
				return ERROR_FAIL;

		offset += chunk_size; count -= chunk_size; buffer += chunk_size;
	}

	return ERROR_OK;
}

static int eoss3_flash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return eoss3_internal_flash_read_verify(bank, buffer, offset, count, false);
}

static int eoss3_flash_verify(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return eoss3_internal_flash_read_verify(bank, (uint8_t *)buffer, offset, count, true);
}

static int eoss3_flash_probe(struct flash_bank *bank)
{
	struct eoss3_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;
	int err;

	uint8_t cmd[1] = { SPIFLASH_READ_ID };
	uint8_t resp[4] = { 0, 0, 0, 0 };

	err = spi_xfer(target, cmd, sizeof(cmd), resp, sizeof(resp));
	if (ERROR_OK != err)
		return err;

	uint32_t device_id;
	device_id  = resp[2] << 16;
	device_id |= resp[1] << 8;
	device_id |= resp[0] << 0;

    LOG_INFO("Required device ID: 0x%06x", device_id);
	/*
	 * note: device id is usually 3 bytes long, however the unused highest byte counts
	 * continuation codes for manufacturer id as per JEP106xx
     * name, read_cmd, qread_cmd, pprog_cmd, erase_cmd, chip_erase_cmd, device_id, pagesize, sectorsize, size_in_bytes*/
    // we want to find this: FLASH_ID("gd gd25q16c",0x03, 0x00, 0x02, 0xd8, 0xc7, 0x001540c8, 0x100, 0x10000, 0x200000),
    LOG_INFO("We want: 0x001540c8");
	priv->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
    {
        // Magic selection of correct device (even though we didnt find the correct ID)
        //if (p->device_id == 0x001540c8) {
        //    LOG_INFO("SELECTING DEVICE: %s", p->name);
        //    priv->dev = p;
        //    break;
        //}

		if (p->device_id == device_id) {
			priv->dev = p;
			break;
		}
    }

	if (!priv->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", device_id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		priv->dev->name, priv->dev->device_id);

	bank->write_start_alignment = priv->dev->pagesize;
	bank->write_end_alignment = priv->dev->pagesize;

	bank->size = priv->dev->size_in_bytes;

	bank->num_sectors = bank->size / priv->dev->sectorsize;
	bank->sectors = alloc_block_array(0, priv->dev->sectorsize, bank->num_sectors);

	if (!bank->sectors)
		return ERROR_FAIL;

	priv->probed = true;

	return ERROR_OK;
}

static int eoss3_flash_auto_probe(struct flash_bank *bank)
{
	struct eoss3_flash_bank *priv = bank->driver_priv;

	if (priv->probed)
		return ERROR_OK;

	return eoss3_flash_probe(bank);
}

static void eoss3_flash_free_driver_priv(struct flash_bank *bank)
{
	free(bank->driver_priv);
	bank->driver_priv = NULL;
}

const struct flash_driver eoss3_flash = {
	.name = "eoss3",
	.flash_bank_command = eoss3_flash_bank_command,
	.erase = eoss3_flash_erase,
	.write = eoss3_flash_write,
	.read = eoss3_flash_read,
	.verify = eoss3_flash_verify,
	.probe = eoss3_flash_probe,
	.auto_probe = eoss3_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = eoss3_flash_free_driver_priv
};
