/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Clement Burin des Roziers                       *
 *   clement.burin-des-roziers@hikob.com                                   *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

/* stm32u0x flash register locations */

#define FLASH_ACR		0x00
#define FLASH_KEYR	        0x04
#define FLASH_OPTKEYR	        0x08
#define FLASH_SR		0x0c
#define FLASH_CR		0x10

/* FLASH_ACR bites */
#define FLASH_ACR__LATENCY		(1<<0)
#define FLASH_ACR__PRFTEN		(1<<8)

/* FLASH_SR bits */
#define FLASH_SR__BSY		(1<<0)
#define FLASH_SR__EOP		(1<<1)
#define FLASH_SR__ENDHV		(1<<2)
#define FLASH_SR__READY		(1<<3)
#define FLASH_SR__WRPERR	(1<<8)
#define FLASH_SR__PGAERR	(1<<9)
#define FLASH_SR__SIZERR	(1<<10)
#define FLASH_SR__OPTVERR	(1<<11)

/* register unlock keys */

#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

/* other registers */
#define DBGMCU_IDCODE		0xE0042000
#define DBGMCU_IDCODE_L0	0x40015800

/* Constants */
#define FLASH_SECTOR_SIZE 0x800
#define FLASH_BANK0_ADDRESS 0x08000000

/* option bytes */
#define OPTION_BYTES_ADDRESS 0x1FF80000

#define OPTION_BYTE_0_PR1 0xFFFF0000
#define OPTION_BYTE_0_PR0 0xFF5500AA

//static int stm32u0x_unlock_program_memory(struct flash_bank *bank);
//static int stm32u0x_lock_program_memory(struct flash_bank *bank);
static int stm32u0x_enable_write_half_page(struct flash_bank *bank);
static int stm32u0x_erase_sector(struct flash_bank *bank, int sector);
static int stm32u0x_wait_until_bsy_clear(struct flash_bank *bank);
//static int stm32u0x_lock(struct flash_bank *bank);
//static int stm32u0x_unlock(struct flash_bank *bank);
static int stm32u0x_mass_erase(struct flash_bank *bank);
static int stm32u0x_wait_until_bsy_clear_timeout(struct flash_bank *bank, int timeout);
static int stm32u0x_update_part_info(struct flash_bank *bank, uint16_t flash_size_in_kb);

struct stm32u0x_rev {
	uint16_t rev;
	const char *str;
};

struct stm32u0x_part_info {
	uint16_t id;
	const char *device_str;
	const struct stm32u0x_rev *revs;
	size_t num_revs;
	unsigned int page_size;
	unsigned int pages_per_sector;
	uint16_t max_flash_size_kb;
	uint16_t first_bank_size_kb; /* used when has_dual_banks is true */
	bool has_dual_banks;

	uint32_t flash_base;	/* Flash controller registers location */
	uint32_t fsize_base;	/* Location of FSIZE register */
};

struct stm32u0x_flash_bank {
	int probed;
	uint32_t idcode;
	uint32_t user_bank_size;
	uint32_t flash_base;

	struct stm32u0x_part_info part_info;
};

static const struct stm32u0x_rev stm32_489_revs[] = {
	{ 0x1000, "A" }
};

static const struct stm32u0x_part_info stm32u0x_parts[] = {
	{
		.id					= 0x489,
		.revs				= stm32_489_revs,
		.num_revs			= ARRAY_SIZE(stm32_489_revs),
		.device_str			= "STM32U0xx (Cat.1 - Low/Medium Density)",
		.page_size			= 2048,
		.pages_per_sector		= 16,
		.max_flash_size_kb		= 256,
		.has_dual_banks			= false,
		.flash_base			= 0x40022000,
		.fsize_base			= 0x1fff6ea0,
	}
};

/* flash bank stm32u0x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32u0x_flash_bank_command)
{
	struct stm32u0x_flash_bank *stm32u0x_info;
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Create the bank structure */
	stm32u0x_info = calloc(1, sizeof(*stm32u0x_info));

	/* Check allocation */
	if (stm32u0x_info == NULL) {
		LOG_ERROR("failed to allocate bank structure");
		return ERROR_FAIL;
	}

	bank->driver_priv = stm32u0x_info;

	stm32u0x_info->probed = 0;
	stm32u0x_info->user_bank_size = bank->size;

	/* the stm32u erased value is 0x00 */
	bank->default_padded_value = bank->erased_value = 0x00;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32u0x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32u0x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "stm32u0x mass erase complete");
	} else {
		command_print(CMD, "stm32u0x mass erase failed");
	}

	return retval;
}
#if 0
COMMAND_HANDLER(stm32u0x_handle_lock_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

		retval = stm32u0x_lock(bank);

	if (retval == ERROR_OK)
		command_print(CMD, "STM32U0x locked, takes effect after power cycle.");
	else
		command_print(CMD, "STM32U0x lock failed");

	return retval;
}

COMMAND_HANDLER(stm32u0x_handle_unlock_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32u0x_unlock(bank);

	if (retval == ERROR_OK)
		command_print(CMD, "STM32U0x unlocked, takes effect after power cycle.");
	else
		command_print(CMD, "STM32U0x unlock failed");

	return retval;
}

static int stm32u0x_protect_check(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;

	uint32_t wrpr;

	/*
	 * Read the WRPR word, and check each bit (corresponding to each
	 * flash sector
	 */
	retval = target_read_u32(target, stm32u0x_info->flash_base + FLASH_WRPR,
			&wrpr);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < bank->num_sectors; i++) {
		if (wrpr & (1 << i))
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}
	return ERROR_OK;
}
#endif

static int stm32u0x_erase(struct flash_bank *bank, int first, int last)
{
	int retval;

	/*
	 * It could be possible to do a mass erase if all sectors must be
	 * erased, but it is not implemented yet.
	 */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/*
	 * Loop over the selected sectors and erase them
	 */
	for (int i = first; i <= last; i++) {
		retval = stm32u0x_erase_sector(bank, i);
		if (retval != ERROR_OK)
			return retval;
		bank->sectors[i].is_erased = 1;
	}
	return ERROR_OK;
}

static int stm32u0x_write_half_pages(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;

	uint32_t hp_nb = stm32u0x_info->part_info.page_size / 2;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;

	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;

	int retval = ERROR_OK;

	static const uint8_t stm32u0x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32u0x.inc"
	};

	/* Make sure we're performing a half-page aligned write. */
	if (count % hp_nb) {
		LOG_ERROR("The byte count must be %" PRIu32 "B-aligned but count is %" PRIi32 "B)", hp_nb, count);
		return ERROR_FAIL;
	}

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stm32u0x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Write the flashing code */
	retval = target_write_buffer(target,
			write_algorithm->address,
			sizeof(stm32u0x_flash_write_code),
			stm32u0x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* Allocate half pages memory */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		if (buffer_size > 1024)
			buffer_size -= 1024;
		else
			buffer_size /= 2;

		if (buffer_size <= stm32u0x_info->part_info.page_size) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);

	/* Enable half-page write */
	retval = stm32u0x_enable_write_half_page(bank);
	if (retval != ERROR_OK) {
		target_free_working_area(target, source);
		target_free_working_area(target, write_algorithm);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		return retval;
	}

	struct armv7m_common *armv7m = target_to_armv7m(target);
	if (armv7m == NULL) {

		/* something is very wrong if armv7m is NULL */
		LOG_ERROR("unable to get armv7m target");
		return retval;
	}

	/* save any DEMCR flags and configure target to catch any Hard Faults */
	uint32_t demcr_save = armv7m->demcr;
	armv7m->demcr = VC_HARDERR;

	/* Loop while there are bytes to write */
	while (count > 0) {
		uint32_t this_count;
		this_count = (count > buffer_size) ? buffer_size : count;

		/* Write the next half pages */
		retval = target_write_buffer(target, source->address, this_count, buffer);
		if (retval != ERROR_OK)
			break;

		/* 4: Store useful information in the registers */
		/* the destination address of the copy (R0) */
		buf_set_u32(reg_params[0].value, 0, 32, address);
		/* The source address of the copy (R1) */
		buf_set_u32(reg_params[1].value, 0, 32, source->address);
		/* The length of the copy (R2) */
		buf_set_u32(reg_params[2].value, 0, 32, this_count / 4);

		/* 5: Execute the bunch of code */
		retval = target_run_algorithm(target, 0, NULL, sizeof(reg_params)
				/ sizeof(*reg_params), reg_params,
				write_algorithm->address, 0, 10000, &armv7m_info);
		if (retval != ERROR_OK)
			break;

		/* check for Hard Fault */
		if (armv7m->exception_number == 3)
			break;

		/* 6: Wait while busy */
		retval = stm32u0x_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			break;

		buffer += this_count;
		address += this_count;
		count -= this_count;
	}

	/* restore previous flags */
	armv7m->demcr = demcr_save;

	if (armv7m->exception_number == 3) {

		/* the stm32l15x devices seem to have an issue when blank.
		 * if a ram loader is executed on a blank device it will
		 * Hard Fault, this issue does not happen for a already programmed device.
		 * A related issue is described in the stm32l151xx errata (Doc ID 17721 Rev 6 - 2.1.3).
		 * The workaround of handling the Hard Fault exception does work, but makes the
		 * loader more complicated, as a compromise we manually write the pages, programming time
		 * is reduced by 50% using this slower method.
		 */

		LOG_WARNING("Couldn't use loader, falling back to page memory writes");

		while (count > 0) {
			uint32_t this_count;
			this_count = (count > hp_nb) ? hp_nb : count;

			/* Write the next half pages */
			retval = target_write_buffer(target, address, this_count, buffer);
			if (retval != ERROR_OK)
				break;

			/* Wait while busy */
			retval = stm32u0x_wait_until_bsy_clear(bank);
			if (retval != ERROR_OK)
				break;

			buffer += this_count;
			address += this_count;
			count -= this_count;
		}
	}

#if 0
	if (retval == ERROR_OK)
		retval = stm32u0x_lock_program_memory(bank);
#endif
	
	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return retval;
}

static int stm32u0x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;

	uint32_t hp_nb = stm32u0x_info->part_info.page_size / 2;
	uint32_t halfpages_number;
	uint32_t bytes_remaining = 0;
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	//	int retval, retval2;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

#if 0
	retval = stm32u0x_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;
#endif
	/* first we need to write any unaligned head bytes upto
	 * the next 128 byte page */

	if (offset % hp_nb)
		bytes_remaining = MIN(count, hp_nb - (offset % hp_nb));

	while (bytes_remaining > 0) {
		uint8_t value[4] = {0xff, 0xff, 0xff, 0xff};

		/* copy remaining bytes into the write buffer */
		uint32_t bytes_to_write = MIN(4, bytes_remaining);
		memcpy(value, buffer + bytes_written, bytes_to_write);

		retval = target_write_buffer(target, address, 4, value);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		bytes_written += bytes_to_write;
		bytes_remaining -= bytes_to_write;
		address += 4;

		retval = stm32u0x_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;
	}

	offset += bytes_written;
	count -= bytes_written;

	/* this should always pass this check here */
	assert((offset % hp_nb) == 0);

	/* calculate half pages */
	halfpages_number = count / hp_nb;

	if (halfpages_number) {
		retval = stm32u0x_write_half_pages(bank, buffer + bytes_written, offset, hp_nb * halfpages_number);
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* attempt slow memory writes */
			LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			halfpages_number = 0;
		} else {
			if (retval != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	/* write any remaining bytes */
	uint32_t page_bytes_written = hp_nb * halfpages_number;
	bytes_written += page_bytes_written;
	address += page_bytes_written;
	bytes_remaining = count - page_bytes_written;

#if 0
	retval = stm32u0x_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;
#endif
	
	while (bytes_remaining > 0) {
		uint8_t value[4] = {0xff, 0xff, 0xff, 0xff};

		/* copy remaining bytes into the write buffer */
		uint32_t bytes_to_write = MIN(4, bytes_remaining);
		memcpy(value, buffer + bytes_written, bytes_to_write);

		retval = target_write_buffer(target, address, 4, value);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		bytes_written += bytes_to_write;
		bytes_remaining -= bytes_to_write;
		address += 4;

		retval = stm32u0x_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;
	}

reset_pg_and_lock:
#if 0	
	retval2 = stm32u0x_lock_program_memory(bank);
	if (retval == ERROR_OK)
		retval = retval2;
#endif
	return retval;
}

static int stm32u0x_read_id_code(struct target *target, uint32_t *id)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int retval;
	if (armv7m->arm.is_armv6m == true)
		retval = target_read_u32(target, DBGMCU_IDCODE_L0, id);
	else
	/* read stm32 device id register */
		retval = target_read_u32(target, DBGMCU_IDCODE, id);
	return retval;
}

static int stm32u0x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb;
	uint32_t device_id;
	uint32_t base_address = FLASH_BANK0_ADDRESS;
	uint32_t second_bank_base;
	unsigned int n;

	stm32u0x_info->probed = 0;

	int retval = stm32u0x_read_id_code(bank->target, &device_id);
	if (retval != ERROR_OK)
		return retval;

	stm32u0x_info->idcode = device_id;

	LOG_DEBUG("device id = 0x%08" PRIx32 "", device_id);

	for (n = 0; n < ARRAY_SIZE(stm32u0x_parts); n++) {
		if ((device_id & 0xfff) == stm32u0x_parts[n].id) {
			stm32u0x_info->part_info = stm32u0x_parts[n];
			break;
		}
	}

	if (n == ARRAY_SIZE(stm32u0x_parts)) {
		LOG_WARNING("Cannot identify target as a STM32L family.");
		return ERROR_FAIL;
	} else {
		LOG_INFO("Device: %s", stm32u0x_info->part_info.device_str);
	}

	stm32u0x_info->flash_base = stm32u0x_info->part_info.flash_base;

	/* Get the flash size from target. */
	retval = target_read_u16(target, stm32u0x_info->part_info.fsize_base,
			&flash_size_in_kb);

	/* Failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("STM32L flash size failed, probe inaccurate - assuming %dk flash",
			stm32u0x_info->part_info.max_flash_size_kb);
		flash_size_in_kb = stm32u0x_info->part_info.max_flash_size_kb;
	} else if (flash_size_in_kb > stm32u0x_info->part_info.max_flash_size_kb) {
		LOG_WARNING("STM32L probed flash size assumed incorrect since FLASH_SIZE=%dk > %dk, - assuming %dk flash",
			flash_size_in_kb, stm32u0x_info->part_info.max_flash_size_kb,
			stm32u0x_info->part_info.max_flash_size_kb);
		flash_size_in_kb = stm32u0x_info->part_info.max_flash_size_kb;
	}

	/* Overwrite default dual-bank configuration */
	retval = stm32u0x_update_part_info(bank, flash_size_in_kb);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	if (stm32u0x_info->part_info.has_dual_banks) {
		/* Use the configured base address to determine if this is the first or second flash bank.
		 * Verify that the base address is reasonably correct and determine the flash bank size
		 */
		second_bank_base = base_address +
			stm32u0x_info->part_info.first_bank_size_kb * 1024;
		if (bank->base == second_bank_base || !bank->base) {
			/* This is the second bank  */
			base_address = second_bank_base;
			flash_size_in_kb = flash_size_in_kb -
				stm32u0x_info->part_info.first_bank_size_kb;
		} else if (bank->base == base_address) {
			/* This is the first bank */
			flash_size_in_kb = stm32u0x_info->part_info.first_bank_size_kb;
		} else {
			LOG_WARNING("STM32L flash bank base address config is incorrect. "
					TARGET_ADDR_FMT " but should rather be 0x%" PRIx32
					" or 0x%" PRIx32,
						bank->base, base_address, second_bank_base);
			return ERROR_FAIL;
		}
		LOG_INFO("STM32L flash has dual banks. Bank (%d) size is %dkb, base address is 0x%" PRIx32,
				bank->bank_number, flash_size_in_kb, base_address);
	} else {
		LOG_INFO("STM32L flash size is %dkb, base address is 0x%" PRIx32, flash_size_in_kb, base_address);
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (stm32u0x_info->user_bank_size) {
		flash_size_in_kb = stm32u0x_info->user_bank_size / 1024;
		LOG_INFO("ignoring flash probed value, using configured bank size: %dkbytes", flash_size_in_kb);
	}

	/* calculate numbers of sectors (4kB per sector) */
	int num_sectors = (flash_size_in_kb * 1024) / FLASH_SECTOR_SIZE;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->size = flash_size_in_kb * 1024;
	bank->base = base_address;
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (bank->sectors == NULL) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	for (i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
		bank->sectors[i].size = FLASH_SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	stm32u0x_info->probed = 1;

	return ERROR_OK;
}

static int stm32u0x_auto_probe(struct flash_bank *bank)
{
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;

	if (stm32u0x_info->probed)
		return ERROR_OK;

	return stm32u0x_probe(bank);
}

/* This method must return a string displaying information about the bank */
static int stm32u0x_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;
	const struct stm32u0x_part_info *info = &stm32u0x_info->part_info;
	uint16_t rev_id = stm32u0x_info->idcode >> 16;
	const char *rev_str = NULL;

	if (!stm32u0x_info->probed) {
		int retval = stm32u0x_probe(bank);
		if (retval != ERROR_OK) {
			snprintf(buf, buf_size,
				"Unable to find bank information.");
			return retval;
		}
	}

	for (unsigned int i = 0; i < info->num_revs; i++)
		if (rev_id == info->revs[i].rev)
			rev_str = info->revs[i].str;

	if (rev_str != NULL) {
		snprintf(buf, buf_size,
			"%s - Rev: %s",
			info->device_str, rev_str);
	} else {
		snprintf(buf, buf_size,
			"%s - Rev: unknown (0x%04x)",
			info->device_str, rev_id);
	}

	return ERROR_OK;
}

static const struct command_registration stm32u0x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = stm32u0x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device. including available EEPROM",
	},
#if 0
	{
		.name = "lock",
		.handler = stm32u0x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Increase the readout protection to Level 1.",
	},
	{
		.name = "unlock",
		.handler = stm32u0x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lower the readout protection from Level 1 to 0.",
	},
#endif	
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32u0x_command_handlers[] = {
	{
		.name = "stm32u0x",
		.mode = COMMAND_ANY,
		.help = "stm32u0x flash command group",
		.usage = "",
		.chain = stm32u0x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver stm32u0x_flash = {
		.name = "stm32u0x",
		.commands = stm32u0x_command_handlers,
		.flash_bank_command = stm32u0x_flash_bank_command,
		.erase = stm32u0x_erase,
		.write = stm32u0x_write,
		.read = default_flash_read,
		.probe = stm32u0x_probe,
		.auto_probe = stm32u0x_auto_probe,
		.erase_check = default_flash_blank_check,
		//		.protect_check = stm32u0x_protect_check,
		.info = stm32u0x_get_info,
		.free_driver_priv = default_flash_free_driver_priv,
};

#if 0
static int stm32u0x_enable_write_half_page(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;
	int retval;
	uint32_t reg32;

#if 0
	/**
	 * Unlock the program memory, then set the FPRG bit in the PECR register.
	 */
	retval = stm32u0x_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;
#endif
	
	retval = target_read_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			&reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__FPRG;
	retval = target_write_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			&reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__PROG;
	retval = target_write_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			reg32);

	return retval;
}

static int stm32u0x_lock_program_memory(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;
	int retval;
	uint32_t reg32;

	/* To lock the program memory, simply set the lock bit and lock PECR */

	retval = target_read_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			&reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__PRGLOCK;
	retval = target_write_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			&reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__PELOCK;
	retval = target_write_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			reg32);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}
#endif
static int stm32u0x_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;
	int retval;
	uint32_t reg32;

	/*
	 * To erase a sector (i.e. stm32u0x_info->part_info.pages_per_sector pages),
	 * first unlock the memory, loop over the pages of this sector
	 * and write 0x0 to its first word.
	 */

#if 0
	retval = stm32u0x_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;
#endif	

	for (int page = 0; page < (int)stm32u0x_info->part_info.pages_per_sector;
			page++) {
		reg32 = FLASH_PECR__PROG | FLASH_PECR__ERASE;
		retval = target_write_u32(target,
				stm32u0x_info->flash_base + FLASH_PECR, reg32);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32u0x_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			return retval;

		uint32_t addr = bank->base + bank->sectors[sector].offset + (page
				* stm32u0x_info->part_info.page_size);
		retval = target_write_u32(target, addr, 0x0);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32u0x_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = stm32u0x_lock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static inline int stm32u0x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;

	return target_read_u32(target, stm32u0x_info->flash_base + FLASH_SR, status);
}

static int stm32u0x_wait_until_bsy_clear(struct flash_bank *bank)
{
	return stm32u0x_wait_until_bsy_clear_timeout(bank, 100);
}

static int stm32u0x_wait_until_bsy_clear_timeout(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32u0x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_SR__BSY) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_SR__WRPERR) {
		LOG_ERROR("access denied / write protected");
		retval = ERROR_FAIL;
	}

	if (status & FLASH_SR__PGAERR) {
		LOG_ERROR("invalid program address");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_SR__OPTVERR) {
		/* If this operation fails, we ignore it and report the original retval */
		target_write_u32(target, stm32u0x_info->flash_base + FLASH_SR, status & FLASH_SR__OPTVERR);
	}

	return retval;
}

static int stm32u0x_obl_launch(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;
	int retval;

	/* This will fail as the target gets immediately rebooted */
	target_write_u32(target, stm32u0x_info->flash_base + FLASH_PECR,
			 FLASH_PECR__OBL_LAUNCH);

	size_t tries = 10;
	do {
		target_halt(target);
		retval = target_poll(target);
	} while (--tries > 0 &&
		 (retval != ERROR_OK || target->state != TARGET_HALTED));

	return tries ? ERROR_OK : ERROR_FAIL;
}
#if 0
static int stm32u0x_lock(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32u0x_unlock_options_bytes(bank);
	if (retval != ERROR_OK)
		return retval;

	/* set the RDP protection level to 1 */
	retval = target_write_u32(target, OPTION_BYTES_ADDRESS, OPTION_BYTE_0_PR1);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32u0x_unlock(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32u0x_unlock_options_bytes(bank);
	if (retval != ERROR_OK)
		return retval;

	/* set the RDP protection level to 0 */
	retval = target_write_u32(target, OPTION_BYTES_ADDRESS, OPTION_BYTE_0_PR0);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32u0x_wait_until_bsy_clear_timeout(bank, 30000);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}
#endif
static int stm32u0x_mass_erase(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;
	struct stm32u0x_flash_bank *stm32u0x_info = NULL;
	uint32_t reg32;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	stm32u0x_info = bank->driver_priv;

	retval = stm32u0x_lock(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32u0x_obl_launch(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32u0x_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32u0x_obl_launch(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32u0x_info->flash_base + FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32u0x_info->flash_base + FLASH_PECR, reg32 | FLASH_PECR__OPTLOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32u0x_update_part_info(struct flash_bank *bank, uint16_t flash_size_in_kb)
{
	struct stm32u0x_flash_bank *stm32u0x_info = bank->driver_priv;

	switch (stm32u0x_info->part_info.id) {
	case 0x489: /* STM32U0xx devices */
		if (flash_size_in_kb == 256) {
 		   stm32u0x_info->part_info.first_bank_size_kb = flash_size_in_kb;
		   stm32u0x_info->part_info.has_dual_banks = false;
		}
		break;
	}

	return ERROR_OK;
}
