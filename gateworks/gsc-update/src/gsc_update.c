/**
 * gsc_update.c Gateworks System Controller Firmware Updater
 *
 * Copyright 2004-2016 Gateworks Corporation
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>

#include "i2c.h"
#include "i2c_upgrader.h"

#define GSC_UPDATER_REV "1.5"

#define GSC_DEVICE		0x20
#define GSC_UPDATER		0x21
#define GSC_PASSWORD 	0x58
#define GSC_UNLOCK 		0x01
#define GSC_PROGRAM 	0x02
#define GSC_ERASE 		0x04
#define GSC_PUC 			0x06

#define GSC2_PUC			0x1
#define GSC2_ADDR			0x2
#define GSC2_ERASE		0x3
#define GSC2_WORD			0x4
#define GSC2_PROG			0x5

struct eeprom_layout {
	const char* name;
	int start;
	int end;
	int eeprom_start;
	int eeprom_end;
};

struct eeprom_layout layouts[] = {
	{
		.name         = "GSC v1",
		.start        = 0xe000,
		.end          = 0xffff,
		.eeprom_start = 0xfc00,
		.eeprom_end   = 0xfdff,
	},
	{
		.name         = "GSC v2",
		.start        = 0xc000,
		.end          = 0xffff,
		.eeprom_start = 0xf800,
		.eeprom_end   = 0xfdff,
	},
};

struct eeprom_layout *parse_data_file(char *filename, unsigned char data[16][16384], unsigned short address[16], unsigned short length[16]);
int calc_crc(struct eeprom_layout *, unsigned char data[16][16384], unsigned short address[16], unsigned short length[16]);

void print_banner(void)
{
	printf("Gateworks GSC Updater v%s\n", GSC_UPDATER_REV);
	printf("Copyright (C) 2004-2016, Gateworks Corporation, All Rights Reserved\n");
	printf("Built %s, %s\n", __TIME__, __DATE__);
}

void print_help(void)
{
	print_banner();
	printf("\nUsage:\n");
	printf("\t-f,--flash <filename>   Program flash device using [filename]\n");
	printf("\t-c,--crc <filename>     Show CRC of [filename] without programming flash\n");
	printf("\t-v,--verbose            Increase Verbosity\n");
	printf("\t-q,--quiet              do not display progress\n");
	printf("\t-v,--verbose            Increase Verbosity\n");
	printf("\n");
}

int main(int argc, char **argv)
{
	struct eeprom_layout *layout;
	unsigned long funcs;
	unsigned char data[16][16384];
	unsigned short address[16];
	unsigned short length[16];
	char *prog_filename = NULL;
	unsigned char buffer[16];
	signed char ret;
#if defined (davinci)
	unsigned char i2cbus = 1;
#else
	unsigned char i2cbus = 0;
#endif
	unsigned char calc_crc_only = 0;
	unsigned short crc;
	unsigned char verbose = 0;
	unsigned char quiet = 0;
	char device[16];

	int i, j;
	int file;

	while (1) {
		int optind = 0;
		int c;
		static struct option long_options[] = {
			{"flash", 1, 0, 'f'},
			{"crc", 1, 0, 'c'},
			{"verbose", 0, 0, 'v'},
			{"quiet", 0, 0, 'q'},
			{0, 0, 0, 0}
		};

		c = getopt_long(argc, argv, "f:c:vqh?", long_options, &optind);
		if (c == -1)
			break;

		switch (c) {
			case 'f':
				prog_filename = optarg;
				break;
			case 'c':
				prog_filename = optarg;
				calc_crc_only = 1;
				break;
			case 'q':
				quiet++;
				break;
			case 'v':
				verbose++;
				break;
			default:
				print_help();
				exit(0);
				break;
		}
	}

	if (!prog_filename) {
		print_help();
		exit(-1);
	}

	print_banner();

	/* parse and validate file */
	layout = parse_data_file(prog_filename, data, address, length);
	if (!layout) {
		exit(2);
	}

	/* display info about memory segments */
	if (verbose) {
		j = 0;
		printf("Segments:\n");
		for(i=0;i<16 && length[i];i++) {
			if (length[i]) {
				printf("\t0x%04x:%05d(%04x) bytes",
				       address[i], length[i], length[i]);
			}
			if (address[i] == layout->start + 0x400) {
				printf(" (%d unused)", (layout->eeprom_start
				       - address[i]) - length[i]);
			}
			else if (address[i] == layout->start) {
				printf(" (%d unused)", 0x400 - length[i]);
			}
			printf("\n");
		}
	}

	/* calculate CRC over parsed file */
	crc = calc_crc(layout, data, address, length);
	printf("%s: %s crc=0x%04x\n", layout->name, prog_filename, crc);
	if (calc_crc_only) {
		return 0;
	}

	sprintf(device, "/dev/i2c-%d", i2cbus);
	file = open(device, O_RDWR);
	if (file < 0 && errno == ENOENT) {
		sprintf(device, "/dev/i2c/%d", i2cbus);
		file = open(device, O_RDWR);
	}

	if (file < 0) {
		perror("unable to open i2c device");
		exit(1);
	}

	if (ioctl(file, I2C_FUNCS, &funcs) < 0) {
		perror("i2c not functional");
		exit(1);
	}

	if (ioctl(file, I2C_SLAVE_FORCE, GSC_DEVICE) < 0) {
		perror("couldn't set GSC address");
		exit(1);
	}

	/* read/show current firmware CRC and version */
	ret = i2c_smbus_read_byte_data(file, 12);
	crc = ret;
	ret = i2c_smbus_read_byte_data(file, 13);
	crc |= ret << 8;
	ret = i2c_smbus_read_byte_data(file, 14);
	printf("Current GSC Firmware Rev: %i (crc=0x%04x)\n", ret & 0xff, crc);

	/* set slave device to GSC update addr and unlock the GSC for programming */
	if (ioctl(file, I2C_SLAVE_FORCE, GSC_UPDATER) < 0) {
		perror("couldn't set GSC_UPDATER address");
		exit(1);
	}
	i2c_smbus_write_byte_data(file, 0, GSC_PASSWORD | GSC_UNLOCK);
	ret = i2c_smbus_read_byte_data(file, 0);

	/* ##### Stage 1 Upgrader ##### */
	// Erase main app
	for (i = layout->start + 0x400; i < layout->end; i += 0x200) {
		if (i >= layout->eeprom_start && i <= layout->eeprom_end)
			continue;
		buffer[0] = i & 0xff;
		buffer[1] = (i >> 8) & 0xff;
		i2c_smbus_write_i2c_block_data(file, 1, 2, buffer);
		i2c_smbus_write_byte_data(file, 0, GSC_PASSWORD | GSC_ERASE);
		while (1) {
			ret = i2c_smbus_read_byte_data(file, 2);
			if (ret != -1)
				break;
			fflush(stdout);
		}
		ret = i2c_smbus_read_byte_data(file, 1);
	}

	// Switch to Programing mode
	i2c_smbus_write_byte_data(file, 0, GSC_PASSWORD | GSC_PROGRAM);

	for (i = 1; i>= 0; i--) {
		if (i2c_upgrader_length[i]) {
			for (j = 0; j < i2c_upgrader_length[i]; j+=2) {
				// Set starting Address
				buffer[0] = (i2c_upgrader_address[i] + j) & 0xff;
				buffer[1] = ((i2c_upgrader_address[i] + j) >> 8) & 0xff;
				i2c_smbus_write_i2c_block_data(file, 1, 2, buffer);
				buffer[0] = i2c_upgrader_data[i][j];
				buffer[1] = i2c_upgrader_data[i][j+1];

				i2c_smbus_write_i2c_block_data(file, 3, 2, buffer);
				while (1) {
					ret = i2c_smbus_read_byte_data(file, 2);
					if (ret != -1)
						break;
					fflush(stdout);
				}
				if (!i && !quiet) {
					printf("Program Upgrader  %2i%%\r", (int)((((double)j / 2) / i2c_upgrader_length[i]) * 100));
					fflush(stdout);
				}
			}
			if (!i && !quiet) {
				printf("Program Upgrader  %i%%\n", 100);
				fflush(stdout);
			}
		}
	}
	// Turn off Programing Mode
	i2c_smbus_write_byte_data(file, 0, GSC_PASSWORD | GSC_UNLOCK);
	// Reset GSC
	i2c_smbus_write_byte_data(file, 0, GSC_PASSWORD | GSC_PUC);


	/* ##### Stage 2 Upgrader ##### */
	while (1) {
		ret = i2c_smbus_read_byte(file);
		if (ret != -1)
			break;
	}

	// Erase all of main flash
	for (i = layout->start; i < layout->end; i += 0x200) {
		if (i >= layout->eeprom_start && i <= layout->eeprom_end)
			continue;
		buffer[0] = i & 0xff;
		buffer[1] = (i >> 8) & 0xff;
		i2c_smbus_write_i2c_block_data(file, GSC2_ADDR, 2, buffer);
		i2c_smbus_write_byte(file, GSC2_ERASE);
		while (1) {
			ret = i2c_smbus_read_byte(file);
			if (ret != -1)
				break;
		}
	}

	/* program new segments */
	for (i = 15; i>= 0; i--) {
		if (length[i]) {
			for (j = 0; j < length[i]; j+=2) {
				// Set starting Address
				buffer[0] = (address[i] + j) & 0xff;
				buffer[1] = ((address[i] + j) >> 8) & 0xff;
				i2c_smbus_write_i2c_block_data(file, GSC2_ADDR, 2, buffer);
				buffer[0] = data[i][j];
				buffer[1] = data[i][j+1];

				i2c_smbus_write_i2c_block_data(file, GSC2_WORD, 2, buffer);
				i2c_smbus_write_byte(file, GSC2_PROG);
				while (1) {
					ret = i2c_smbus_read_byte(file);
					if (ret != -1)
						break;
					fflush(stdout);
				}
				if (!quiet) {
					printf("MSP Prg B%i   %2i%%\r", i, (int)((((double)j / 2) / length[i]) * 100));
					fflush(stdout);
				}
			}
			if (!quiet) {
				printf("MSP Prg B%i  %i%%\n", i, 100);
				fflush(stdout);
			}
		}
	}
	i2c_smbus_write_byte(file, GSC2_PUC);

	/* disable boot watchdog */
	sleep(1);
	if (ioctl(file, I2C_SLAVE_FORCE, GSC_DEVICE) < 0) {
		perror("couldn't set GSC address");
		exit(1);
	}
	i2c_smbus_write_byte(file, 1);
	ret = i2c_smbus_read_byte_data(file, 1);
	printf("R1:0x%02x\n", ret);
	if (ret & 1<<6) {
		printf("stopping boot watchdog timer\n");
		i2c_smbus_write_byte_data(file, 1, 1<<7);
	}

	close(file);

	return 0;
}

int calc_crc(struct eeprom_layout *layout, unsigned char data[16][16384], unsigned short address[16], unsigned short length[16])
{
	const unsigned short crc_16_table[16] = {
	  0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
 	  0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400 
	};
	unsigned int addr;
	unsigned short crc = 0;
	unsigned short r;
	int i,j;

	addr = layout->start;
	for (i = 0; i < 16; i++) {
		if (length[i]) {
			int segaddr = (unsigned short) address[i];

			// loop over blank gap 
			for (; addr < segaddr; addr++) {
				if (addr >= (layout->end+1))
					continue;
				// skip EEPROM flash segments
				if (addr >= layout->eeprom_start && addr <= layout->eeprom_end)
					continue;
				r = crc_16_table[crc & 0xf];
				crc = (crc >> 4) & 0x0fff;
				crc = crc ^ r ^ crc_16_table[0xf];
				r = crc_16_table[crc & 0xf];
				crc = (crc >> 4) & 0x0fff;
				crc = crc ^ r ^ crc_16_table[0xf];
			}

			// loop over segment data
			for (j = 0; j < length[i]; j++, addr++) {
				if (addr >= (layout->end+1))
					continue;
				r = crc_16_table[crc & 0xf];
				crc = (crc >> 4) & 0x0fff;
				crc = crc ^ r ^ crc_16_table[data[i][j] & 0xf];			
				r = crc_16_table[crc & 0xf];
				crc = (crc >> 4) & 0x0fff;
				crc = crc ^ r ^ crc_16_table[(data[i][j] >> 4) & 0xf];
			}
		}
	}

	return crc;
}

struct eeprom_layout *parse_data_file(char *filename, unsigned char data[16][16384], unsigned short address[16], unsigned short length[16])
{
	FILE *fd;
	char line[1024];
	short address_loc = -1;
	char t[16][4];
	short temp_word = 0;
	int i = 0, j = 0, num_scan = 0;
	int linenum = 0;

	memset(t, 0, sizeof(t));
	memset(length, 0, 16*2);
	memset(data, 0, 16*16384);

	fd = fopen(filename, "r");
	if (!fd) {
		perror("open failed");
		return NULL;
	}
	while (fgets(line, sizeof(line), fd)) {
		linenum++;
		if (linenum == 1 && line[0] != '@') {
			fprintf(stderr, "Invalid GSC firmware file\n");
			return NULL;
		} 
		if (line[0] == '@' && address_loc < 16) {
			j = 0;
			address_loc++;
			address[address_loc] = (short) strtol(line+1, 0, 16);
		} else if (line[0] != 'q') {
			num_scan = sscanf(line, "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s",
				t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9], t[10], t[11], t[12], t[13], t[14], t[15]);
			for (i = 0; i < num_scan && j < 16384; i++) {
				temp_word = strtol(t[i], 0, 16);
				data[address_loc][j++] = temp_word;
				length[address_loc]++;
			}
		}
	}

	/* Verify we match a known FLASH layout and have the 2 segments */
	for (i = 0; i < sizeof(layouts)/sizeof(layouts[0]); i++) {
		if (layouts[i].start == address[0] &&
		    layouts[i].start + 0x400 == address[1])
		{
			return(&layouts[i]);
		}
	}
	fprintf(stderr, "%s: unrecognized layout\n", filename);

	return NULL;
}
