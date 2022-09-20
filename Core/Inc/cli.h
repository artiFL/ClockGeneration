#pragma once
#ifndef _CLI_H_
#define _CLI_H_

#include "stdint.h"
#include <sys/unistd.h>
#include <errno.h>
#include "stdlib.h"
typedef struct
{
	const char *CLI_Command;
	const char *CLI_Command_Description;
	uint16_t CLI_Command_Length;
    uint8_t (*CLI_Callback)(uint8_t argc,
                            const char *argv[]);
} CLI_Command_t;

void cli_init(void);

uint8_t registration(const char *command, const char *declaration, void (*function)(uint8_t argc, const char *argv[]));

uint8_t CLI_Registration_command(CLI_Command_t *command_def);

uint8_t CLI_Process_Command(const char *cli_in_buffer);
void CLI_Parse_Arguments(const char *cli_in_buffer, uint8_t *argc,
		const char *argv[]);
uint8_t CLI_Get_Argument_Length(const char *arg);
uint8_t Parse_Integer(const char *param, int32_t *value);
uint8_t scan_cmd(const char *argv, const char *fmt);
void Help_show(void);

#endif
