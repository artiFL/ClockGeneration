#include "cli.h"
#include "string.h"
#include "stdio.h"

#define MAX_COMMANDS 40
#define MAX_ARGS_IN_CMD 255

static uint8_t max_command = 20;
static uint16_t Command_Count = 0;
//extern char str_rx[40];

static CLI_Command_t *Command_List;


uint8_t scan_cmd(const char *argv, const char *fmt)
{
	return !strncmp(argv, fmt, strlen(fmt));
}

void Help_show(void)
{
	static uint16_t count = 0;
	while (count < Command_Count)
	{
		CLI_Command_t command_list_ptr = Command_List[count];
		printf("\r\n%-20s: %s\r\n", command_list_ptr.CLI_Command,
				command_list_ptr.CLI_Command_Description);
		count++;
	}
	count = 0;
}

void cli_init(void)
{
	Command_List = malloc(sizeof(CLI_Command_t) * max_command);
}

uint8_t registration(const char *command, const char *declaration, void (*function)(uint8_t argc, const char *argv[]))
{
	CLI_Command_t Definition =
	{
		.CLI_Command = command,
		.CLI_Command_Description = declaration,
		.CLI_Command_Length = 0,
		.CLI_Callback = (void*) function };

	if (Command_Count < MAX_COMMANDS)
	{
		Definition.CLI_Command_Length = strlen(command);
		Command_List[Command_Count] = Definition;
		Command_Count++;
	}
	return 0;
}


uint8_t CLI_Registration_command(CLI_Command_t *command_def)
{
	if (Command_Count < MAX_COMMANDS)
	{
		command_def->CLI_Command_Length = strlen(command_def->CLI_Command);
		Command_Count++;
		return 1; // command add successful
	}

	return 0;
}

uint8_t CLI_Process_Command(const char *cli_in_buffer)
{
	uint8_t is_command_valid = 0;
	const char *argv[MAX_ARGS_IN_CMD];
	uint8_t argc = CLI_Get_Argument_Length(argv);

	CLI_Command_t *command_list_ptr = NULL;

	/* Search for the command string in the list of registered commands. */
	for (uint16_t i = 0; i < Command_Count; i++)
	{
		command_list_ptr = &Command_List[i];
		uint16_t cmd_len = command_list_ptr->CLI_Command_Length;

		if ((cli_in_buffer[cmd_len] == ' ') || (cli_in_buffer[cmd_len] == 0x00))
		{
			if (strncmp(cli_in_buffer, command_list_ptr->CLI_Command, cmd_len)
					== 0)
			{
				is_command_valid = 1;
				/* command found break the loop */
				break;
			}
		}
	}

	if (is_command_valid)
	{
		if (command_list_ptr->CLI_Callback != NULL)
		{
			CLI_Parse_Arguments(cli_in_buffer, &argc, argv);

			command_list_ptr->CLI_Callback(argc, argv);
		}
	}

	return 0;
}

void CLI_Parse_Arguments(const char *cli_in_buffer, uint8_t *argc,
		const char *argv[])
{
	uint8_t argc_temp = 0;
	/* arg 0 is input command */
	argv[argc_temp++] = cli_in_buffer;

	while (argc_temp < MAX_ARGS_IN_CMD)
	{
		while (((*cli_in_buffer) != 0x00) && ((*cli_in_buffer) != ' '))
		{
			cli_in_buffer++;
		}

		while ((*cli_in_buffer) == ' ')
		{
			/* convert ' ' to NULL */
			//*cli_in_buffer = 0x00;
			cli_in_buffer++;
		}

		if (*cli_in_buffer != 0x00)
		{
			argv[argc_temp++] = cli_in_buffer;
		}
		else
		{
			break;
		}
	}

	*argc = argc_temp;
}

uint8_t CLI_Get_Argument_Length(const char *arg)
{
	uint8_t len = 0;
	while (((*arg) != 0x00) && ((*arg) != ' '))
	{
		arg++;
		len++;
	}
	return len;
}

